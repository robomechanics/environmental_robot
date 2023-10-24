#!/usr/bin/env python3
# Test comment

import os
import hebi
import numpy as np
from time import time, sleep
from enum import Enum, auto
from scipy.spatial.transform import Rotation as R

from camera import HebiCamera

import rospy
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from gps_navigation.srv import SetString, SetStringRequest

import typing
if typing.TYPE_CHECKING:
    from typing import Sequence, Optional
    import numpy.typing as npt
    from hebi._internal.mobile_io import MobileIO
    from hebi._internal.group import Group
    from hebi._internal.trajectory import Trajectory
    from hebi.robot_model import RobotModel


class HebiCameraMast:
    default_hrdf_file = os.path.join(os.path.dirname(__file__), 'pan_tilt_mast.hrdf')

    def __init__(self, pan_tilt_group: 'Group', hrdf: 'RobotModel | str' = default_hrdf_file):
        self.group = pan_tilt_group
        self.fbk = self.group.get_next_feedback()
        while self.fbk is None:
            self.fbk = self.group.get_next_feedback()
        self.cmd = hebi.GroupCommand(self.group.size)
        self.trajectory: 'Optional[Trajectory]' = None

        if isinstance(hrdf, str):
            self.robot_model = hebi.robot_model.import_from_hrdf(hrdf)
        else:
            self.robot_model = hrdf

        self.output_frame = np.empty((4,4), dtype=np.float64)

    def update(self, t_now: float):
        self.group.get_next_feedback(reuse_fbk=self.fbk)

        wxyz = self.fbk[0].orientation
        xyzw = [*wxyz[1:], wxyz[0]]

        base_frame = np.eye(4)
        base_frame[:3, :3] = R.from_quat(xyzw).as_matrix()
        self.robot_model.base_frame = base_frame
        self.robot_model.get_end_effector(self.fbk.position, output=self.output_frame)

        # execute current trajectory
        if self.trajectory is not None:
            p, v, _ = self.trajectory.get_state(t_now)
            self.cmd.position = p
            self.cmd.velocity = v


    def send(self):
        self.group.send_command(self.cmd)

    def set_velocity(self, t_now: float, duration: float, velocity: 'Sequence[float]'):

        positions = np.empty((2,2))
        velocities = np.empty((2,2))
        accelerations = np.empty((2,2))

        if self.trajectory is None:
            positions[:, 0] = self.fbk.position
            velocities[:, 0] = self.fbk.velocity
            accelerations[:, 0] = 0
        else:
            p, v, a = self.trajectory.get_state(t_now)
            positions[:, 0] = p
            velocities[:, 0] = v
            accelerations[:, 0] = a

        positions[:, 1] = np.nan

        velocities[0, 1] = velocity[0]
        velocities[1, 1] = velocity[1]

        accelerations[:, 1] = 0.0

        self.trajectory = hebi.trajectory.create_trajectory([t_now, t_now+duration], positions, velocities, accelerations)

    def set_position(self, t_now: float, duration: float, position: 'Sequence[float]'):
        p = np.empty((2,2))
        p[:, 0] = self.fbk.position
        p[:, 1] = position
        self.trajectory = hebi.trajectory.create_trajectory([t_now, t_now+duration], p)


class MastControlState(Enum):
    STARTUP = auto()
    HOMING = auto()
    TELEOP = auto()
    DISCONNECTED = auto()
    EXIT = auto()

class MastInputs:
    v_pan: float
    v_tilt: float
    goto_pose: 'int|None'
    flood_light: float

    def __init__(self, sliders: 'list[float]', goto_pose: 'str|None', flood: float):
        self.pan = sliders[0]
        self.tilt = sliders[1]
        self.goto_pose = goto_pose
        self.flood_light = flood


class MastControl:
    PAN_SCALING = 0.5
    TILT_SCALING = -0.5

    def __init__(self, camera_mast: HebiCameraMast, camera: HebiCamera, home_poses={}):
        self.state = MastControlState.STARTUP
        self.mast = camera_mast
        self.camera = camera

        self.last_input_time = time()

        self.home_poses = home_poses
        self.current_pose = None

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def send(self):
        self.mast.send()
        self.camera.send()

    def update(self, t_now: float, inputs: 'Optional[MastInputs]'):
        self.mast.update(t_now)
        self.camera.update()
        if not inputs:
            if t_now - self.last_input_time > 1.0 and self.state is not self.state.DISCONNECTED:
                print("Warning, lost signal to Mobile IO, going into safety state!")
                self.transition_to(t_now, self.state.DISCONNECTED)
            return

        self.last_input_time = t_now
        if self.state is self.state.DISCONNECTED:
            self.transition_to(t_now, self.state.TELEOP)

        self.camera.spot_light = inputs.flood_light

        if self.state is self.state.STARTUP:
            self.transition_to(t_now, self.state.HOMING, 'drive')

        elif self.state is self.state.HOMING:
            if t_now > self.mast.trajectory.end_time:
                self.transition_to(t_now, self.state.TELEOP)

        elif self.state is self.state.TELEOP:
            if inputs.goto_pose is not None:
                self.transition_to(t_now, self.state.HOMING, inputs.goto_pose)
            else:
                Δpan  = inputs.pan * self.PAN_SCALING
                Δtilt = inputs.tilt * self.TILT_SCALING
                Δt = 0.25 # sec

                if Δpan != 0.0 or Δtilt != 0.0:
                    self.current_pose = None

                self.mast.set_velocity(t_now, Δt, [Δpan, Δtilt])

    def transition_to(self, t_now: float, new_state: MastControlState, *args):
        if new_state is self.state:
            return

        if new_state is self.state.DISCONNECTED:
            print("Transitioning to Disconnected")
            self.mast.trajectory = None
            self.mast.cmd.velocity = None
            self.current_pose = None

        elif new_state is self.state.HOMING:
            pose_name = args[0]
            if pose_name not in self.home_poses:
                print(f"Cannot find pose {pose_name}")
            else:
                print(f"Transitioning to Homing (Pose {pose_name})")
                self.current_pose = pose_name
                self.mast.set_position(t_now, 3.0, self.home_poses[pose_name])

        elif new_state is self.state.TELEOP:
            print("Transitioning to Manual Control")

        elif new_state is self.state.EXIT:
            print("Transitioning to Exit")

        self.state = new_state


if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    rospy.init_node('camera_ctrl')

    # Setup Camera Pan/Tilt
    family = "Chevron"
    module_names = ['C1_pan', 'C2_tilt']

    group = lookup.get_group_from_names(family, module_names)
    while group is None:
        print('Looking for pan/tilt modules...')
        sleep(1)
        group = lookup.get_group_from_names(family, module_names)

    cam_group = lookup.get_group_from_names('Chevron', ['Widey'])
    while cam_group is None:
        print('Looking for camera...')
        sleep(1)
        cam_group = lookup.get_group_from_names('Chevron', ['Widey'])

    mast = HebiCameraMast(group)
    camera = HebiCamera(cam_group)

    poses = {'front': [-0.7, 2.2], 'rear': [0.55, 4.15], 'drive': [0.0, 2.0]}
    mast_control = MastControl(mast, camera, poses)

    light_level = 0.0
    def light_cb(msg):
        global light_level
        light_level = msg.data

    rospy.Subscriber('~light', Float64, light_cb)

    pan_tilt_twist = Twist()
    def move_cb(msg):
        global pan_tilt_twist
        pan_tilt_twist = msg

    rospy.Subscriber('pan_tilt_vel', Twist, move_cb)

    view_pub = rospy.Publisher('~pose', String, queue_size=3, latch=True)
    pose_str = String()
    def publish_pose(evt):
        global pose_str
        if mast_control.current_pose is None:
            pose_str.data = 'roam'
        else:
            pose_str.data = mast_control.current_pose
        view_pub.publish(pose_str)
    rospy.Timer(rospy.Duration.from_sec(1.0), publish_pose)

    def goto_cb(req: SetStringRequest):
        global pose_str
        if mast_control.state != mast_control.state.TELEOP:
            return [False, 'called too soon']

        mast_control.transition_to(rospy.get_time(), mast_control.state.HOMING, req.text)
        view_pub.publish(String(req.text))

        return [True, req.text]

    rospy.Service('~goto_pose', SetString, goto_cb)

    #######################
    ## Main Control Loop ##
    #######################

    cmd = hebi.GroupCommand(1)
    while mast_control.running and not rospy.is_shutdown():
        t = rospy.get_time()

        pan = pan_tilt_twist.angular.z
        tilt = pan_tilt_twist.angular.x
        mast_control.update(t, MastInputs([pan, tilt], None, light_level))

        mast_control.send()
        camera.send()
