#!/usr/bin/env python3

import os
import hebi
import numpy as np
from time import time, sleep
from hebi.util import create_mobile_io
from enum import Enum, auto
from scipy.spatial.transform import Rotation as R

from camera import HebiCamera

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
    home: bool
    flood_light: float

    def __init__(self, sliders: 'list[float]', home: bool, flood: float):
        self.pan = sliders[0]
        self.tilt = sliders[1]
        self.home = home
        self.flood_light = flood


class MastControl:
    PAN_SCALING = 0.5
    TILT_SCALING = -0.5

    def __init__(self, camera_mast: HebiCameraMast, camera: HebiCamera, home_pose=[0, 0]):
        self.state = MastControlState.STARTUP
        self.mast = camera_mast
        self.camera = camera

        self.last_input_time = time()

        self.home_pose = home_pose

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

        self.camera.flood_light = inputs.flood_light

        if self.state is self.state.STARTUP:
            self.transition_to(t_now, self.state.HOMING)

        elif self.state is self.state.HOMING:
            if t_now > self.mast.trajectory.end_time:
                self.transition_to(t_now, self.state.TELEOP)

        elif self.state is self.state.TELEOP:
            if inputs.home:
                self.transition_to(t_now, self.state.HOMING)
            else:
                Δpan  = inputs.pan * self.PAN_SCALING
                Δtilt = inputs.tilt * self.TILT_SCALING
                Δt = 0.25 # sec

                self.mast.set_velocity(t_now, Δt, [Δpan, Δtilt])

    def transition_to(self, t_now: float, new_state: MastControlState):
        if new_state is self.state:
            return

        if new_state is self.state.DISCONNECTED:
            print("Transitioning to Disconnected")
            self.mast.trajectory = None
            self.mast.cmd.velocity = None

        elif new_state is self.state.HOMING:
            print("Transitioning to Homing")
            self.mast.set_position(t_now, 3.0, self.home_pose)

        elif new_state is self.state.TELEOP:
            print("Transitioning to Manual Control")

        elif new_state is self.state.EXIT:
            print("Transitioning to Exit")

        self.state = new_state


def setup_mobile_io(m: 'MobileIO'):
    m.set_button_label(1, '⟲')
    m.set_button_label(2, 'light')
    m.set_button_mode(2, 1)

    m.set_axis_label(2, 'camera')
    m.set_axis_label(1, '')
    m.set_axis_label(3, '\U0001F4A1')

    # Since this value is rescaled -1:1 -> 0:1, set to "0" position to start
    #m.set_axis_value(3, -1.0)


def parse_mobile_feedback(m: 'MobileIO'):
    if not m.update(0.0):
        return None

    # Update Camera light
    flood_light = 0.0
    if m.get_button_state(2):
        flood_light = (m.get_axis_state(3) + 1.0) / 2.0

    pan  = -1.0 * m.get_axis_state(1)
    tilt = m.get_axis_state(2)

    return MastInputs([pan, tilt], m.get_button_state(1), flood=flood_light)


if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

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

    # Setup MobileIO
    print('Looking for Mobile IO...')
    m = create_mobile_io(lookup, family)
    while m is None:
        print('Waiting for Mobile IO device to come online...')
        sleep(1)
        m = create_mobile_io(lookup, family)

    m.update()
    setup_mobile_io(m)
    
    mast_control = MastControl(mast, camera, home_pose=[-1.57, -1.57])

    #######################
    ## Main Control Loop ##
    #######################
    
    while mast_control.running:
        t = time()
        inputs = parse_mobile_feedback(m)
        mast_control.update(t, inputs)
        # Update mobileIO stream angle
        if inputs:
            # pi rad offset needed for wide angle cameras
            m._cmd.io.c.set_float(1, mast_control.camera.roll + np.pi)
            m._group.send_command(m._cmd)

        mast_control.send()
        camera.send()
