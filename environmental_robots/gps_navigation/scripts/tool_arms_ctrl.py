#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolRequest
from std_msgs.msg import Float64

import numpy as np
import hebi

from enum import Enum, auto

class ToolMode(Enum):
    Position = auto()
    Torque = auto()


if __name__ == '__main__':
    rospy.init_node('tool_arms_ctrl')

    lookup = hebi.Lookup()
    rospy.sleep(2)

    family = 'Chevron'
    names = ['J1_rake']
    tool_arm = lookup.get_group_from_names(family, names)
    while tool_arm is None:
        tool_arm = lookup.get_group_from_names(family, names)

    tool_arm_cmd = hebi.GroupCommand(tool_arm.size)

    tool_arm_fbk = tool_arm.get_next_feedback()

    tool_ctrl_mode = ToolMode.Position
    tool_angle_down = 1.0
    tool_angle_up = 2.5


    family = 'Chevron'
    names = ['J1_shoulder', 'J2_elbow']
    sensor_arm = lookup.get_group_from_names(family, names)
    while sensor_arm is None:
        sensor_arm = lookup.get_group_from_names(family, names)

    sensor_arm_cmd = hebi.GroupCommand(sensor_arm.size)

    sensor_arm_fbk = sensor_arm.get_next_feedback()

    sensor_angles_down = [-1.0, -1.0]
    sensor_angles_up = [1.0, 1.0]

    t = rospy.get_time()
    tool_arm_trajectory = hebi.trajectory.create_trajectory([t, t+3], [tool_arm_fbk.position[0], tool_angle_up])

    positions = np.empty((2,2), dtype=np.float64)
    positions[:, 0] = sensor_arm_fbk.position
    positions[:, 1] = sensor_angles_up
    sensor_arm_trajectory = hebi.trajectory.create_trajectory([t, t+3], positions)


    def tool_arm_cb(request: SetBoolRequest):
        global tool_arm_trajectory, tool_ctrl_mode
        t = rospy.get_time()
        times = [t, t+1]
        curr_pos = tool_arm_fbk.position[0]

        if request.data:
            tool_ctrl_mode = ToolMode.Torque
        else:
            tool_ctrl_mode = ToolMode.Position
            tool_arm_trajectory = hebi.trajectory.create_trajectory(times, [curr_pos, tool_angle_up])
        return []

    
    def sensor_arm_cb(request):
        global sensor_arm_trajectory
        t = rospy.get_time()
        times = [t, t+2]
        positions = np.empty((2, 2), dtype=np.float64)

        if request.data:
            positions[:, 0] = sensor_arm_fbk.position
            positions[:, 1] = sensor_angles_down
        else:
            positions[:, 0] = sensor_arm_fbk.position
            positions[:, 1] = sensor_angles_up

        sensor_arm_trajectory = hebi.trajectory.create_trajectory(times, positions)
        return []

    rospy.Service('deploy_tool', SetBool, tool_arm_cb)
    rospy.Service('deploy_sensor', SetBool, sensor_arm_cb)

    dig_torque = 0.0
    def dig_cb(msg):
        global dig_torque
        dig_torque = msg.data

    rospy.Subscriber('dig_torque', Float64, dig_cb)

    while not rospy.is_shutdown():
        t = rospy.get_time()
        sensor_arm.get_next_feedback(reuse_fbk=sensor_arm_fbk)
        tool_arm.get_next_feedback(reuse_fbk=tool_arm_fbk)

        p, v, a = sensor_arm_trajectory.get_state(t)
        sensor_arm_cmd.position = p
        sensor_arm_cmd.velocity = v
        sensor_arm.send_command(sensor_arm_cmd)

        #print(f'p: {p}, v: {v}')
        if tool_ctrl_mode == ToolMode.Position:
            p, v, a = tool_arm_trajectory.get_state(t)
            tool_arm_cmd.position = p
            tool_arm_cmd.velocity = v
            tool_arm_cmd.effort = np.nan
        else:
            tool_arm_cmd.position = np.nan
            tool_arm_cmd.velocity = np.nan
            tool_arm_cmd.effort = dig_torque

        tool_arm.send_command(tool_arm_cmd)
