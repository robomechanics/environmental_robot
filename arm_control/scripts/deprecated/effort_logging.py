#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np
from time import sleep
import hebi
import rospkg
import math
import matplotlib.pyplot as plt

class RobotController:
    def __init__(self, family_name, module_names, hrdf_path):
        # HEBI setup
        self.lookup = hebi.Lookup()
        sleep(2.0)

        self.group = self.lookup.get_group_from_names([family_name], module_names)   #from HEBI API

        if self.group is None:
            rospy.logerr('Group not found')
            exit(1)

        self.model = hebi.robot_model.import_from_hrdf(hrdf_path)

        # Set the gains
        #gainsCmd = hebi.GroupCommand(self.group.size)
        #rospack = rospkg.RosPack()
        #ak = rospack.get_path('anakin_control')
        #path = ak + '/config/gains/anakin_gains.xml'
		# path = "/home/rover/catkin_ws/src/anakin_control/config/gains/anakin_gains.xml"
        #gainsCmd.read_gains(path)
        #self.group.send_command_with_acknowledgement(gainsCmd)

        self.num_joints = self.group.size
        self.group_fbk = hebi.GroupFeedback(self.num_joints)

    def get_effort(self):

        self.group_fbk = self.group.get_next_feedback(reuse_fbk = self.group_fbk)

        #effort_array = np.zeros(self.num_joints, dtype=np.float32) + 0.1
        #print(effort)
        effort_array = self.group_fbk.effort
        rospy.loginfo('Effort: {0}'.format(effort_array))
        #print(effort_array)


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('robot_control_node')

    # Create an instance of the RobotController class
    robot_controller = RobotController("Arm_RML", ["J1_base", "J2_shoulder", "J3_elbow", 'J4_wrist'], "/home/patrick/catkin_ws/src/anakin_control/config/hrdf/anakin.hrdf")

    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        robot_controller.get_effort()
        rate.sleep()

    rospy.loginfo("Script completed. Shutting down the ROS node.")