#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Header
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def initialize():
	rospy.init_node('showStates', anonymous=True)
	rospy.Subscriber('joint_states', JointState, printStates)
	rospy.spin()

def printStates(data):
	print(data.position[0])
	print(data.position[1])
	print(data.position[2])
	print(data.position[3])
	print("---------------------------------------------------")

if __name__ == '__main__':
	initialize()
