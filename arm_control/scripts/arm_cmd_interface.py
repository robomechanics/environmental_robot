#!/usr/bin/env python3

import time
import math
import hebi
import rospy
import rospkg

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from pathlib import Path

class ArmCommand:

	def __init__(self):

		rospy.init_node('ArmCommand', anonymous=True)
		rospy.logwarn("initializing arm cmd interface...")

		# initialize Hebi to Python connection
		lookup = hebi.Lookup()	
		family = ["Arm_RML"]
		names = ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist"]
		self.armGroup = lookup.get_group_from_names(family, names)
		if self.armGroup == None:
			rospy.logerr("arm not connected, failed to initialize arm cmd interface")
			return

		# load gains
		gainsCmd = hebi.GroupCommand(self.armGroup.size)
		rospack = rospkg.RosPack()
		ak = rospack.get_path('anakin_control')
		path = ak + '/config/gains/anakin_gains.xml'
		# path = "/home/rover/catkin_ws/src/anakin_control/config/gains/anakin_gains.xml"
		gainsCmd.read_gains(path)
		self.armGroup.send_command_with_acknowledgement(gainsCmd)		

		rospy.Subscriber('hebi_arm_commands', JointState, self.sendCommand, queue_size=10) # no pileup of commands
		rospy.logwarn("arm cmd interface initialized")
		rospy.spin()

	def sendCommand(self, data):

		# load commands into Hebi group_command then publish to topic AND send to arm
		command = hebi.GroupCommand(self.armGroup.size)
		command.position = data.position
		command.velocity = data.velocity
		command.effort = data.effort

		# send the command to the arm
		self.armGroup.send_command(command)

if __name__ == '__main__':
	node = ArmCommand()
