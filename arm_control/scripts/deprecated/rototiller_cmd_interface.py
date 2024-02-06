#!/usr/bin/env python3

import time
import math
import hebi
import rospy
import os
import subprocess

from std_msgs.msg import Header, Int16
from sensor_msgs.msg import JointState

class RototillerCommand:

	def __init__(self):
		
		rospy.init_node('RototillerCommand', anonymous=True)
		rospy.logwarn("initializing rototiller cmd interface...")

		# initialize Hebi group
		lookup = hebi.Lookup()	
		family = ["R8-16"]
		names = ["JR_rototiller"]
		self.rotoGroup = lookup.get_group_from_names(family, names)
		if self.rotoGroup == None:
			rospy.logerr("rototiller Hebi module not connected, failed to initialize rototiller cmd interface")
			return

		# keep track of motor speed, only update when necessary
		self.speed = 0

		# set up subscribers
		rospy.Subscriber('rototiller_height_commands', JointState, self.commandHebi, queue_size=10)
		rospy.Subscriber('rototiller_speed_commands', Int16, self.updateMotorSpeed, queue_size=10) # percentage out of 100, can be negative

		# spin
		rospy.logwarn("rototiller cmd interface initialized")
		rospy.spin()

	def commandHebi(self, data):

		# load commands into Hebi group_command then publish to topic AND send to arm
		command = hebi.GroupCommand(self.rotoGroup.size)
		command.position = data.position
		command.velocity = data.velocity
		command.effort = data.effort

		# send the command to the arm
		self.rotoGroup.send_command(command)

	def updateMotorSpeed(self, data):

		oldSpeed = self.speed

		# round to nearest int on [-600, 600] and send
		speed = round((data.data/100)*600)
		self.speed = speed
		if oldSpeed != self.speed:
			self.commandMotor()
		
	def commandMotor(self):
		try:
			os.system('jrk2cmd --target %d' % (self.speed + 2048))
		except:
			rospy.logwarn("jrk module not connected, failed to command rototiller drive motor")

if __name__ == '__main__':
	node = RototillerCommand()
	
		

		


