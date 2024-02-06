#!/usr/bin/env python3

import rospy
import time
import math
import hebi
import os
import subprocess
import yaml
import numpy as np 
from std_msgs.msg import Header, Float64, Int64
from anakin_control.msg import ModuleMsg, MotorMsg, FeedbackMsg

class ModuleCommand:

	def __init__(self):

		# initialize node
		rospy.init_node('ModuleCommand', anonymous=True)
		rospy.logwarn("initializing module node...")

		# set up publisher to set up publishers (uhhhhhh)
		moduleHoldPub = rospy.Publisher('single_msgs', FeedbackMsg, queue_size=10)
		self.rototillerHeightPub = rospy.Publisher('rototiller_height_feedback', Float64, queue_size=10)

		time.sleep(3)

		# start continuous publication from command_node.py
		moduleHoldPub.publish(FeedbackMsg(0))

		useAuger = False

		# set up module groups
		lookup = hebi.Lookup()
		familyR = ['R8-16']
		nameR = ['JR_rototiller']
		self.rototillerGroup = lookup.get_group_from_names(familyR, nameR)
		if useAuger:
			familyA = ['R8-16']
			nameA = ['JA_auger']
			self.augerGroup = lookup.get_group_from_names(familyA, nameA)
		
		# Hebi module initial positions
		rototiller_fbk = self.rototillerGroup.get_next_feedback(timeout_ms=10000)
		rospy.logwarn("rototiller height discovered")
		if useAuger:
			auger_fbk = self.augerGroup.get_next_feedback(timeout_ms=10000)
			rospy.logwarn("auger position discovered")

		# Set Hebi module gains
		"""group_info = self.rototillerGroup.request_info()
		print(group_info.position_kp)
		cmd = hebi.GroupCommand(self.rototillerGroup.size)
		cmd.position_kp = 40
		self.rototillerGroup.send_command(cmd)"""

		# variables
		self.motorSpeed = 0
		self.update_rate = 0.05
		self.rPert = 0.0025
		self.aPert = 0.00075
		self.rototillerHeight = rototiller_fbk.position[0]
		self.rototillerHeightMax = 0.3
		if useAuger:
			self.augerPos = auger_fbk.position[0]

		# set rototiller motor duty cycle cap
		statusRaw = subprocess.check_output(['jrk2cmd'] + ['-s'])
		status = yaml.safe_load(statusRaw)
		VIN = status['VIN voltage']
		VIN = float(VIN[0:-2])
		rospy.logwarn("battery voltage: %f", VIN)
		if VIN > 24:
			self.dutyCycleMult = round(23/VIN, 2)
		else:
			self.dutyCycleMult = 1
		rospy.logwarn("duty Cycle limit: %f percent", self.dutyCycleMult*100)

		# run rototiller motor
		os.system('jrk2cmd --target 2048')
		os.system('jrk2cmd --run')

		# initialize subscribers
		rospy.Subscriber('rototiller_height_commands', ModuleMsg, self.pushRototillerHeightCommand) # , ('roto')
		rospy.Subscriber('other_topic', MotorMsg, self.pushMotorCommand)
		if useAuger:
			rospy.logwarn("created auger subscriber")
			rospy.Subscriber('auger_commands', ModuleMsg, self.pushAugerCommand) # , ('auger')

		# spin
		rospy.logwarn("module node initialized")
		rospy.spin()

	def pushRototillerHeightCommand(self, data): # , args

		# push ModuleMsg data from ROS topic to Hebi command struct, send command

		"""device = args[0]
		
		if device == 'auger':
			print('hit')
			group = self.augerGroup
			self.augerPos += data.direction*self.aPert
			pos = self.augerPos
			
		else:"""
		group = self.rototillerGroup
		if self.rototillerHeight < self.rototillerHeightMax and data.direction > 0:
			self.rototillerHeight += self.rPert
		elif data.direction < 0:
			self.rototillerHeight -= self.rPert
		pos = self.rototillerHeight
		self.rototillerHeightPub.publish(Float64(self.rototillerHeight))
	
		group_command = hebi.GroupCommand(group.size)
		group_command.position = pos
		group_command.velocity = data.velocity
		group_command.effort = data.effort
		group.send_command(group_command)

	def pushAugerCommand(self, data): # , args

		# push ModuleMsg data from ROS topic to Hebi command struct, send command

		# device = args[0]
		
		# if device == 'auger':
		group = self.augerGroup
		self.augerPos += data.direction*self.aPert
		pos = self.augerPos
			
		"""else:
			group = self.rototillerGroup
			if self.rototillerHeight < self.rototillerHeightMax and data.direction > 0:
				self.rototillerHeight += self.rPert
			elif data.direction < 0:
				self.rototillerHeight -= self.rPert
			pos = self.rototillerHeight"""
	
		group_command = hebi.GroupCommand(group.size)
		group_command.position = pos
		group_command.velocity = data.velocity
		group_command.effort = data.effort
		group.send_command(group_command)

	def pushMotorCommand(self, data):
		# push data from speed_commands to jrk2cmd interface
		if data.stop:
			os.system('jrk2cmd --stop')
		else:
			# switch negative sign to reverse motor direction
			# rospy.logwarn(data.speed)
			target = 5 * round((-1*data.speed*self.dutyCycleMult)/5)

			rate = 0.01
			# speed = self.motorSpeed

			"""# speed up
			while speed < target:
				speed += 5
				os.system('jrk2cmd --target %d' % (speed + 2048))
				time.sleep(rate)

			# slow down
			while speed > target:
				speed -= 5
				os.system('jrk2cmd --target %d' % (speed + 2048))
				time.sleep(rate)"""

			speed = target
			self.motorSpeed = speed
			os.system('jrk2cmd --target %d' % (data.speed + 2048))
			time.sleep(rate)


		"""statusRaw = subprocess.check_output(['jrk2cmd'] + ['-s'])
		status = yaml.safe_load(statusRaw)
		# feedback = status['Feedback']
		# print(feedback)
		voltage = status['VIN voltage']
		# print(voltage)"""


	def test(self):

		speed = 0
		target = 400
		timeToTarget = 1
		rate = timeToTarget/target # seconds/tick

		# run the motor
		os.system('jrk2cmd --run')

		# build to speed
		while speed < target:
			speed += 2
			os.system('jrk2cmd --speed %d' % (speed))
			time.sleep(rate)

		# hold speed
		print("full speed ahead")
		time.sleep(3)
		print("slowing...")

		# slow to a stop
		while speed > 0:
			speed -= 2
			os.system('jrk2cmd --speed %d' % (speed))
			time.sleep(rate)

		print("stopped")

		# stop the motor
		os.system('jrk2cmd --stop')

if __name__ == '__main__':

	node = ModuleCommand()
	# node.test()
