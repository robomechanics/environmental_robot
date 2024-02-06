#!/usr/bin/env python3

import time
import math
import rospy
import os
import subprocess
import yaml

from std_msgs.msg import Header, Float32

class JrkFeedback:

	def __init__(self):

		# start initialization
		rospy.init_node('JrkFeedback', anonymous=True)
		rospy.logwarn("initializing jrk fbk interface...")

		# set up publisher
		lipoPub = rospy.Publisher('lipo_battery_voltage', Float32, queue_size=10)
		currentPub = rospy.Publisher('motor_current', Float32, queue_size=10)

		# get initial value
		try:
			statusRaw = subprocess.check_output(['jrk2cmd'] + ['-s'])
		except:
			rospy.logerr("jrk module not connected, failed to initialize jrk fbk interface")
			return
		status = yaml.safe_load(statusRaw)
		# rospy.logwarn(status)
		VIN = status['VIN voltage']
		VIN = float(VIN[0:-2])
		current = status['Current']
		current = float(current[0:-3])
		rospy.logwarn("battery voltage: %f", VIN)

		# complete initialization
		rospy.logwarn("jrk fbk interface initialized")

		# publish state indefinitely 
		updateRate = 0.1
		while not rospy.is_shutdown():
			statusRaw = subprocess.check_output(['jrk2cmd'] + ['-s'])
			status = yaml.safe_load(statusRaw)
			VIN = status['VIN voltage']
			VIN = float(VIN[0:-2])
			current = status['Current']
			current = float(current[0:-3])
			lipoPub.publish(Float32(VIN))
			currentPub.publish(Float32(current))
			time.sleep(updateRate)

if __name__ == '__main__':
	node = JrkFeedback()
					





