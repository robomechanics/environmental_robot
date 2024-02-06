#!/usr/bin/env python3

import rospy
import time
import math
import os
import subprocess
import yaml
from anakin_control.msg import MotorMsg, FeedbackMsg

class MotorFeedback:

	def __init__(self):

		# initialize node
		rospy.init_node('MotorFeedback', anonymous=True)

		# initialize publisher
		self.pubFbk = rospy.Publisher('rototiller_speed_feedback', FeedbackMsg, queue_size=10)


		# initialize subscriber
		rospy.Subscriber('rototiller_speed_commands', MotorMsg, self.convertFeedback)

		# spin
		rospy.spin()

	def convertFeedback(self, data):

		statusRaw = subprocess.check_output(['jrk2cmd'] + ['-s'])
		status = yaml.safe_load(statusRaw)
		feedback = status['Feedback']

		pulseTimingClock = 1500000 # [Hz]
		freqDivider = 32
		reductionRatio = 24

		numerator = (2*math.pi*pulseTimingClock)
		denominator = 24*((2**26)/(freqDivider*(feedback - 2048)))
		speed = int(numerator//denominator)
		print(speed)	

		self.pubFbk.publish(FeedbackMsg(speed))



if __name__ == '__main__':

	node = MotorFeedback()