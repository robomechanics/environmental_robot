#!/usr/bin/env python3

import time
import rospy
import subprocess
import yaml

from std_msgs.msg import Float32

class JrkFeedback:
	def __init__(self):

		# start initialization
		rospy.init_node('JrkFeedback', anonymous=True)
		rospy.logwarn("initializing jrk fbk interface...")

		# set up publisher
		self._lipo_volt_topic = rospy.get_param('lipo_battery_voltage_topic')
		self._lipo_current_topic = rospy.get_param('lipo_current_topic')
		self.lipo_battery_refresh_interval = rospy.get_param('lipo_battery_info_refresh_interval')

		lipoPub = rospy.Publisher(self._lipo_volt_topic, Float32, queue_size=1)
		currentPub = rospy.Publisher(self._lipo_current_topic, Float32, queue_size=1)
  
		# get initial value
		try:
			statusRaw = subprocess.check_output(['jrk2cmd'] + ['-s'])
		except:
			rospy.logerr("jrk module not connected, failed to initialize jrk interface")
			return
		status = yaml.safe_load(statusRaw)
		VIN = float(status['VIN voltage'][0:-2])
		current = float(status['Current'][0:-3])
		rospy.logwarn(status)
		rospy.logwarn("Battery voltage: %f", VIN)

		# publish state indefinitely 
		while not rospy.is_shutdown():
			statusRaw = subprocess.check_output(['jrk2cmd'] + ['-s'])
			status = yaml.safe_load(statusRaw)
			VIN = float(status['VIN voltage'][0:-2])
			current = float(status['Current'][0:-3])
   
			lipoPub.publish(Float32(VIN))
			currentPub.publish(Float32(current))
   
			time.sleep(self.lipo_battery_refresh_interval)

if __name__ == '__main__':
	node = JrkFeedback()