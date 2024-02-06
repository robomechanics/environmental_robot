#!/usr/bin/env python3

import time
import math
import hebi
import rospy

from std_msgs.msg import Header, Float64
from sensor_msgs.msg import JointState

class RototillerFeedback:

	def __init__(self):

		# start initialization
		rospy.init_node('RototillerFeedback', anonymous=True)
		rospy.logwarn("initializing rototiller fbk interface...")

		# initialize Hebi group
		lookup = hebi.Lookup()	
		family = ["R8-16"]
		names = ["JR_rototiller"]
		rotoGroup = lookup.get_group_from_names(family, names)
		if rotoGroup == None:
			rospy.logerr("rototiller Hebi module not connected, failed to initialize rototiller fbk interface")
			return
		
		# set up publisher
		jsPub = rospy.Publisher('rototiller_hebi_fbk', JointState, queue_size=10)
		
		# get initial state values
		hebiFbk = rotoGroup.get_next_feedback(timeout_ms=1000)
		
		# complete initialization
		rospy.logwarn("rototiller fbk interface initialized")	
		
		# publish states indefinitely 
		updateRate = 0.01
		while not rospy.is_shutdown():
			fbk = hebiFbk
			hebiFbk = rotoGroup.get_next_feedback(reuse_fbk=hebiFbk)
			if hebiFbk is None:
				hebiFbk = fbk
			jsFbk = JointState()
			jsFbk.header = Header()
			jsFbk.header.stamp = rospy.Time.now()
			jsFbk.name = names
			jsFbk.position = hebiFbk.position
			jsFbk.velocity = hebiFbk.velocity
			jsFbk.effort = hebiFbk.effort
			jsPub.publish(jsFbk)
			time.sleep(updateRate)

if __name__ == '__main__':
	node = RototillerFeedback()








