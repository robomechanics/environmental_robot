#!/usr/bin/env python3

import time
import math
import hebi
import rospy
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

class ArmFeedback:

	def __init__(self):
			
		rospy.init_node('ArmFeedback', anonymous=True)
		rospy.logwarn("initializing arm fbk interface...")

		# initialize Hebi to Python connection
		lookup = hebi.Lookup()	
		family = ["Arm_RML"]
		names = ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist"]
		armGroup = lookup.get_group_from_names(family, names)
		if armGroup == None:
			rospy.logerr("arm not connected, failed to initialize arm fbk interface")
			return
		jsPub = rospy.Publisher('hebi_joint_states', JointState, queue_size=10)
		updateRate = 0.01
		armFbk = armGroup.get_next_feedback(timeout_ms=1000)
		rospy.logwarn("fbk interface initialized")

		printVals = True		
		
		while not rospy.is_shutdown():
			fbk = armFbk
			armFbk = armGroup.get_next_feedback(reuse_fbk=armFbk)
			if armFbk is None:
				armFbk = fbk
			jsFbk = JointState()
			jsFbk.header = Header()
			jsFbk.header.stamp = rospy.Time.now()
			jsFbk.name = names
			jsFbk.position = armFbk.position
			jsFbk.velocity = armFbk.velocity
			jsFbk.effort = armFbk.effort
			jsPub.publish(jsFbk)
			if printVals:
				print("Pos Vec:", jsFbk.position[1:])
				print("Sum of Angles: ",np.sum(jsFbk.position[1:])/np.pi)

			time.sleep(updateRate)

if __name__ == '__main__':
	node = ArmFeedback()
