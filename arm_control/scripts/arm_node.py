#!/usr/bin/env python3

import time
import math
import hebi
import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

# run the Hebi arm through ROS	
# continuously query the arm for feedback
# publish said feedback to '/hebi_joint_states' topic
# push commands from '/hebi_arm_commands' topic to hardware

class ArmFeedback:

	def __init__(self, armGroup):
		rospy.init_node('ArmFeedback', anonymous=True)
		jsPub = rospy.Publisher('hebi_joint_states', JointState, queue_size=10)
		updateRate = 0.01
		armFbk = armGroup.get_next_feedback(timeout_ms=1000)

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
			time.sleep(updateRate)

class ArmCommand:

	def __init__(self, armGroup):

		rospy.init_node('ArmCommand', anonymous=True)
		self.armGroup = armGroup
		rospy.Subscriber('hebi_arm_commands', JointState, self.sendCommand, queue_size=1) # no pileup of commands
		print("ArmCommand initialized")
		rospy.spin()

	def sendCommand(self, data):

		# load commands into Hebi group_command then publish to topic AND send to arm
		command = hebi.GroupCommand(self.armGroup.size)
		command.position = data.position
		command.velocity = data.velocity
		command.effort = data.effort

		# send the command to the arm
		print("about to send command")
		# self.armGroup.send_command(command)

if __name__ == '__main__':

	# initialize Hebi to Python connection
	lookup = hebi.Lookup()	
	family = ["Arm_RML"] # DOUBLE CHECK THIS
	names = ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist"]
	armGroup = lookup.get_group_from_names(family, names)
	
	print("arm initialized")

	# run the nodes
	cmdNode = ArmCommand(armGroup)
	print("this line runs")
	fbkNode = ArmFeedback(armGroup)



	
