#!/usr/bin/env python3
import rospy
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# import tf
# import tf2_ros, tf2_geometry_msgs
# import geometry_msgs.msg
# from sensor_msgs.msg import NavSatFix
# from nav_msgs.msg import Odometry
# from autonomy_manager.srv import NavigateGPS
# from autonomy_manager.srv import Complete

# ------------------------------------

class NavigationInterface(object):
	
	def __init__(self):
	
		rospy.init_node('NavigationInterfaceNode')
		self.goalFlag = False
		self.goalFinished = False
		self.firstTime = True

		# Services
		self.next_goal_nav = rospy.Service('next_goal_nav', OdomGoal, self.setGoal)
		self.cancel_goal_nav = rospy.Service('cancel_goal', OdomGoal, self.cancelGoal)
		
		# Service proxies
		self.goal_reach = rospy.ServiceProxy('goal_reach', Complete)

		# Action client things		
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.client.wait_for_server()
		
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			if self.goalFlag == True: # Check for if next command is wanted. Always false after goal is met.
				self.make_goal()
				self.send_goal()
			rate.sleep()

	def setGoal(self, data):      
		self.goalFlag = True
		self.x_Goal = data.odomX
		self.y_Goal = data.odomY
		return True
		
	def cancelGoal(self, data):
		self.client.cancel_all_goals()
		return True
	
	def make_goal(self):
		self.goal = MoveBaseGoal()
		self.goal.target_pose.header.frame_id = "my_odom"
		self.goal.target_pose.header.stamp = rospy.Time.now()
		self.goal.target_pose.pose.position.x = self.x_Goal
		self.goal.target_pose.pose.position.y = self.y_Goal
		self.goal.target_pose.pose.orientation.x = 0
		self.goal.target_pose.pose.orientation.y = 0
		self.goal.target_pose.pose.orientation.z = 0
		self.goal.target_pose.pose.orientation.w = 1
	
	def send_goal(self):
		self.client.send_goal(self.goal,self.done_callback)
		self.goalFlag = False
		self.goalFinished = False
	
	def done_callback(self,status,result):
		self.goalStatusRaw = status
		self.goalResultRaw = result
		
		if status == 3:
			self.goalStatusManager = 1 #Made it to the goal successfully!
			self.goalFinished = True
			res = self.goal_reach(self.goalFinished)
			print("Goal is met")
		elif status == 4:
			self.goalStatusManager = 2 #Goal was aborted due to some failure
			res = self.goal_reach(self.goalFinished)
			print("Goal failed somehow")
		elif  status == 5:
			self.goalStatusManager = 3 #Unattainable or invalid goal
			res = self.goal_reach(self.goalFinished)
			print("Invalid goal")
	
	def active_callback(self,status,result):
		print("Active Callback")
	
	def fbk_callback(self,status,result):
		print("Feedback Callback")
	
	def check_goal_status(self):
		print("Dummy Goal Status Check")
	
if __name__ == '__main__':
	NavigationInterface()
