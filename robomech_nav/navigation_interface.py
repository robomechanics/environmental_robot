#!/usr/bin/env python3
import rospy
import numpy as np
import pyproj
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#import statistics

from pyproj import CRS
from pyproj import Transformer
from pyproj import Proj
from pyproj.database import query_utm_crs_info
from pyproj.aoi import AreaOfInterest

import tf
import tf2_ros, tf2_geometry_msgs
import geometry_msgs.msg
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

# ------------------------------------

class NavigationInterface(object):
	
	def __init__(self):
	
		rospy.init_node('NavigationInterfaceNode')
		
		self.useDummyValues = True
		self.dummyMatrix = np.array([[0,5,5,5,5,0,0,0]])
		self.obstacleCounter = 0
		self.NavigationStatus = 0
		self.navStatus = rospy.Publisher("navstack_status", int64, queue_size=1)
		self.goal = MoveBaseGoal()
		self.goalFinished = True
		self.wantsNextCommand = True
		self.goalStatusManager = 0
		self.thetaOffset = 0
		
		self.xy_iniital = rospy.Subscriber("/utm_start", Odometry, xy_listener)
		self.navStatus = rospy.Publisher("navstack_status", int64, queue_size=1, latch=False)
		#self.grabPoints = rospy.ServiceProxy('grab_points',GrabPoints)
		#self.commandCheck = rospy.ServiceProx('checkForCommand',CommandCheck)
		
		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.client.wait_for_server()
		
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			#self.wantsNextCommand = True
			if self.goalFinished and self.wantsNextCommand:
				self.grab_goal()
				self.send_goal()
				self.navStatus.publish(self.goalStatusManager)
			rate.sleep()
		
	def xy_listener(data,self):
		self.x_UTM_initial = data.pose.pose.position.x
		self.y_UTM_initial = data.pose.pose.position.y
	
	def grab_goal(self):
		if self.useDummyValues == True and self.obstacleCounter < 4:
			self.goal = MoveBaseGoal()
			self.goal.target_pose.header.frame_id = "utm_odom2"
			self.goal.target_pose.header.stamp = rospy.Time.now()
			self.goal.target_pose.pose.position.x = self.dummyMatrix[0][2*self.obstacleCounter]
			self.goal.target_pose.pose.position.y = self.dummyMatrix[0][2*self.obstacleCounter+1]
			self.goal.target_pose.pose.orientation.x = 0
			self.goal.target_pose.pose.orientation.y = 0
			self.goal.target_pose.pose.orientation.z = 0
			self.goal.target_pose.pose.orientation.w = 1
			print(self.goal)
		elif self.useDummyValues == False:
			self.goal = MoveBaseGoal()
			self.goal.target_pose.header.frame_id = "utm_odom2"
			self.goal.target_pose.header.stamp = rospy.Time.now()
			self.goal.target_pose.pose.position.x = self.x_UTM-self.x_UTM_initial
			self.goal.target_pose.pose.position.y = self.y_UTM-self.y_UTM_iniital
			self.goal.target_pose.pose.orientation.x = 0
			self.goal.target_pose.pose.orientation.y = 0
			self.goal.target_pose.pose.orientation.z = 0
			self.goal.target_pose.pose.orientation.w = 1
	
	def send_goal(self):
		self.client.send_goal(self.goal,self.done_callback)
		print("Goal is sent")
		self.goalFinished = False
		self.obstacleCounter = self.obstacleCounter + 1
	
	def done_callback(self,status,result):
		self.goalStatusRaw = status
		self.goalResultRaw = result
		
		if status == 3:
			self.goalStatusManager = 1 #Made it to the goal successfully!
			self.goalFinished = True
			print("Goal is met")
		elif status == 4:
			self.goalStatusManager = 2 #Goal was aborted due to some failure
			print("Goal failed somehow")
		elif  status == 5:
			self.goalStatusManager = 3 #Unattainable or invalid goal
			print("Invalid goal")
	
	def active_callback(self,status,result):
		print("Active Callback")
	
	def fbk_callback(self,status,result):
		print("Feedback Callback")
	
	def check_goal_status(self):
		print("Dummy Goal Status Check")
		
	def status_service(self,data)
		return self.goalStatusManager
	
if __name__ == '__main__':
	NavigationInterface()
