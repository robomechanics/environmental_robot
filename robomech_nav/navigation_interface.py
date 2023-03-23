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
		
		self.NavigationStatus = 0
		self.managerListener = rospy.Subscriber("/target_odom", Odometry, listener1)
		self.transformListener = rospy.Subscriber("/transform_data", Odometry, listener2)
		self.navStatus = rospy.Publisher("navigation_status_for_manager", Odometry, queue_size=1)
		self.goal = MoveBaseGoal()
		self.goalFinished = True
		self.wantsNextCommand = True
		self.goalStatusManager = 0
		self.thetaOffset = 0
		
		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			if goalFinished and wantsNextCommand:
				self.grab_goal()
				self.send_goal()				
			rate.sleep()
			
	def listener1(data,self);
		self.x_UTM = data.pose.pose.position.x
		self.y_UTM = data.pose.pose.position.y
		
	def listener2(data,self);
		self.x_UTM_initial = data.pose.pose.position.x
		self.y_UTM_initial = data.pose.pose.position.y
		self.goal_orientation = data.pose.pose.orientation
	
	def grab_goal(self):
		self.goal = MoveBaseGoal()
		self.goal.target_pose.header.frame_id = "utm_odom2"
		self.goal.target_pose.header.stamp = rospy.Time.now()
		self.goal.target_pose.pose.position.x = self.x_UTM-self.x_UTM_initial
		self.goal.target_pose.pose.position.y = self.y_UTM-self.y_UTM_iniital
		self.goal.target_pose.pose.orientation = self.goal_orientation
	
	def send_goal(self):
		self.client.send_goal(goal,self.done_callback)
		self.goalFinished = False
	
	def done_callback(self,status,result):
		self.goalStatusRaw = status
		self.goalResultRaw = result
		
		if status == 3:
			self.goalStatusManager = 1 #Made it to the goal successfully!
		else if status == 4:
			self.goalStatusManager = 2 #Goal was aborted due to some failure
		else if status == 5:
			self.goalStatusManager = 3 #Unattainable or invalid goal
	
	def check_goal_status(self):
		
	
if __name__ = '__main__':
	NavigationInterface()
