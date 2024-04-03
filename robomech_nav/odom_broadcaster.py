#!/usr/bin/env python2
import rospy
import numpy as np

#import tf
import tf2_ros, tf2_geometry_msgs
import geometry_msgs.msg
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

class OdomBroadcaster(object):

	def __init__(self):

		self.odomX = 0
		self.odomY = 0
		self.odomQuaternion = np.zeros((1,4))

		# Creating a transform and its broadcaster	
		self.broadcaster = tf2_ros.TransformBroadcaster()
		self.transform = geometry_msgs.msg.TransformStamped()

		rospy.init_node('odomBroadcaster', anonymous=True)
		self.odomSubscriber = rospy.Subscriber('/rr_openrover_driver/odom_encoder', Odometry, self.odomCallback)

		rospy.spin()

	def odomCallback(self,data):
		# data is a ROS Odometry message
		self.odomX = data.pose.pose.position.x
		self.odomY = data.pose.pose.position.y
		self.odomQuaternion[0,0] = data.pose.pose.orientation.x
		self.odomQuaternion[0,1] = data.pose.pose.orientation.y
		self.odomQuaternion[0,2] = data.pose.pose.orientation.z
		self.odomQuaternion[0,3] = data.pose.pose.orientation.w
		
		# fill out the transform message from odom to base_link
		self.transform.header.stamp = data.header.stamp
		self.transform.header.frame_id = "odom"
		self.transform.child_frame_id = "base_link"
		
		self.transform.transform.translation.x = self.odomX
		self.transform.transform.translation.y = self.odomY
		self.transform.transform.translation.z = 0
		
		self.transform.transform.rotation.x = self.odomQuaternion[0,0]
		self.transform.transform.rotation.y = self.odomQuaternion[0,1]
		self.transform.transform.rotation.z = self.odomQuaternion[0,2]
		self.transform.transform.rotation.w = self.odomQuaternion[0,3]

		# send the transform
		self.broadcaster.sendTransform(self.transform)



if __name__ == '__main__':
	# make an OdomBroadcaster object
	OdomBroadcaster()

