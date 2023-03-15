#!/usr/bin/env python3
import rospy
import numpy as np
import pyrr

# Because of transformations
import tf_conversions
import tf
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

x = 0
y = 0
z = 0

xr = 0
yr = 0
zr = 0
wr = 0

futureTimeIncrement = rospy.Duration.from_sec(0.1)

def world_pose_talker(data):
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	z = data.pose.pose.position.z
	
	xr = data.pose.pose.orientation.x
	yr = data.pose.pose.orientation.y
	zr = data.pose.pose.orientation.z
	wr = data.pose.pose.orientation.w
	
	quatNew = tf.transformations.quaternion_inverse((xr, yr, zr, wr))
	
	t.header.stamp = rospy.Time.now() + futureTimeIncrement
	t.header.frame_id = "base_link"
	t.child_frame_id = "gnss1_antenna_wgs84"
	
	t.transform.translation.x = -x
	t.transform.translation.y = -y
	t.transform.translation.z = -z
	
	t.transform.rotation.x = quat[0]
	t.transform.rotation.y = quat[1]
	t.transform.rotation.z = quat[2]
	t.transform.rotation.w = quat[3]
	
	br.sendTransform(t)
	print(t)
    
if __name__ == '__main__':
	rospy.init_node('world_pose_to_odom', anonymous=True)
	odom_msg = rospy.Subscriber("/gnss1/odom", Odometry, world_pose_talker)

	rospy.spin()

