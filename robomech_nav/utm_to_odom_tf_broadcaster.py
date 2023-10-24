#!/usr/bin/env python3
import rospy
import numpy as np

from pyproj import CRS
from pyproj import Transformer

import tf
import tf2_ros, tf2_geometry_msgs
import geometry_msgs.msg
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

x = 0
y = 0
z = 0

xr = 0
yr = 0
zr = 0
wr = 0

futureTimeIncrement = rospy.Duration.from_sec(0.1)

quatInv = tf.transformations.random_quaternion()

t = geometry_msgs.msg.TransformStamped()

# ------------------------------------

firstTime = True
	
lat_GPS = 0
long_GPS = 0

XY_UTM = 0
x_UTM = 0
y_UTM = 0

theta_declination = -9.0

thetaHeading = 0
thetaStart = 0

currentSample = 0
numSamples = 10

lat_array = np.zeros(numSamples)
lon_array = np.zeros(numSamples)
head_array = np.zeros(numSamples)

pose_UTM = geometry_msgs.msg.Pose()
pose_UTM_covariance = geometry_msgs.msg.PoseWithCovariance()
pose_Odom = geometry_msgs.msg.Pose()
pose_Odom_covariance = geometry_msgs.msg.PoseWithCovariance()

quatRaw = tf.transformations.random_quaternion()
quat = tf.transformations.random_quaternion()

test = 1234

Odom = Odometry()

def utm_broadcaster(data):
	
	global firstTime
	
	global lat_GPS
	global long_GPS
	
	global lat_GPS_start
	global long_GPS_start

	global XY_UTM
	global x_UTM
	global y_UTM

	global XY_UTM_start
	global x_UTM_start
	global y_UTM_start

	global theta_declination
	global numSamples

	global lat_array
	global lon_array

	global thetaHeading
	global thetaStart
	
	global quatRaw
	global quat
	
	global eulers

	global pose_UTM
	global pose_UTM_covariance
	global pose_Odom
	global pose_Odom_covariance
	
	global transformer
	
	global test
	
	global Odom
	
	global x
	global y
	global z
	
	global xr
	global yr
	global zr
	global wr
	
	global t
	global quatInv
	
	Odom = Odometry()
	
	# Set the condition for the first time running
	if firstTime == True:
		# Set the averages of lat and lon
		lat_GPS_start = data.pose.pose.position.x
		lon_GPS_start = data.pose.pose.position.y
		
		# Convert from lat-lon to UTM using py proj
		crs_GPS = CRS.from_epsg(4326)
		crs_UTM = CRS.from_epsg(3651)
		
		transformer = Transformer.from_crs(crs_GPS,crs_UTM)
		XY_UTM_start = transformer.transform(lat_GPS_start,lon_GPS_start)
		x_UTM_start = XY_UTM_start[0]
		y_UTM_start = XY_UTM_start[1]
		quatRaw[0] = data.pose.pose.orientation.x
		quatRaw[1] = data.pose.pose.orientation.y
		quatRaw[2] = data.pose.pose.orientation.z
		quatRaw[3] = data.pose.pose.orientation.w
		eulers =  tf.transformations.euler_from_quaternion(quatRaw,'sxyz')#sxyz
		thetaHeading = eulers[2] + (0.0)/180*np.pi
		thetaHeading = np.arctan2(np.sin(thetaHeading), np.cos(thetaHeading))
		
		firstTime = False
	
	crs_GPS = CRS.from_epsg(4326)
	crs_UTM = CRS.from_epsg(3651) # was 3857
		
	transformer = Transformer.from_crs(crs_GPS,crs_UTM)
	
	lat_GPS = data.pose.pose.position.x
	lon_GPS = data.pose.pose.position.y
	
	XY_UTM = transformer.transform(lat_GPS,lon_GPS)
	x_UTM = XY_UTM[0]
	y_UTM = XY_UTM[1]
	
	quatRaw[0] = data.pose.pose.orientation.x
	quatRaw[1] = data.pose.pose.orientation.y
	quatRaw[2] = data.pose.pose.orientation.z
	quatRaw[3] = data.pose.pose.orientation.w
	eulers =  tf.transformations.euler_from_quaternion(quatRaw,'sxyz') #sxyz
	#print(eulers)
	#print(-eulers[2]/3.1415*180)
	thetaHeading = eulers[2] + (0.0)/180*np.pi
	thetaHeading = np.arctan2(np.sin(thetaHeading), np.cos(thetaHeading))
	print(eulers[2]*180/np.pi)

	
	quat = tf.transformations.quaternion_from_euler(0,0,thetaHeading,'sxyz')
	
	# Creating a pose message for UTM
	#pose_UTM.position.x = x_UTM
	#pose_UTM.position.y = y_UTM
	#pose_UTM.position.z = 0.0
	#pose_UTM.orientation.x = quat[0]
	#pose_UTM.orientation.y = quat[1]
	#pose_UTM.orientation.z = quat[2]
	#pose_UTM.orientation.w = quat[3]
	#pose_UTM_covariance.pose = pose_UTM
	
	# Creating a pose message for location in Odom
	pose_Odom.position.x = x_UTM - x_UTM_start
	pose_Odom.position.y = y_UTM - y_UTM_start
	pose_Odom.position.z = 0.0
	pose_Odom.orientation.x = quat[0]
	pose_Odom.orientation.y = quat[1]
	pose_Odom.orientation.z = quat[2]
	pose_Odom.orientation.w = quat[3]
	pose_Odom_covariance.pose = pose_Odom
	
	#Formatting the Odom message
	Odom.header.stamp = data.header.stamp
	Odom.header.frame_id = "utm_odom2"
	Odom.child_frame_id = "base_link"
	Odom.pose = pose_Odom_covariance
	
	# Publishing the Odom message
	utm_odom_pub.publish(Odom)
	
	#x = Odom.pose.pose.position.x
	#y = Odom.pose.pose.position.y
	#z = Odom.pose.pose.position.z
	
	#xr = Odom.pose.pose.orientation.x
	#yr = Odom.pose.pose.orientation.y
	#zr = Odom.pose.pose.orientation.z
	#wr = Odom.pose.pose.orientation.w
	
	# Finding the reverse translation
	#rad = np.arctan2((y_UTM - y_UTM_start),(x_UTM - x_UTM_start))
	
	g = np.array([[np.cos(thetaHeading), -np.sin(thetaHeading), 0, (x_UTM - x_UTM_start)], [np.sin(thetaHeading), np.cos(thetaHeading), 0, (y_UTM - y_UTM_start)], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
	gInv = tf.transformations.inverse_matrix(g)
	
	xBack = gInv[0][3]
	yBack = gInv[1][3]
	
	# Creating a transform and its broadcaster
	br = tf2_ros.TransformBroadcaster()
	t = geometry_msgs.msg.TransformStamped()
	
	t.header.stamp = data.header.stamp
	t.header.frame_id = "base_link"
	t.child_frame_id = "utm_odom2"
	
	t.transform.translation.x = xBack
	t.transform.translation.y = yBack
	t.transform.translation.z = 0
	
	#quatT = tf.transformations.quaternion_from_euler(0,0,thetaHeading,'rxyz')
	quatInv = tf.transformations.quaternion_inverse(quat)
	
	t.transform.rotation.x = quatInv[0]
	t.transform.rotation.y = quatInv[1]
	t.transform.rotation.z = quatInv[2]
	t.transform.rotation.w = quatInv[3]
	
	br.sendTransform(t)

if __name__ == '__main__':
	# Initiate ros subscriber and publisher
	rospy.init_node('utm_node2', anonymous=True)
	utm_odom_pub = rospy.Publisher("utm_odom2", Odometry, queue_size=1)
	gps_listener = rospy.Subscriber("/nav/odom", Odometry, utm_broadcaster)
	
	#utm_odom_pub.publish(Odom)
	rospy.spin()

