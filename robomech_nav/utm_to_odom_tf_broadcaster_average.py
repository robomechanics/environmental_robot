#!/usr/bin/env python3
import rospy
import numpy as np
import pyproj
import statistics

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
firstTime = True
thetaHeading = 0
utmDefault = 'EPSG:32617'
nPoints = 0
lat_list = []
lon_list = []

#get the zone based on the location
def get_zone(posx, posy):
	utm_crs_list = query_utm_crs_info( 
	datum_name="WGS 84", 
	area_of_interest=AreaOfInterest( 
	    west_lon_degree=posy, 
	    south_lat_degree=posx, 
	    east_lon_degree=posy, 
	    north_lat_degree=posx, 
	    ), 
	)
	utm_crs = CRS.from_epsg(utm_crs_list[0].code)
	utmDefault = utm_crs
	return utm_crs

# get utm positions
def get_utm(posx, posy):
	wgs84 = pyproj.CRS('EPSG:4326') # WGS 84
	utm17n = pyproj.CRS(utmDefault) 
	transformer = pyproj.Transformer.from_crs(wgs84, utm17n)
	return transformer.transform(posx, posy)

def utm_broadcaster(data):
	global firstTime
	global x_UTM_start
	global y_UTM_start
	global zone
	global utmDefault	
	global nPoints
	global lat_list
	global lon_list
	
	#initialization
	Odom = Odometry()
	pose_Odom = geometry_msgs.msg.Pose()
	pose_Odom_covariance = geometry_msgs.msg.PoseWithCovariance()
	quatRaw = [0,0,0,0]	
	# Set the condition for the first time running
	#if firstTime == True:
	if nPoints < 100:
		# Set the averages of lat and lon
		lat_GPS_start = data.pose.pose.position.y #data.pose.pose.position.x
		lon_GPS_start = data.pose.pose.position.x #data.pose.pose.position.y
		nPoints += 1
		lat_list.append(lat_GPS_start)
		lon_list.append(lon_GPS_start)
			
		# Average 100 datapoints
		lat_GPS_start = statistics.mean(lat_list)
		lon_GPS_start = statistics.mean(lon_list)	
		#zone = get_zone(lat_GPS_start, lon_GPS_start)
		x_UTM_start, y_UTM_start = get_utm(lat_GPS_start, lon_GPS_start) 
		#firstTime = False
	#print(len(lat_list))
	lat_GPS = data.pose.pose.position.y #data.pose.pose.position.x
	lon_GPS = data.pose.pose.position.x #data.pose.pose.position.y
	
	x_UTM, y_UTM = get_utm(lat_GPS, lon_GPS)	
	
	quatRaw[0] = data.pose.pose.orientation.x
	quatRaw[1] = data.pose.pose.orientation.y
	quatRaw[2] = data.pose.pose.orientation.z
	quatRaw[3] = data.pose.pose.orientation.w
	eulers =  tf.transformations.euler_from_quaternion(quatRaw,'sxyz') #sxyz
	#print(eulers)
	#thetaHeading = eulers[2] +(-65.0)/180*np.pi
	thetaHeading = eulers[2]
	#thetaHeading = np.arctan2(np.sin(thetaHeading), np.cos(thetaHeading))
	thetaHeading = np.mod(np.arctan2(np.sin(thetaHeading),np.cos(thetaHeading))-(np.pi/3.0),2*np.pi)
	print(thetaHeading/np.pi*180)
	
	quat = tf.transformations.quaternion_from_euler(0,0,thetaHeading,'sxyz')
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
	#print(data.header.stamp)
	Odom.header.frame_id = "utm_odom2"
	Odom.child_frame_id = "base_link"
	Odom.pose = pose_Odom_covariance
	
	# Transformation matrix	
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
	
	quatInv = tf.transformations.quaternion_inverse(quat)
	
	t.transform.rotation.x = quatInv[0]
	t.transform.rotation.y = quatInv[1]
	t.transform.rotation.z = quatInv[2]
	t.transform.rotation.w = quatInv[3]
	
	# Publishing the Odom message and transformation
	utm_odom_pub.publish(Odom)
	br.sendTransform(t)

if __name__ == '__main__':
	# Initiate ros subscriber and publisher
	rospy.init_node('utm_node2', anonymous=True)
	utm_odom_pub = rospy.Publisher("utm_odom2", Odometry, queue_size=1)
	gps_listener = rospy.Subscriber("/nav/odom", Odometry, utm_broadcaster)
	rospy.spin()

