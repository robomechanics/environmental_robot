#!/usr/bin/env python3
import rospy
import numpy as np

from pyproj import CRS, Transformer

import tf
from tf.transformations import euler_from_quaternion
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

class UTMToOdom():
    def __init__(self, parent_frame_id="base_link", child_frame_id="utm_odom2"):
        self.first_time = True
        
        self.crs_GPS = 'EPSG:4326'
        self.crs_UTM = 'placeholder_EPSG'
        self.transformer = Transformer.from_crs(self.crs_GPS, self.crs_UTM)

        self.pose_odom_initial = geometry_msgs.msg.Pose()
        self.pose_odom_covariance_initial = geometry_msgs.msg.PoseWithCovariance()
        self.pose_odom = geometry_msgs.msg.Pose()
        self.pose_odom_covariance = geometry_msgs.msg.PoseWithCovariance()

        self.quaternion = tf.transformations.random_quaternion()

        self.odom_initial = Odometry()
        self.odom = Odometry()

        # Creating a transform and its broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        self.t = geometry_msgs.msg.TransformStamped()
        self.t.header.frame_id = parent_frame_id
        self.t.child_frame_id = child_frame_id

		# Publishers and Subscribers
        self.utm_odom_pub = rospy.Publisher("utm_odom2", Odometry, queue_size=1)
        self.utm_odom_initial_pub = rospy.Publisher("utm_odom2_initial", Odometry, queue_size=1)
        self.gps_sub = rospy.Subscriber("/nav/odom", Odometry, self.utm_broadcaster)

    def utm_broadcaster(self, data):
        self.odom = Odometry()
        self.firstTimeCode(data)
        self.utm_odom_initial_pub.publish(self.odom_initial)
        self.handleGPS_MSG(data)
        self.create_UTM_Odom(data)
        self.findInverses(data)
        self.sendBackTF(data)

    def firstTimeCode(self,data):
        if self.first_time == True:
            # Set the averages of lat and lon
            self.lat_GPS_start = data.pose.pose.position.x
            self.lon_GPS_start = data.pose.pose.position.y

            # Convert from lat-lon to UTM using py proj
            XY_UTM_start = self.transformer.transform(self.lat_GPS_start, self.lon_GPS_start)
            self.x_UTM_start = XY_UTM_start[0]
            self.y_UTM_start = XY_UTM_start[1]

            self.first_time = False

            # Creating a pose message for location in Odom
            self.pose_odom_initial.position.x = self.x_UTM_start
            self.pose_odom_initial.position.y = self.y_UTM_start
            self.pose_odom_initial.position.z = 0.0
            self.pose_odom_covariance_initial.pose = self.pose_odom_covariance_initial

            # Formatting the Odom message
            self.odom_initial.header.stamp = data.header.stamp
            self.odom_initial.header.frame_id = "utm_odom2"
            self.odom_initial.child_frame_id = "base_link"
            self.odom_initial.pose = self.pose_odom_covariance_initial

    def handleGPS_MSG(self,data):

        # Grab lat and lon values
        self.lat_GPS = data.pose.pose.position.x
        self.lon_GPS = data.pose.pose.position.y

        # Convert to UTM/XY values
        self.XY_UTM = self.transformer.transform(self.lat_GPS, self.lon_GPS)
        self.x_UTM = self.XY_UTM[0]
        self.y_UTM = self.XY_UTM[1]

        # Grabbing the quaternion orientation
        self.quaternion[0] = data.pose.pose.orientation.x
        self.quaternion[1] = data.pose.pose.orientation.y
        self.quaternion[2] = data.pose.pose.orientation.z
        self.quaternion[3] = data.pose.pose.orientation.w

        # Grabbing a heading angle
        eulerVals = euler_from_quaternion(self.quaternion, 'sxyz')
        self.theta_heading = eulerVals[2]

    def create_UTM_Odom(self,data):
        # Creating a pose message for location in Odom
        self.pose_odom.position.x = self.x_UTM - self.x_UTM_start
        self.pose_odom.position.y = self.y_UTM - self.y_UTM_start
        self.pose_odom.position.z = 0.0
        self.pose_odom.orientation.x = self.quaternion[0]
        self.pose_odom.orientation.y = self.quaternion[1]
        self.pose_odom.orientation.z = self.quaternion[2]
        self.pose_odom.orientation.w = self.quaternion[3]
        self.pose_odom_covariance.pose = self.pose_odom

        # Formatting the Odom message
        self.odom.header.stamp = data.header.stamp
        self.odom.header.frame_id = "utm_odom2"
        self.odom.child_frame_id = "base_link"
        self.odom.pose = self.pose_odom_covariance

    def findInverses(self,data):
        # Publishing the Odom message
        self.utm_odom_pub.publish(self.odom)

        # Inverse transformation matrix
        self.g = np.array([[np.cos(self.theta_heading), -np.sin(self.theta_heading), 0, (self.x_UTM - self.x_UTM_start)], 
                      [np.sin(self.theta_heading),  np.cos(self.theta_heading), 0, (self.y_UTM - self.y_UTM_start)],
                      [0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0]])
        self.gInv = tf.transformations.inverse_matrix(g)

        self.xBack = self.gInv[0][3]
        self.yBack = self.gInv[1][3]

    def sendBackTF(self,data):
        # Defining the backwards transformation
        self.t.header.stamp = data.header.stamp

        self.t.transform.translation.x = self.xBack
        self.t.transform.translation.y = self.yBack
        self.t.transform.translation.z = 0

        quatInv = tf.transformations.quaternion_inverse(self.quaternion)

        self.t.transform.rotation.x = quatInv[0]
        self.t.transform.rotation.y = quatInv[1]
        self.t.transform.rotation.z = quatInv[2]
        self.t.transform.rotation.w = quatInv[3]

        self.br.sendTransform(self.t)

if __name__ == '__main__':
    rospy.init_node('utm_to_odom_node', anonymous=True)

    utm_to_odom = UTMToOdom()

    rospy.spin()