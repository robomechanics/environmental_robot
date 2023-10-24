#!/usr/bin/env python3
import rospy
import numpy as np

from pyproj import CRS, Transformer

import tf
from tf.transformations import euler_from_quaternion
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

# x = 0
# y = 0
# z = 0

# xr = 0
# yr = 0
# zr = 0
# wr = 0

class UTMToOdom():
    def __init__(self, parent_frame_id="base_link", child_frame_id="utm_odom2"):
        self.first_time = True

        self.lat_GPS = 0
        self.long_GPS = 0

        self.XY_UTM = 0
        self.x_UTM = 0
        self.y_UTM = 0

        self.theta_declination = -9.0

        self.theta_heading = 0
        self.thetaStart = 0

        self.currentSample = 0
        self.numSamples = 10

        self.lat_array = np.zeros(self.numSamples)
        self.lon_array = np.zeros(self.numSamples)
        self.head_array = np.zeros(self.numSamples)

        self.pose_UTM = geometry_msgs.msg.Pose()
        self.pose_UTM_covariance = geometry_msgs.msg.PoseWithCovariance()
        self.pose_odom = geometry_msgs.msg.Pose()
        self.pose_odom_covariance = geometry_msgs.msg.PoseWithCovariance()

        self.quat_raw = tf.transformations.random_quaternion()
        self.quat = tf.transformations.random_quaternion()

        self.odom = Odometry()

        # Creating a transform and its broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        self.t = geometry_msgs.msg.TransformStamped()
        self.t.header.frame_id = parent_frame_id
        self.t.child_frame_id = child_frame_id

		# Publishers and Subscribers
        self.utm_odom_pub = rospy.Publisher("utm_odom2", Odometry, queue_size=1)
        self.gps_sub = rospy.Subscriber("/nav/odom",
                                        Odometry,
                                        self.utm_broadcaster)

    def utm_broadcaster(self, data):
        self.odom = Odometry()

        # Set the condition for the first time running
        if first_time == True:
            # Set the averages of lat and lon
            lat_GPS_start = data.pose.pose.position.x
            lon_GPS_start = data.pose.pose.position.y

            # Convert from lat-lon to UTM using py proj
            crs_GPS = CRS.from_epsg(4326)
            crs_UTM = CRS.from_epsg(3651)

            transformer = Transformer.from_crs(crs_GPS, crs_UTM)
            XY_UTM_start = transformer.transform(lat_GPS_start, lon_GPS_start)
            x_UTM_start = XY_UTM_start[0]
            y_UTM_start = XY_UTM_start[1]
            self.quat_raw[0] = data.pose.pose.orientation.x
            self.quat_raw[1] = data.pose.pose.orientation.y
            self.quat_raw[2] = data.pose.pose.orientation.z
            self.quat_raw[3] = data.pose.pose.orientation.w
            eulers = euler_from_quaternion(self.quat_raw, 'sxyz')  # sxyz
            theta_heading = eulers[2] + (0.0)/180*np.pi
            theta_heading = np.arctan2(
                np.sin(theta_heading), np.cos(theta_heading))

            first_time = False

        crs_GPS = CRS.from_epsg(4326)
        crs_UTM = CRS.from_epsg(3651)  # was 3857

        transformer = Transformer.from_crs(crs_GPS, crs_UTM)

        lat_GPS = data.pose.pose.position.x
        lon_GPS = data.pose.pose.position.y

        XY_UTM = transformer.transform(lat_GPS, lon_GPS)
        x_UTM = XY_UTM[0]
        y_UTM = XY_UTM[1]

        self.quat_raw[0] = data.pose.pose.orientation.x
        self.quat_raw[1] = data.pose.pose.orientation.y
        self.quat_raw[2] = data.pose.pose.orientation.z
        self.quat_raw[3] = data.pose.pose.orientation.w
        eulers = tf.transformations.euler_from_quaternion(
            self.quat_raw, 'sxyz')  # sxyz
        # print(eulers)
        # print(-eulers[2]/3.1415*180)
        theta_heading = eulers[2] + (0.0)/180*np.pi
        theta_heading = np.arctan2(
            np.sin(theta_heading), np.cos(theta_heading))
        print(eulers[2]*180/np.pi)

        quat = tf.transformations.quaternion_from_euler(
            0, 0, theta_heading, 'sxyz')

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
        self.pose_odom.position.x = x_UTM - x_UTM_start
        self.pose_odom.position.y = y_UTM - y_UTM_start
        self.pose_odom.position.z = 0.0
        self.pose_odom.orientation.x = quat[0]
        self.pose_odom.orientation.y = quat[1]
        self.pose_odom.orientation.z = quat[2]
        self.pose_odom.orientation.w = quat[3]
        self.pose_odom_covariance.pose = self.pose_odom

        # Formatting the Odom message
        self.odom.header.stamp = data.header.stamp
        self.odom.header.frame_id = "utm_odom2"
        self.odom.child_frame_id = "base_link"
        self.odom.pose = self.pose_odom_covariance

        # Publishing the Odom message
        self.utm_odom_pub.publish(self.odom)

        #x = Odom.pose.pose.position.x
        #y = Odom.pose.pose.position.y
        #z = Odom.pose.pose.position.z

        #xr = Odom.pose.pose.orientation.x
        #yr = Odom.pose.pose.orientation.y
        #zr = Odom.pose.pose.orientation.z
        #wr = Odom.pose.pose.orientation.w

        # Finding the reverse translation
        #rad = np.arctan2((y_UTM - y_UTM_start),(x_UTM - x_UTM_start))

        g = np.array([[np.cos(theta_heading), -np.sin(theta_heading), 0, (x_UTM - x_UTM_start)], [np.sin(theta_heading),
                     np.cos(theta_heading), 0, (y_UTM - y_UTM_start)], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        gInv = tf.transformations.inverse_matrix(g)

        xBack = gInv[0][3]
        yBack = gInv[1][3]

        self.t.header.stamp = data.header.stamp

        self.t.transform.translation.x = xBack
        self.t.transform.translation.y = yBack
        self.t.transform.translation.z = 0

        #quatT = tf.transformations.quaternion_from_euler(0,0,theta_heading,'rxyz')
        quatInv = tf.transformations.quaternion_inverse(quat)

        self.t.transform.rotation.x = quatInv[0]
        self.t.transform.rotation.y = quatInv[1]
        self.t.transform.rotation.z = quatInv[2]
        self.t.transform.rotation.w = quatInv[3]

        self.br.sendTransform(self.t)

if __name__ == '__main__':
    rospy.init_node('utm_to_odom_node', anonymous=True)

    utm_to_odom = UTMToOdom()

    rospy.spin()