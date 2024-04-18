#!/usr/bin/env python3
import rospy
import numpy as np
import tf
from tf.transformations import euler_from_quaternion
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from microstrain_inertial_msgs.msg import HumanReadableStatus
import message_filters
from pyproj import Transformer

class UTMToOdom():
    def __init__(self, parent_frame_id="base_link", child_frame_id="utm_odom"):
        self.first_LatLon = True
        self.is_full_nav_achieved = False
        # self.first_time_Orientation = True
        
        self.load_ros_params()
        
        self.crs_GPS = 'EPSG:4326'
        self.crs_UTM = 'EPSG:3364'
        self.transformer = Transformer.from_crs(self.crs_GPS, self.crs_UTM)

        self.pose_odom_covariance_initial = geometry_msgs.msg.PoseWithCovariance()
        self.pose_odom = geometry_msgs.msg.Pose()
        self.pose_odom_covariance = geometry_msgs.msg.PoseWithCovariance()

        self.quaternion = tf.transformations.random_quaternion()

        self.odom = Odometry()

        # Creating a transform and its broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        self.t = geometry_msgs.msg.TransformStamped()
        self.t.header.frame_id = parent_frame_id
        self.t.child_frame_id = child_frame_id

		# Publishers and Subscribers
        self.utm_odom_pub = rospy.Publisher(self._tf_utm_odom_topic, Odometry, queue_size=1)

        self.status_sub = rospy.Subscriber(self._gq7_ekf_status_topic, HumanReadableStatus, self.status_callback)
        
        self.ekf_odom_map_sub = message_filters.Subscriber(self._gq7_ekf_odom_map_topic, Odometry)
        self.ekf_llh_sub = message_filters.Subscriber(self._gq7_ekf_llh_topic, NavSatFix)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.ekf_odom_map_sub , self.ekf_llh_sub], 1, True, True)
        self.ts.registerCallback(self.time_sync_callback)
        
        # self.atenna1_sub = rospy.Subscriber("/gq7/gnss_1/llh_position", NavSatFix, self.antenna1_update)
        # self.atenna2_sub = rospy.Subscriber("/gq7/gnss_2/llh_position", NavSatFix, self.antenna2_update)
    
    def load_ros_params(self):
        # Load topic names into params
        self._tf_base_link_frame = rospy.get_param('tf_base_link_frame')
        self._tf_utm_odom_frame = rospy.get_param('tf_utm_odom_frame')
        self._tf_utm_odom_topic = rospy.get_param('tf_utm_odom_topic')
        self._gq7_ekf_odom_map_topic = rospy.get_param('gq7_ekf_odom_map_topic')
        self._gq7_ekf_llh_topic = rospy.get_param('gq7_ekf_llh_topic')
        self._gq7_ekf_status_topic = rospy.get_param('gq7_ekf_status_topic')
        
    def status_callback(self, data):
        # todo: check data.status_flags.heading_warning?
        if ((self.is_full_nav_achieved == False) and
            (data.filter_state == '"Full Nav"') and
            ('"Stable"' in data.status_flags) ):
            self.is_full_nav_achieved = True
            rospy.loginfo ("FULL NAV ACHIEVED")
            
    def time_sync_callback(self, odom_map_data, llh_data):
        if self.is_full_nav_achieved:
            self.update_orientation(odom_map_data)
            self.utm_broadcaster(llh_data)
                
    def update_orientation(self,data):
        # Grabbing the quaternion orientation
        self.quaternion[0] = data.pose.pose.orientation.x
        self.quaternion[1] = data.pose.pose.orientation.y
        self.quaternion[2] = data.pose.pose.orientation.z
        self.quaternion[3] = data.pose.pose.orientation.w

    def utm_broadcaster(self, data):
        self.odom = Odometry()
        if self.first_LatLon is True:
            self.get_origin(data)
        self.handle_GPS_MSG(data)
        self.create_UTM_Odom(data)
        self.find_inverses(data)
        self.send_back_TF(data)
        self.back_calc_antennas(data)

    def get_origin(self,data):
        # Grab initial latlon values
        self.lat_GPS_start = data.latitude
        self.lon_GPS_start = data.longitude

        # Convert from lat-lon to UTM using py proj
        self.XY_UTM_start = self.transformer.transform(self.lat_GPS_start, self.lon_GPS_start)
        self.x_UTM_start = self.XY_UTM_start[0]
        self.y_UTM_start = self.XY_UTM_start[1]

        self.first_LatLon = False
        
        self.status_sub.unregister()
        

    def handle_GPS_MSG(self,data):
        # Grab latlon values
        self.lat_GPS = data.latitude
        self.lon_GPS = data.longitude

        # Convert to UTM/XY values
        self.XY_UTM = self.transformer.transform(self.lat_GPS, self.lon_GPS)
        self.x_UTM = self.XY_UTM[0]
        self.y_UTM = self.XY_UTM[1]

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
        self.odom.header.frame_id = "utm_odom"
        self.odom.child_frame_id = "base_link"
        self.odom.pose = self.pose_odom_covariance

    def find_inverses(self,data):
        # Publishing the Odom message
        self.utm_odom_pub.publish(self.odom)

        # Inverse transformation matrix
        self.g = np.array([[np.cos(self.theta_heading), -np.sin(self.theta_heading), 0, (self.x_UTM - self.x_UTM_start)], 
                      [np.sin(self.theta_heading),  np.cos(self.theta_heading), 0, (self.y_UTM - self.y_UTM_start)],
                      [0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0]])
        self.gInv = tf.transformations.inverse_matrix(self.g)

        self.xBack = self.gInv[0][3]
        self.yBack = self.gInv[1][3]

    def send_back_TF(self,data):
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

    def back_calc_antennas(self,data):
        a = 0

    def antenna1_update(self,data):
        self.lat1 = data.latitude
        self.lon1 = data.longitude

        self.xy1 = self.transformer.transform(self.lat1, self.lon1)
        self.x1 = self.xy1[0]
        self.y1 = self.xy1[1]

        self.v1 = np.zeros((4,1))
        self.v1[0,0] = self.x1
        self.v1[1,0] = self.y1
        self.v1[3,0] = 1.0

        self.b1 = self.gInv @ self.v1

    def antenna2_update(self,data):
        self.lat2 = data.latitude
        self.lon2 = data.longitude

        self.xy2 = self.transformer.transform(self.lat2, self.lon2)
        self.x2 = self.xy2[0]
        self.y2 = self.xy2[1]

        self.v2 = np.zeros((4,1))
        self.v2[0,0] = self.x2
        self.v2[1,0] = self.y2
        self.v2[3,0] = 1.0

        self.b2 = self.gInv @ self.v2

        self.delta = self.b2 - self.b1

        print(self.delta)

if __name__ == '__main__':
    rospy.init_node('utm_to_odom_node', anonymous=True)

    utm_to_odom = UTMToOdom()

    rospy.spin()