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
from autonomy_manager.srv import NavigateGPS

class GPSNavigationInterface:
    def __init__(self):
        self.load_ros_params()
        
        self.first_LatLon = True
        self.is_full_nav_achieved = False
        self._gps_avg_history_sum = NavSatFix()
        self._gps_avg = NavSatFix()
        self.reset_gps_moving_avg()
        # self.first_time_Orientation = True

        self.transformer = Transformer.from_crs(self._crs_GPS, self._crs_UTM)

        self.quaternion = tf.transformations.random_quaternion()
        self.odom = Odometry()
        self.odom.header.frame_id = self._tf_utm_odom_frame
        self.odom.child_frame_id = self._tf_base_link_frame

        # Creating a transform and its broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        self.t = geometry_msgs.msg.TransformStamped()
        self.t.header.frame_id = self._tf_base_link_frame
        self.t.child_frame_id = self._tf_utm_odom_frame

        # Publishers and Subscribers
        self.utm_odom_pub = rospy.Publisher(
            self._gps_odom_topic, Odometry, queue_size=1
        )
        
        self.gps_avg_pub = rospy.Publisher(
            self._gps_moving_avg_topic, NavSatFix, queue_size=1
        )
        
        self.gps_status_sub = rospy.Subscriber(
            self._gq7_ekf_status_topic, HumanReadableStatus, self.gps_status_callback
        )

        self.ekf_odom_map_sub = message_filters.Subscriber(
            self._gq7_ekf_odom_map_topic, Odometry
        )
        self.ekf_llh_sub = message_filters.Subscriber(
            self._gq7_ekf_llh_topic, NavSatFix
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.ekf_odom_map_sub, self.ekf_llh_sub], 1, True, True
        )
        self.ts.registerCallback(self.time_sync_callback)
        
        # self.atenna1_sub = rospy.Subscriber("/gq7/gnss_1/llh_position", NavSatFix, self.antenna1_update)
        # self.atenna2_sub = rospy.Subscriber("/gq7/gnss_2/llh_position", NavSatFix, self.antenna2_update)

    def load_ros_params(self):
        # Load topic names into params
        self._tf_base_link_frame = rospy.get_param("tf_base_link_frame")
        self._tf_utm_odom_frame = rospy.get_param("tf_utm_odom_frame")
        self._gps_odom_topic = rospy.get_param("gps_odom_topic")
        self._gq7_ekf_odom_map_topic = rospy.get_param("gq7_ekf_odom_map_topic")
        self._gps_moving_avg_topic = rospy.get_param("gps_moving_avg_topic")
        self._gq7_ekf_status_topic = rospy.get_param("gq7_ekf_status_topic")
        self._gq7_ekf_llh_topic = rospy.get_param("gq7_ekf_llh_topic")
        self._crs_GPS = rospy.get_param("crs_GPS")
        self._crs_UTM = rospy.get_param("crs_UTM")
        self._gps_avg_time = rospy.get_param("gps_moving_avg_time")
        
        self._move_base_action_server_name = rospy.get_param('move_base_action_server_name')
        
        self._start_utm_x_param = rospy.get_param("start_utm_x_param")
        self._start_utm_y_param = rospy.get_param("start_utm_y_param")
        

    def gps_status_callback(self, data):
        # todo: check data.status_flags.heading_warning?
        if (
            (self.is_full_nav_achieved == False)
            and (data.filter_state == '"Full Nav"')
            and ('"Stable"' in data.status_flags)
        ):
            self.is_full_nav_achieved = True
            rospy.loginfo("FULL NAV ACHIEVED")

    def time_sync_callback(self, odom_map_data, llh_data):
        # rospy.loginfo("Sync Callback")
        if self.is_full_nav_achieved:
            self.update_orientation(odom_map_data)
            self.utm_broadcaster(llh_data)
            self.publish_gps_moving_avg(llh_data)

    def update_orientation(self, data):
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

    def get_origin(self, data):
        # Convert from lat-lon to UTM using py proj
        self.x_UTM_start, self.y_UTM_start = self.transformer.transform(
             data.latitude, data.longitude
        )

        self.first_LatLon = False
        
        rospy.set_param(self._start_utm_x_param, self.x_UTM_start)
        rospy.set_param(self._start_utm_y_param, self.y_UTM_start)

        self.gps_status_sub.unregister()

    def handle_GPS_MSG(self, data):
        # Convert to UTM/XY values
        self.x_UTM, self.y_UTM  = self.transformer.transform(data.latitude, data.longitude)

        # Grabbing a heading angle
        eulerVals = euler_from_quaternion(self.quaternion, "sxyz")
        self.theta_heading = eulerVals[2]
        
    def reset_gps_moving_avg(self):
        self._gps_avg_history_sum.latitude = 0
        self._gps_avg_history_sum.longitude = 0
        self._gps_avg_history_sum.altitude = 0
        
        self._gps_avg_history_count = 0
        self.gps_avg_last_publish_time = rospy.Time.now()
        
    def publish_gps_moving_avg(self, data: NavSatFix):
        if ( ( (rospy.Time.now() - self.gps_avg_last_publish_time).secs > self._gps_avg_time) and (self._gps_avg_history_count > 0) ):
                self._gps_avg.latitude = self._gps_avg_history_sum.latitude / self._gps_avg_history_count
                self._gps_avg.longitude = self._gps_avg_history_sum.longitude / self._gps_avg_history_count
                
                self.reset_gps_moving_avg()
                
                self.gps_avg_pub.publish(self._gps_avg)
        else:
            self._gps_avg_history_count += 1
            self._gps_avg_history_sum.latitude += data.latitude
            self._gps_avg_history_sum.longitude += data.longitude
            
    def create_UTM_Odom(self, data):
        # Creating a pose message for location in Odom
        self.odom.pose.pose.position.x = self.x_UTM - self.x_UTM_start
        self.odom.pose.pose.position.y = self.y_UTM - self.y_UTM_start
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation.x = self.quaternion[0]
        self.odom.pose.pose.orientation.y = self.quaternion[1]
        self.odom.pose.pose.orientation.z = self.quaternion[2]
        self.odom.pose.pose.orientation.w = self.quaternion[3]

        # Publish Odom message
        self.odom.header.stamp = data.header.stamp
        self.utm_odom_pub.publish(self.odom)

    def find_inverses(self, data):
        # Inverse transformation matrix
        self.g = np.array(
            [
                [
                    np.cos(self.theta_heading),
                    -np.sin(self.theta_heading),
                    0,
                    (self.x_UTM - self.x_UTM_start),
                ],
                [
                    np.sin(self.theta_heading),
                    np.cos(self.theta_heading),
                    0,
                    (self.y_UTM - self.y_UTM_start),
                ],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.gInv = tf.transformations.inverse_matrix(self.g)

        self.xBack = self.gInv[0][3]
        self.yBack = self.gInv[1][3]

    def send_back_TF(self, data):
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

    def antenna1_update(self, data):
        self.lat1 = data.latitude
        self.lon1 = data.longitude

        self.xy1 = self.transformer.transform(self.lat1, self.lon1)
        self.x1 = self.xy1[0]
        self.y1 = self.xy1[1]

        self.v1 = np.zeros((4, 1))
        self.v1[0, 0] = self.x1
        self.v1[1, 0] = self.y1
        self.v1[3, 0] = 1.0

        self.b1 = self.gInv @ self.v1

    def antenna2_update(self, data):
        self.lat2 = data.latitude
        self.lon2 = data.longitude

        self.xy2 = self.transformer.transform(self.lat2, self.lon2)
        self.x2 = self.xy2[0]
        self.y2 = self.xy2[1]

        self.v2 = np.zeros((4, 1))
        self.v2[0, 0] = self.x2
        self.v2[1, 0] = self.y2
        self.v2[3, 0] = 1.0

        self.b2 = self.gInv @ self.v2

        self.delta = self.b2 - self.b1

        print(self.delta)


if __name__ == "__main__":
    rospy.init_node("utm_to_odom_node", anonymous=True)

    gps_nav_interface = GPSNavigationInterface()
    rospy.loginfo("Started GPS Navigation Interface Node...")
    rospy.spin()