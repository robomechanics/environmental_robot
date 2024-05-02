#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("utm_to_odom_node")

    tf_utm_odom_frame = rospy.get_param("tf_utm_odom_frame")
    tf_base_link_frame = rospy.get_param("tf_base_link_frame")
    gps_odom_topic = rospy.get_param("gps_odom_topic")
    gq7_ekf_llh_topic = rospy.get_param("gq7_ekf_llh_topic")

    utm_odom_pub = rospy.Publisher(gps_odom_topic, Odometry, queue_size=10)
    ekf_ll_pub = rospy.Publisher(gq7_ekf_llh_topic, NavSatFix, queue_size=10)
    
    
    # Odom Msg
    odom_msg = Odometry()
    odom_msg.header.frame_id = tf_utm_odom_frame
    odom_msg.child_frame_id = tf_base_link_frame
    odom_msg.pose.pose.orientation.w = 1
    
    # Nav Msg    
    nav_msg = NavSatFix()
    
    while not rospy.is_shutdown():
        utm_odom_pub.publish(odom_msg)
        ekf_ll_pub.publish(nav_msg)
        
        rospy.sleep(1)

    rospy.spin()
    rospy.loginfo("Shutting down the ROS node.")
