#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from env_utils.ros_utils import get_ros_pkg_path
import os
import yaml

def gps_publisher():
    # Initialize the ROS node
    rospy.init_node('fake_static_gps_publisher', anonymous=True)
    
    config_dir_path = os.path.join(get_ros_pkg_path('gps_gui'), 'config')
    config_file_path = os.path.join(config_dir_path, 'gui_config.yaml')

    gui_config = None
    with open(config_file_path, 'r') as file:
        gui_config = yaml.safe_load(file) 
        
    gq7_ekf_llh_topic = rospy.get_param("gq7_ekf_llh_topic")
    gps_moving_avg_topic = rospy.get_param("gps_moving_avg_topic")
    
    # Create a publisher for the /gq7/ekf/llh_position topic
    gps_pub = rospy.Publisher(gq7_ekf_llh_topic, NavSatFix, queue_size=10)
    avg_gps_pub = rospy.Publisher(gps_moving_avg_topic, NavSatFix, queue_size=10)

    # Set the rate of publishing (in Hz)
    rate = rospy.Rate(1)  # 1 Hz

    # Create a NavSatFix message
    gps_msg = NavSatFix()
    gps_msg.header.stamp = rospy.Time.now()
    gps_msg.header.frame_id = "gps"
    # Set static GPS values (latitude, longitude, altitude)
    gps_msg.latitude = gui_config['robot_start_location'][0]
    gps_msg.longitude = gui_config['robot_start_location'][1]
    gps_msg.altitude =gui_config['robot_start_altitude']

    # Optional: Set covariance values (identity matrix)
    gps_msg.position_covariance = [0.0] * 9  # 3x3 covariance matrix
    gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

    # rospy.loginfo("Publishing static GPS data: %s", gps_msg)

    # Publish the message
    while not rospy.is_shutdown():
        gps_msg.header.stamp = rospy.Time.now()  # Update timestamp
        gps_pub.publish(gps_msg)
        avg_gps_pub.publish(gps_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass
