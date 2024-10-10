#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

def gps_publisher():
    # Initialize the ROS node
    rospy.init_node('static_gps_publisher', anonymous=True)

    # Create a publisher for the /gq7/ekf/llh_position topic
    pub = rospy.Publisher('/gq7/ekf/llh_position', NavSatFix, queue_size=10)

    # Set the rate of publishing (in Hz)
    rate = rospy.Rate(1)  # 1 Hz

    # Create a NavSatFix message
    gps_msg = NavSatFix()
    gps_msg.header.stamp = rospy.Time.now()
    gps_msg.header.frame_id = "gps"

    # Set static GPS values (latitude, longitude, altitude)
    gps_msg.latitude = 40.47219969363796
    gps_msg.longitude = -79.96628216486742
    gps_msg.altitude = 10.0 

    # Optional: Set covariance values (identity matrix)
    gps_msg.position_covariance = [0.0] * 9  # 3x3 covariance matrix
    gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

    # rospy.loginfo("Publishing static GPS data: %s", gps_msg)

    # Publish the message
    while not rospy.is_shutdown():
        gps_msg.header.stamp = rospy.Time.now()  # Update timestamp
        pub.publish(gps_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass
