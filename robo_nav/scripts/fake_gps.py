#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from microstrain_inertial_msgs.msg import HumanReadableStatus

def gps_publisher():
    # Initialize the ROS node
    rospy.init_node('fake_static_gps_publisher', anonymous=True)
    
    llh_pub = rospy.Publisher('/gq7/ekf/llh_position', NavSatFix, queue_size=10)
    odom_pub = rospy.Publisher('/gq7/ekf/odometry_map', Odometry, queue_size=10)
    status_pub = rospy.Publisher('/gq7/ekf/status', HumanReadableStatus, queue_size=10)
    gps_moving_avg_pub = rospy.Publisher('/gps_moving_avg', NavSatFix, queue_size=1)

    llh_msg = NavSatFix()
    llh_msg.header.frame_id = "gq7_link"
    llh_msg.status = NavSatStatus(status=0, service=0)
    llh_msg.latitude = 40.45898801882863
    llh_msg.longitude = -79.95823765892326
    llh_msg.altitude = 277.9383034546891
    llh_msg.position_covariance = [
        0.9874569211100415, 0.0, 0.0,
        0.0, 1.3351858854839662, 0.0,
        0.0, 0.0, 3.422781499451858
    ]
    llh_msg.position_covariance_type = 2

    odom_msg = Odometry()
    odom_msg.header.frame_id = "map"
    odom_msg.child_frame_id = "gq7_link"
    
    # Pose setup
    odom_msg.pose.pose.position = Point(
        x=5.755908464896493,
        y=-4.518386106472462,
        z=-3.2766532097011805
    )
    odom_msg.pose.pose.orientation = Quaternion(
        x=0.005964690479088392,
        y=-0.006743469320058195,
        z=-0.6586022539645009,
        w=0.7524373855424101
    )
    odom_msg.pose.covariance = [
        0.987456, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.335185, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 3.422781, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.2915e-07, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.2481e-07, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0004321
    ]
    
    # Twist setup
    odom_msg.twist.twist.linear = Vector3(
        x=0.13992607060061374,
        y=0.03447835453757019,
        z=0.03439104913436829
    )
    odom_msg.twist.twist.angular = Vector3(
        x=0.003496671561151743,
        y=-0.011589989997446537,
        z=-0.0028887467924505472
    )
    odom_msg.twist.covariance = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0001844, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0001631, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0004315
    ]

    status_msg = HumanReadableStatus()
    status_msg.header.frame_id = "gq7_link"
    status_msg.device_info.firmware_version = "1.1.02"
    status_msg.device_info.model_name = "3DM-GQ7"
    status_msg.device_info.model_number = "6284-4220"
    status_msg.device_info.serial_number = "6284.154203"
    status_msg.device_info.device_options = "8g,300dps"
    status_msg.gnss_state = "3D Fix"
    status_msg.dual_antenna_fix_type = "Dual Antenna Float"
    status_msg.filter_state = '"Full Nav"'
    status_msg.status_flags = ['"Stable"']
    status_msg.continuous_bit_flags = ["RTK Dongle Fault"]

    print("Publishing fake GPS data...")

    # Publish the message
    rate = rospy.Rate(30)
    counter = 0
    while not rospy.is_shutdown():
        llh_msg.header.stamp = rospy.Time.now()
        odom_msg.header.stamp = rospy.Time.now()
        status_msg.header.stamp = rospy.Time.now()
        llh_pub.publish(llh_msg)
        odom_pub.publish(odom_msg)
        status_pub.publish(status_msg)
        if counter % 60 == 0:
            gps_moving_avg_pub.publish(llh_msg)
        rate.sleep()
        counter += 1

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass
