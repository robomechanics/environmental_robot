#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

if __name__ == '__main__':
    # start node
    rospy.init_node('rake_scan_teleop', anonymous=True)
    rospy.sleep(5)

    # setting rake torque
    dig_torque_publisher = rospy.Publisher('/dig_torque',Float64, queue_size=10)
    msg = Float64()
    msg.data = -30
    while True:
        dig_torque_publisher.publish(msg)
        rospy.sleep(1)
