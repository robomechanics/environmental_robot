#!/usr/bin/env python3

import rospy


if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("dummy_node")

    rospy.spin()
    rospy.loginfo("Shutting down the ROS node.")
