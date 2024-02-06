#!/usr/bin/env python3

import rospy
import hebi
import math
import time
import numpy as np
from sensor_msgs.msg import JointState

def printData(msg):
    print(msg.name)
    print(msg.position)
    print(msg.velocity)
    print(msg.effort)

if __name__ == '__main__':
    rospy.init_node("ForceFeedback", anonymous=True)
    rospy.Subscriber('hebi_joint_states', JointState, printData, queue_size=1)
    rospy.spin()

