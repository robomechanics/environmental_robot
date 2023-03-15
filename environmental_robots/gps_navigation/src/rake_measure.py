#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolRequest
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import numpy as np


def rake_measure():
    pass
    
def odom_callback(msg):
    pass
    
def drive():
    pass

def rake():
    rospy.wait_for_service('deploy_tool')
    try:
        deploy_tool = rospy.ServiceProxy('deploy_tool', SetBool)
        res = deploy_tool(True)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def measure():
    rospy.wait_for_service('deploy_sensor')
    try:
        deploy_sensor = rospy.ServiceProxy('deploy_sensor', SetBool)
        res = deploy_sensor(True)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    rospy.init_node('rake_measure_node')
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry , odom_callback)
    rospy.Service('my_service', SetBool, rake_measure)
    rake_measure()
    rospy.spin()