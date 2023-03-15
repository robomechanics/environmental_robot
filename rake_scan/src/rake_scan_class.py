#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

class rake_scan(object):
	def __init__(self):
		rosp.init_node('rake_scan', anonymouse=True)
