#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolRequest
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib as plt 
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class obsAvoidance(object):
	def __init__(self):
		rospy.init_node('obstacleAvoidance', anonymous=True)
		rospy.Subscriber('/ouster/points', PointCloud2, self.obsMap)
		#define class variables
		self.heightThreshLow = -0.4
		self.heightThreshHigh = 0.1
		#rate = rospy.Rate(10)
		#rate.sleep()
		rospy.spin()		
	def obsMap(self, data):
		env = []
		for point in pc2.read_points(data, skip_nans=True):
			# if the obstacle height is lower, then skip
			if point[2] < self.heightThreshLow:
				continue
			elif point[2] > self.heightThreshHigh:
				continue
			elif point[0] == point[1] == point[2] == 0:
				continue
			print("x:{}, y:{}, z:{}".format(point[0], point[1], point[2]))
			env.append(env)
		#env = np.asarray(env)
		#print(env.shape)	
			
if __name__ == '__main__':
	obsAvoidance()










