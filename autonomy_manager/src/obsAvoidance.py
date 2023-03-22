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
		
		self.frontBack = 0.5
		self.side = 0.3
		
		self.x = self.side * 2 + 0.6
		self.y = self.frontBack * 2 + 0.8
		
		#discretization
		self.cell = 0.05

		# map size
		self.mapx = self.x / self.cell
		self.mapy = self.y / self.cell

		#rate = rospy.Rate(10)
		#rate.sleep()
		rospy.spin()		
	def obsMap(self, data):
		env = []
		localMap = np.full((self.mapx, self.mapy), 0)
		for point in pc2.read_points(data, skip_nans=True):
			# if the obstacle height is lower, then skip
			if point[2] < self.heightThreshLow or point[2] > self.heightThreshHigh:
				continue
			# if the point is invalid
			elif point[0] == point[1] == point[2] == 0:
				continue
			# if the point is 	
			elif abs(point[0]) > (self.side + 0.42) or abs(point[1]) > (self.frontBack + 0.64):
				continue

			localMap[round(point[0] / self.cell)][round(point[1] / self.cell)] += 1


			print("x:{}, y:{}, z:{}".format(point[0], point[1], point[2]))
			env.append(env)
		#env = np.asarray(env)
		#print(env.shape)	
			
if __name__ == '__main__':
	obsAvoidance()










