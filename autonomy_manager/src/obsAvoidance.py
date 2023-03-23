#/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose
from std_srvs.srv import SetBool, SetBoolRequest
from std_msgs.msg import Float64, Header
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
import matplotlib as plt 
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class obsAvoidance(object):
	def __init__(self):
		rospy.init_node('obstacleAvoidance', anonymous=True)
		rospy.Subscriber('/ouster/points', PointCloud2, self.obsMap)
		self.cost_map_pub = rospy.Publisher('/costmap', OccupancyGrid, queue_size=1)
		rospy.Subscriber('/utm_odom2', Odometry, self.odomCallback)

		#define class variables
		self.heightThreshLow = -0.4
		self.heightThreshHigh = 0.1
		
		self.frontBack = 0.5
		self.side = 0.3
		
		self.x = self.side * 2 + 0.6
		self.y = self.frontBack * 2 + 0.8
		
		#discretization
		self.cell = 0.05
		self.threshold = 2

		# map size
		self.mapx = self.x / self.cell
		self.mapy = self.y / self.cell

		#pose
		self.pose = Pose()
		self.time = None 
		#rate = rospy.Rate(10)
		#rate.sleep()
		rospy.spin()	

	def odomCallback(self, data):
			
		self.pose = data.pose.pose
		self.time = data.header.stamp
		# get the current position of the robot

	def create_occupancy_grid(self, cost_map_2d):
		grid = OccupancyGrid()
		
		# Customize the header information
		grid.header = Header()
		grid.header.stamp = self.time
		grid.header.frame_id = "costmap_frame"
		
		# Set the properties of the OccupancyGrid
		grid.info.resolution = 0.1  # resolution in meters
		grid.info.width = len(cost_map_2d[0])
		grid.info.height = len(cost_map_2d)
		grid.info.origin.position = self.pose.position
		grid.info.origin.orientation = self.pose.orientation
		
		flat_map = [cell for row in cost_map_2d for cell in row]
		
		# Assign the flattened cost map to the OccupancyGrid data
		grid.data = flat_map

		return grid
	
	def obsMap(self, data):
		env = []
		print(self.mapx)
		localMap = np.full((round(self.mapx), round(self.mapy)), 0)
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
			# print("x:{}, y:{}, z:{}".format(point[0], point[1], point[2]))
		costMap = (localMap > self.threshold).astype(int)

		og = self.create_occupancy_grid(costMap)
		self.cost_map_pub.publish(og)
		#env.append(env)
	
if __name__ == '__main__':
	try:
		obsAvoidance()
	except rospy.ROSInterruptException:
		pass









