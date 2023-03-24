#/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose
import geometry_msgs.msg
from std_srvs.srv import SetBool, SetBoolRequest
from std_msgs.msg import Float64, Header
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
import matplotlib as plt 
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf
import tf2_ros, tf2_geometry_msgs

class obsAvoidance(object):
        def __init__(self):
                rospy.init_node('obstacleAvoidance', anonymous=True)
                rospy.Subscriber('/ouster/points', PointCloud2, self.obsMap)
                self.cost_map_pub = rospy.Publisher('/costmap', OccupancyGrid, queue_size=1)
                rospy.Subscriber('/utm_odom2', Odometry, self.odomCallback)
                self.staticBr = tf2_ros.StaticTransformBroadcaster()

                #define class variables
                self.heightThreshLow = -0.4
                self.heightThreshHigh = 0.2
                
                self.frontBack = 0.6 * 2
                self.side = 0.4 * 2
                
                self.x = self.side * 2 + 0.6
                self.y = self.frontBack * 2 + 0.8
                
                #discretization
                self.cell = 0.05
                self.threshold = 3

                # map size
                self.mapx = self.x / self.cell
                self.mapy = self.y / self.cell

                #pose
                self.pose = Pose()
                self.time = None 
                rospy.spin()    

        def odomCallback(self, data):
                #update the pose and time stamp                        
                self.pose = data.pose.pose
                self.time = data.header.stamp

        def create_occupancy_grid(self, cost_map_2d):
                grid = OccupancyGrid()

                # Customize the header information
                grid.header = Header()
                grid.header.stamp = self.time
                grid.header.frame_id = "costmap_frame"
                
                # Set the properties of the OccupancyGrid
                grid.info.resolution = self.cell  # resolution in meters
                grid.info.width = len(cost_map_2d[0])
                grid.info.height = len(cost_map_2d)
                grid.info.origin.position = self.pose.position
                grid.info.origin.orientation = self.pose.orientation
                #flat the map 
                flat_map = [cell for row in cost_map_2d for cell in row]

                # Assign the flattened cost map to the OccupancyGrid data
                grid.data = flat_map
                return grid
        
        def obsMap(self, data):
                #initialize the variables 
                env = []
                localMap = np.full((round(self.mapx), round(self.mapy)), 0)
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = self.time
                t.header.frame_id = "base_link"
                t.child_frame_id = "costmap_frame"
                t.transform.rotation.w = 1.0
                #offset the point to the correct coordinate
                x_offset = self.x / 2 + 0.14
                y_offset = self.y / 2 + 0.16

                t.transform.translation.x = x_offset -0.14 
                t.transform.translation.y = -y_offset - 0.16
                
                for point in pc2.read_points(data, skip_nans=True):
                        # offset each cloud point         
                        pointx = point[0] + x_offset
                        pointy = point[1] + y_offset
                        pointz = point[2]
                        
                        # if the obstacle height is lower, then skip
                        if pointz < self.heightThreshLow or pointz > self.heightThreshHigh or pointx > self.x or pointy >= self.y or pointx < 0 or pointy < 0:
                                continue

                        # if the point is invalid
                       # elif pointx == pointy == pointz == 0 or abs(pointx < 0.5) or abs(pointy < 0.5):
                       #         continue
                       # # front
                       # elif (pointx < 0 and pointx < 0.65 + self.frontBack) or (pointx < 0 and pointx > -0.65):
                       #         continue
                       # #back
                       # elif (pointx > 0 and pointx > 0.14 + self.frontBack):
                       #         continue
                       # #left
                       # elif (pointy < 0 and pointy < -0.42 - self.side):
                       #         continue
                       # #right
                       # elif (pointy > 0 and pointy > 0.16 + self.side):
                       #         continue
                        localMap[round(pointx / self.cell) - 1][round(pointy / self.cell) - 1] += 1
                        #print("x:{}, y:{}, z:{}".format(point[0], point[1], point[2]))
                
                #localMap = np.flipud(localMap) 
                costMap = (localMap > self.threshold).astype(int)
                
                print(costMap)
                og = self.create_occupancy_grid(np.flipud(costMap))
                self.cost_map_pub.publish(og)
                self.staticBr.sendTransform(t)
                #env.append(env)
        
if __name__ == '__main__':
        try:
                obsAvoidance()
        except rospy.ROSInterruptException:
                pass
