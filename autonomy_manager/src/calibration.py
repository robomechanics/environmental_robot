#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry
from microstrain_inertial_msgs.msg import FilterHeading
import math
import numpy as np
from collections import deque
from std_msgs.msg import Float64, Bool
from std_srvs.srv import SetBool
from sensor_msgs.msg import NavSatFix
from autonomy_manager.srv import RunSensorPrep

from pyproj import CRS
from pyproj import Transformer
from pyproj import Proj
from pyproj.database import query_utm_crs_info
from pyproj.aoi import AreaOfInterest
import pyproj

class calibration(object):
    def __init__(self):

        # start node
        rospy.init_node('calibration', anonymous=True)
        rospy.Subscriber('/nav/heading',FilterHeading,self.heading_callback)
        rospy.Subscriber('/gnss1/fix',NavSatFix,self.gps_callback)
        rospy.Subscriber('/cmd_vel',Twist,self.speed_callback)
        self.startService = rospy.Service('start', RunSensorPrep, self.startService)
        self.heading = None
        self.queueSize = 100
        self.lon = None
        self.lat = None
        self.lon_avg = deque(maxlen=self.queueSize)
        self.lat_avg = deque(maxlen=self.queueSize)
        self.offsetParam = None
        self.linear = 0
        self.angular = 0
        self.counter = 0
        self.time = 0
        self.gps_msg = NavSatFix()
        self.heading_msg = FilterHeading()
        self.start = False
        self.utmDefault = 'EPSG:32617'

        while self.lon is None or self.lat is None or self.start == False:
            print(self.lon, self.lat, self.start)
            rospy.sleep(1)

        self.movefb = rospy.ServiceProxy('/move_fb',RunSensorPrep)
        self.gps_filter = rospy.Publisher('/gps_avg', NavSatFix, queue_size=1)
        self.heading = rospy.Publisher('/heading_true', FilterHeading, queue_size=1)
        # start calibration
        if self.offsetParam == None:
            #reset dequeue
            print("here2")
            self.lat_avg.clear()
            self.lon_avg.clear()
            rospy.sleep(5)
            lat_init = sum(self.lat_avg) / len(self.lat_avg)
            lon_init = sum(self.lon_avg) / len(self.lon_avg)
            # move forward
            self.movefb(True)
            while self.linear != 0 or self.angular != 0:
                rospy.sleep(0.1)
                print("moving forward")
            #clear the dequeue 
            self.lat_avg.clear()
            self.lon_avg.clear()
            rospy.sleep(5) # wait for 5 seconds
            lat_final = sum(self.lat_avg) / len(self.lat_avg)
            lon_final = sum(self.lon_avg) / len(self.lon_avg)
            self.movefb(False)
            # calculate offset
            self.calculate_offset(lat_init,lon_init,lat_final,lon_final)
            # get the zone 
            self.get_zone(lat_final,lon_final)
            print("offset is computed")
        
        while not rospy.is_shutdown():
            if self.linear == 0 and self.angular == 0:
                self.gps_msg.longitude = sum(self.lon_avg) / len(self.lon_avg)
                self.gps_msg.latitude = sum(self.lat_avg) / len(self.lat_avg)
                self.gps_filter.publish(self.gps_msg)
            else:
               
                self.gps_filter.publish(self.gps_msg)
            # todo, compute the average heading
            print("corrected Angle:" + str(self.heading_msg.heading_rad + self.offsetParam))
            self.heading.publish(self.heading_msg.heading_rad + self.offsetParam)

    def calculate_offset(self ,lat_init,lon_init,lat_final,lon_final):
        x_UTM_init, y_UTM_init = self.get_utm(lat_init, lon_init) 
        x_UTM_final, y_UTM_final = self.get_utm(lat_final, lon_final) 
        delta_x = x_UTM_final - x_UTM_init
        delta_y = y_UTM_final - y_UTM_init
        heading = np.arctan2(delta_y, delta_x)
        self.offsetParam = heading - self.heading_msg.heading_rad 
        print("offset:" + str(self.offsetParam))
    
    def heading_callback(self,msg):
        self.heading_msg = msg
        #self.heading = msg.heading_rad
       
    def gps_callback(self,msg):
        
        ##################################
        msg.longitude = 10
        msg.latitude = 10
        ##################################3

        
        # if the data is invalid, don't use it
        if msg.longitude == 0 or msg.latitude == 0 or msg.longitude == None or msg.latitude == None:
            return
        self.gps_msg = msg
        self.lon = msg.longitude
        self.lat = msg.latitude
        self.time = msg.header.stamp
        self.lon_avg.append(self.lon)
        self.lat_avg.append(self.lat)
    
    def speed_callback(self,msg):
        if self.linear != 0 or self.angular != 0: # in motion
            if msg.linear.x == 0 and msg.angular.z == 0: # stopped
                # clear the dequeue
                self.lat_avg.clear()
                self.lon_avg.clear()
        self.linear = msg.linear.x
        self.angular = msg.angular.z

    def startService(self,msg):
        self.start = msg.run
        return True
    
    def get_zone(self, posx, posy):
        utm_crs_list = query_utm_crs_info( 
        datum_name="WGS 84", 
        area_of_interest=AreaOfInterest( 
            west_lon_degree=posy, 
            south_lat_degree=posx, 
            east_lon_degree=posy, 
            north_lat_degree=posx, 
            ), 
        )
        utm_crs = CRS.from_epsg(utm_crs_list[0].code)
        self.utmDefault = utm_crs
        return utm_crs

    # get utm positions
    def get_utm(self, posx, posy):
        wgs84 = pyproj.CRS('EPSG:4326') # WGS 84
        utm17n = pyproj.CRS(self.utmDefault) 
        transformer = pyproj.Transformer.from_crs(wgs84, utm17n)
        return transformer.transform(posx, posy)

if __name__ == "__main__":
    calibration()
