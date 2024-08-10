#!/usr/bin/env python3

import csv
import time
import rospy
import statistics
from std_msgs.msg import String
from pxrf.msg import PxrfMsg
import sys
sys.path.insert(0,"/home/hebi/catkin_ws/src/environmental_robot/pxrf/scripts")
from sensor_msgs.msg import NavSatFix

# add pxrf's plot script to lookup path
import os
import rospkg
rospack = rospkg.RosPack()
pxrf_path = rospack.get_path('pxrf')
sys.path.insert(0, os.path.abspath(os.path.join(pxrf_path, "scripts")))

from plot import generate_plot

class CHEMISTRY_PARSER:
    def __init__(self):

        self.dailyId = -1
        self.testId = -1
        self.testDateTime = ""
        self.chemistry = ""
        self.numData = 0
        self.lon_list = []
        self.lat_list = []
        self.testStopped = False
        self.longitude = 0
        self.latitude = 0
        self.location = ""
        # initialize node
        rospy.init_node('chemistry_parser', anonymous=True)
        print("Parser started")
        # Subscriber
        rospy.Subscriber("pxrf_data", PxrfMsg, self.writeData)
        rospy.Subscriber("pxrf_response", String, self.listener)
        rospy.Subscriber("gnss1/fix",NavSatFix,self.gps)
        # spin
        rospy.spin()

    def listener(self, msg):
        if msg.data == "201": # test stopped response
            self.testStopped = True
            print("Test complete")
            
    def gps(self,msg):
        if (msg.longitude != None and msg.latitude != None and msg.longitude != 0 and msg.latitude != 0):
            self.lon_list.append(msg.longitude)
            self.lat_list.append(msg.latitude)
            #self.latitude = (self.latitude * self.numData + msg.latitude) / self.numData
	
    def writeData(self, msg):
        self.dailyId = msg.dailyId
        self.testId = msg.testId
        self.testDateTime = msg.testDateTime
        self.chemistry = msg.chemistry
        
        if self.testStopped == True:
        	self.location = "Lon: " + str(statistics.mean(self.lon_list)) + " Lat: " + str(statistics.mean(self.lat_list))
            s = self.chemistry
            s = s.strip()
            s = s.replace(" ", "") # remove whitespace
            s = s.replace("\"","") # remove random backslashes
            s = s.replace("\\","") # remove quotes
            s = s.replace("[","") # remove array beginning
            s = s.replace("]","") # remove array ending
            s = s.replace("{", "")
            s = s.replace("}","")
            s = s.replace("\n","")
            s = s.replace("concentration:","")
            s = s.replace("elementName:","")
            s = s.replace("error:","")
            arr = s.split(",")

            header = [self.dailyId, self.testId, self.testDateTime,self.location]
            concentration = []
            element = []
            error = []

            for i in range(0, len(arr)):
                if (i % 3 == 0):
                    arr[i] = float(arr[i])
                    concentration.append(arr[i])
                elif (i % 3 == 1):
                    # string element name
                    element.append(arr[i])
                else: # i % 3 == 2
                    arr[i] = float(arr[i])
                    error.append(arr[i])

            with open(os.path.join(pxrf_path, 'scripts','chemistry.csv'), 'a') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerow(element)
                writer.writerow(concentration)
                writer.writerow(error)
                print("Chemistry parsing complete")
                self.testStopped = False
			                
            time.sleep(2)
            generate_plot()

if __name__ == '__main__':
    CHEMISTRY_PARSER()
