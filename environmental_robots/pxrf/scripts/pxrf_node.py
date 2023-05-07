#!/usr/bin/env python3
import time
from tkinter import *
import rospy
from std_msgs.msg import String
import sys
from sensor_msgs.msg import NavSatFix
import os
import rospkg
from autonomy_manager.srv import DeployAutonomy, NavigateGPS, RunSensorPrep
from pxrf.msg import PxrfMsg
rospack = rospkg.RosPack()
pxrf_path = rospack.get_path('pxrf')
sys.path.insert(0, os.path.abspath(os.path.join(pxrf_path, "scripts")))



class pxrfNode:
    def __init__(self):
        rospy.init_node('pxrf_node')
        self.cmd_pub = rospy.Publisher('pxrf_cmd', String, queue_size=1)
        self.cmd_sub = rospy.Subscriber('pxrf_response', String, self.pxrf_cb)
        self.gps = rospy.Subscriber("/gnss1/fix", NavSatFix, self.location)
        self.lon = 0
        self.lat = 0
        self.pxrfRunning = False
        rospy.spin()

    def pxrf_cb(self, msg):
        if msg.data == "201":
            self.pxrfRunning = False

    def location(self,msg):
        if (msg.longitude != None and msg.latitude != None and msg.longitude != 0 and msg.latitude != 0):
            self.lon = msg.longitude
            self.lat = msg.latitude
    

    def execute(self, goal):
        self.pxrf_response = 'NoResponse'
        self.should_stop = False

        sample_time = float(rospy.get_param('~sample_time', 2.0))

        print('deploying arm')
        self.deploy_pxrf(True)
        rospy.sleep(2)

        self.cmd_pub.publish(String('start'))

        print('sampling...')
        rospy.Timer(rospy.Duration.from_sec(sample_time), self.stop_sampling, oneshot=True)
        while self.pxrf_response != '201' and not self.should_stop:
            rospy.sleep(0.1)
        print('Sample Complete!')

        if self.pxrf_response != '201':
            self.cmd_pub.publish(String('stop'))

        self.deploy_pxrf(False)
        rospy.sleep(2)
        print('arm stowed')

        result = TakeMeasurementResult()
        result.result.data = self.pxrf_response
        self.server.set_succeeded(result)


if __name__ == "__main__":
    
    server = pxrfNode()
    

    

