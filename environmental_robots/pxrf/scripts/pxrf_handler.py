#!/usr/bin/env python3
import rospy
import rosservice
from pxrf.msg import PxrfMsg
from std_msgs.msg import String
from autonomy_manager.srv import Complete, ScanData
import os
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from datetime import date
import csv
import time

def chemistryParser(chemistry):
    s = chemistry.strip()
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
    return element, concentration, error

class pxrf_handler(object):
    def __init__(self,dataDir,elementOfInterest):
        self.dataDir = dataDir
        self.elementOfInterest = elementOfInterest
        self.load_ros_params()
        rospy.init_node(self._node_name, anonymous=True)
        self.pxrfCommand = rospy.Publisher('pxrf_cmd', String, queue_size=1)
        rospy.Subscriber(self._pxrf_response_topic, String, self.responseListener)
        rospy.Subscriber(self._pxrf_data_topic, PxrfMsg, self.dataListener)
        rospy.Service(self._scan_service,Complete, self.scan_start)
        rospy.Subscriber(self._odometry_topic, Odometry, self.odometry)
        self.scanning = False
        self.location = [0, 0]
        rospy.spin()

    def load_ros_params(self):
        usingYAML = False
        if (usingYAML == True):
            self._node_name = rospy.get_param('node_name')
            self._pxrf_response_topic = rospy.get_param('pxrf_response_topic')
            self._pxrf_data_topic = rospy.get_param('pxrf_data_topic')
            self._scan_service = rospy.get_param('scan_service')
            self._odometry_topic = rospy.get_param('odometry_topic')
        else:
            self._node_name = 'pxrf_handler'
            self._pxrf_response_topic = "pxrf_response" #self._variable style
            self._pxrf_data_topic = "pxrf_data"
            self._scan_service = 'scan_start'
            self._odometry_topic = 'odometry'

    def odometry(self, data):
        self.location[0] = data.pose.pose.position.y
        self.location[1] = data.pose.pose.position.x

    def scan_start(self,data):
        if data.status:
            self.scanning = True
            self.pxrfCommand.publish("start")
        else:
            self.pxrfCommand.publish("stop")
            self.scanning = False
        return True
    def fileCheck(self):
        if not os.path.exists(self.dataDir):
            os.mkdir(self.dataDir)
        if not os.path.exists(os.path.join(self.dataDir,str(date.today())+'.csv')):
            a = 0 #placeholder for making a csv
        return None
    def dataListener(self,data):
        if self.scanning and self.testStopped:
            self.scanning = False
            #get ros and system time
            self.systemTime = time.localtime()
            self.rosTime = rospy.Time.now()
            #record and process received response
            element, concentration, error = chemistryParser(data.chemistry)
            header = [data.dailyId, data.testId, data.testDateTime, self.location, self.systemTime, self.rosTime]
            self.fileCheck()
            with open(os.path.join(self.dataDir,str(date.today())+'.csv'), 'a+') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerow(element)
                writer.writerow(concentration)
                writer.writerow(error)
            # return result to manager
            if not '/pxrf_complete' in rosservice.get_service_list():
                return
            scan_result_proxy = rospy.ServiceProxy('/pxrf_complete', ScanData)
            i = element.index(self.elementOfInterest)
            scan_result_proxy(True,self.elementOfInterest,concentration[i],error[i])

    def responseListener(self,data):
        print("test complete")
        self.testStopped = data.data == "201"

if __name__ == "__main__":
    dataDir = os.path.expanduser('~/pxrf_results')
    # if os.path.isdir(dataDir):
    #     os.mkdir(dataDir)
    elementOfInterest = 'Cl'
    pxrf_handler(dataDir,elementOfInterest)
