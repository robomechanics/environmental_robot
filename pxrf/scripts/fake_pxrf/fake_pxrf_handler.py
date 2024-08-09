#!/usr/bin/env python3


#TO-DO change the sessionID to dailyID

#Generates fake PXRF data for use in simulation and testing purposes
#Uploads this data into fake_chemistry.csv for general use
#Incorporated into a rosservice and that can be launched and run 
#Mimics the data in chemistry.csv
import csv
import copy
import time
import rospy
import rosservice
import rospkg
import statistics
from std_msgs.msg import String
#from nav_msgs.msg import Odometry
from pxrf.msg import PxrfMsg, CompletedScanData
from pxrf.srv import fake_pxrf_response
from autonomy_manager.srv import Complete
import sys
import os


# add pxrf's plot script to lookup path

rospack = rospkg.RosPack()
pxrf_path = rospack.get_path('pxrf')
sys.path.insert(0, os.path.abspath(os.path.join(pxrf_path, "scripts")))

import random
import datetime

longitude = -101.992708
latitude = 31.845601
    
class FakePXRFHandler:
    fakeScans = []
    def __init__(self, longitude, latitude):
        rospy.init_node("fake_pxrf_handler", anonymous=True) #
        self.load_fake_ros_params() #

        self.fake_pxrf_command_pub = rospy.Publisher(self._fake_pxrf_cmd_topic, String, queue_size=1)
        self.fake_scan_completed_pub = rospy.Publisher(self._fake_scan_completed_topic, CompletedScanData, queue_size=1)

        
        #self._fake_pxrf_response_sub = rospy.Subscriber(self._fake_pxrf_response_topic, String, self.response_listener)
        #don't think the above line is neccessary since we aren't 
        self._fake_pxrf_data_sub = rospy.Subscriber(self._fake_pxrf_data_topic, PxrfMsg, self.fake_data_listener)


        self._fake_start_scan_service = rospy.Service(self._fake_start_scan_service_name, Complete, self.fake_scan_start)

        self.longitude = longitude
        self.latitude = latitude
        self.status = True
        self.scanning = False
        self.fakeScansMadeInSesssion = 0
        self.fileContainingFakeData = 'fake_chemistry.csv'
        self.concentrations = []
        self.scan = []
        rospy.init_node('fake_pxrf_data', anonymous=True)
        rospy.Service('/generate_fake_pxrf_data', fake_pxrf_response, 
                      self.generateElementDistributions) 
        
        rospy.spin()
    

    def load_fake_ros_params(self): #modify this and constants file
        self._fake_pxrf_cmd_topic = rospy.get_param("fake_pxrf_cmd_topic")
        self._fake_pxrf_response_topic = rospy.get_param("fake_pxrf_response_topic")
        self._fake_pxrf_data_topic = rospy.get_param("fake_pxrf_data_topic")
        self._gps_topic = rospy.get_param("gq7_ekf_llh_topic") #UNSURE
        self._fake_scan_completed_topic = rospy.get_param("fake_scan_completed_topic")
        self._fake_start_scan_service_name = rospy.get_param("fake_start_scan_service_name")
        self._gps_odom_topic = rospy.get_param("gps_odom_topic")
        self._rotate_scan_log_file_service_name = rospy.get_param("rotate_scan_log_file_service_name")
        self._autonomy_params_service_name = rospy.get_param("autonomy_params_service_name")
        self._algorithm_type_param_name = rospy.get_param("algorithm_type_param_name")

        self.algorithm_type = rospy.get_param(self._algorithm_type_param_name)
        self.fake_root_data_dir = os.path.expanduser(rospy.get_param("fake_data_dir"))
        self.fake_element_of_interest = rospy.get_param("fake_element_of_interest")
    
    def fake_scan_start(self, data):
        if self.status:
            self.scanning = True
            self.fake_pxrf_command_pub.publish("start fake scan")
        else:
            self.fake_pxrf_command_pub.publish("stop fake scan")
            self.scanning = False
        return True

    def generate_fake_pxrf_data(self):
        measuredElements = ['Mg','Al','Si','P','S','Cl','Ca','Ti','V','Cr','Mn',
                            'Fe','Co','Ni','Cu','Zn','As','Se','Rb','Sr','Y','Zr',
                            'Nb','Mo','Ag','Cd','Sn','Sb','Ba','La','Ce','Pr','Nd',
                            'W','Hg','Pb','Bi','Th','U','LE']

        self.numFakeScansInSesssion += 1
        numFakeScans = self.getAndUpdateNumFakeScans('numFakeScans.txt')
        self.scan.append([self.fakeScansMadeInSesssion, 
                            numFakeScans, 
                            f'{datetime.date.today()} {datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]}',
                            f'Lon: {self.longitude} Lat: {self.longitude}'])

        self.scan.append(measuredElements)

        self.scan.append(self.generateFakeConcentrations('fakeSoilConcentrationRanges.txt'))

        #add fourth line [list of predicted element error range] currently modelled after a basic exponential distribution
        #could be improved
        averageError = 1 / 10**3
        self.scan.append([random.expovariate(1/averageError) for i in range(len(measuredElements))])
        self.fakeScans.append(self.scan)

        with open(self.fileContainingFakeData, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(self.scan)
        
        return copy.copy(self.scan)
        
    def fake_data_listener(self, data: PxrfMsg): #need to heavily modify (I think i've done it, double check)
        if self.scanning and self.test_stopped:
            self.scanning = False
            
            # get ros and system time
            self.system_time = str(datetime.now())
            
            
            # record and process received response
            fake_data = self.generate_fake_pxrf_data()
            header, element, concentration, error = fake_data[0], fake_data[1], fake_data[2], fake_data[3]
            
            self.algorithm_type = rospy.get_param(self._algorithm_type_param_name)
            self.fake_data_file = os.path.join(self.fake_root_data_dir, "scan_results_" + self.algorithm_type + ".csv")
            
            with open(self.fake_data_file, "a+") as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerow(element)
                writer.writerow(concentration)
                writer.writerow(error)

            # Publish results to manager and GUI
            i = element.index(self.fake_element_of_interest)
            completed_scan_data_msg = CompletedScanData()
            completed_scan_data_msg.status = True
            completed_scan_data_msg.element = self.element_of_interest
            completed_scan_data_msg.mean = concentration[i]
            completed_scan_data_msg.error = error[i]
            completed_scan_data_msg.file_name = self.fake_data_file
            self.fake_scan_completed_pub.publish(completed_scan_data_msg)

    def response_listener(self, data):
        rospy.loginfo("Test complete")
        self.test_stopped = (data.data == "201")

    #line contains a string with the lower and upper range of values
    def generateIndividualFakeConcentration(self, line):
        words = line.split()
        if words[1] == 'Trace':
            return 0.0
    
        _, lowerRange, upperRange = words[0][:-1], words[1][:-1], words[3][:-1]
        return random.uniform(float(lowerRange), float(upperRange))
    
    #random fake concentration parsing is roughly based on the ranges in fake_soil_concentration_ranges.txt
    def generateFakeConcentrations(self, filename):
        with open(filename, 'r') as file:
            fakeConcentrationRanges = file.read()

        concentrations = [] 
        for line in fakeConcentrationRanges.splitlines(): 
            elementConcentration = self.generateIndividualFakeConcentration(line)
            concentrations.append(elementConcentration)

        total = sum(concentrations)
        concentrations = [((concentrations[i]/total)*100) for i in range(len(concentrations))]
        return concentrations

    def getAndUpdateNumFakeScans(self, filepath):
        with open(filepath, 'r') as file: 
            totalFakeScans = file.read()
        totalFakeScans = int(totalFakeScans.split(':')[1])
        totalFakeScans += 1
        with open(filepath, 'w') as file: #rewrite to the file
            file.write(f'numFakeSScandMade:{totalFakeScans}')
        return totalFakeScans


if __name__ == '__main__':
    FakePXRFHandler(longitude, latitude)
