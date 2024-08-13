#!/usr/bin/env python3

#Generates fake PXRF data for use in simulation and testing purposes
#Uploads this data into fake_chemistry.csv for general use
#Incorporated into a rosservice and that can be launched and run 
#Mimics the data in chemistry.csv
import csv
import rospy
import rosservice
import rospkg
from std_msgs.msg import String
from pxrf.msg import PxrfMsg, CompletedScanData
from std_srvs.srv import SetBool, SetBoolResponse
from pxrf.srv import GetPxrf, GetPxrfResponse
import sys
import os
import datetime, time, random, copy


# add pxrf's plot script to lookup path
sys.path.insert(0,"/home/hebi/catkin_ws/src/environmental_robot/pxrf/scripts") #test to see if this is needed
rospack = rospkg.RosPack()
pxrf_path = rospack.get_path('pxrf') 
sys.path.insert(0, os.path.abspath(os.path.join(pxrf_path, "scripts", "fake_pxrf")))
sys.path.insert(0, os.path.abspath(os.path.join(pxrf_path, "scripts")))
    
class FakePXRFHandler:
    fakeScans = []
    def __init__(self):
        rospy.init_node("fake_pxrf_handler", anonymous=True) 
        self.load_fake_ros_params() 

        self.fake_pxrf_command_pub = rospy.Publisher(self._fake_pxrf_cmd_topic, String,
                                                     queue_size=1)
        self.fake_scan_completed_pub = rospy.Publisher(self._fake_scan_completed_topic,
                                                       CompletedScanData, queue_size=1)
        self._fake_start_scan_service = rospy.Service(self._fake_start_scan_service_name,
                                                      GetPxrf, self.fake_scan_start_callback)

        self.status = True
        self.scanning = False
        self.fakeScansMadeInSesssion = 0
        self.fileContainingFakeData = 'fake_chemistry.csv'
        self.concentrations = []
        self.scan = []
        self.measuredElements = ['Mg','Al','Si','P','S','Cl','Ca','Ti','V','Cr','Mn',
                            'Fe','Co','Ni','Cu','Zn','As','Se','Rb','Sr','Y','Zr',
                            'Nb','Mo','Ag','Cd','Sn','Sb','Ba','La','Ce','Pr','Nd',
                            'W','Hg','Pb','Bi','Th','U','LE']
        
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
        self.fake_root_data_dir = os.path.expanduser(rospy.get_param("data_dir"))
        self.fake_element_of_interest = rospy.get_param("fake_element_of_interest")
    
    def fake_scan_start_callback(self, req):
        if not req.data:
            return SetBoolResponse(True,"Fake Scan not Initated")
        
        self.longitude = req.longitude
        self.latitude = req.latitude
        
        self.fake_pxrf_command_pub.publish("start fake scan")
        
        # generate and process fake data
        fake_data = self.generate_fake_pxrf_data()
        header, element, concentration, error = fake_data[0], fake_data[1], fake_data[2], fake_data[3]
        
        self.algorithm_type = rospy.get_param(self._algorithm_type_param_name)
        self.fake_data_file = os.path.join(self.fake_root_data_dir, "fake_scan_results_" + self.algorithm_type + ".csv")
        
        with open(os.path.join(self.fake_data_file), "a+") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerow(element)
            writer.writerow(concentration)
            writer.writerow(error)

        # Publish results to manager and GUI
        i = element.index(self.fake_element_of_interest)
        completed_scan_data_msg = CompletedScanData()
        completed_scan_data_msg.status = True
        completed_scan_data_msg.element = self.fake_element_of_interest
        completed_scan_data_msg.mean = concentration[i]
        completed_scan_data_msg.error = error[i]
        completed_scan_data_msg.file_name = self.fake_data_file
        self.fake_scan_completed_pub.publish(completed_scan_data_msg)

        self.fake_pxrf_command_pub.publish("Stop Fake Scan")

        return GetPxrfResponse(completed_scan_data_msg.status,
                            completed_scan_data_msg.element,
                            completed_scan_data_msg.mean,
                            completed_scan_data_msg.error,
                            completed_scan_data_msg.file_name)

    #add fourth line [list of predicted element error range] currently modelled after a basic exponential distribution
    #methodology could certaintly be improved
    def generate_fake_pxrf_errors(self):
        averageError = 1 / 10**3
        return [random.expovariate(1/averageError) for i in range(len(self.measuredElements))]
    
    def write_to_fake_chemistry(self):
        with open(os.path.join(pxrf_path, 'scripts', 'fake_pxrf',self.fileContainingFakeData), mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(self.scan)

    def getAndUpdateFakeScans(self, filepath, mode):
        with open(os.path.join(pxrf_path, 'scripts','fake_pxrf', filepath), 'r') as file: 
            totalFakeScans = file.read()
        descriptor, fakeScans = totalFakeScans.split(':')
        fakeScans = int(fakeScans)
        if mode == "daily" and descriptor != datetime.date.today():
            fakeScans = 0
        fakeScans += 1

        with open(os.path.join(pxrf_path, 'scripts','fake_pxrf',filepath), 'w') as file: #rewrite to the file
            if mode == "daily": file.write(f'{datetime.date.today()}:{fakeScans}')
            else: file.write(f'numFakeScansMade:{fakeScans}')
        return fakeScans
    
    #generates fake_pxrf_data and writes to the fake_chemistry.csv file, does not publish
    def generate_fake_pxrf_data(self):
        self.fakeScansMadeInSesssion += 1

        dailyID = self.getAndUpdateFakeScans("dailyFakeScansMade.txt", "daily")
        numFakeScans = self.getAndUpdateFakeScans("numFakeScansMade.txt", "num")

        self.scan.append([dailyID, numFakeScans, #header
                          f'{datetime.date.today()} {datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]}',
                          f'Lon: {self.longitude} Lat: {self.longitude}'])

        self.scan.append(self.measuredElements)
        self.scan.append(self.generateFakeConcentrations("fakeSoilConcentrationRanges.txt")) #path to a file containing ranges        
        self.scan.append(self.generate_fake_pxrf_errors())
        self.fakeScans.append(self.scan)

        self.write_to_fake_chemistry()
        
        fake_data = copy.copy(self.scan)
        self.scan = [] #reset the scan 
        
        return fake_data

    #line contains a string with the lower and upper range of values
    def generateIndividualFakeConcentration(self, line):
        words = line.split()
        if words[1] == 'Trace':
            return 0.0
    
        _, lowerRange, upperRange = words[0][:-1], words[1][:-1], words[3][:-1]
        return random.uniform(float(lowerRange), float(upperRange))
    
    #random fake concentration parsing is roughly based on the ranges in fake_soil_concentration_ranges.txt
    def generateFakeConcentrations(self, filename):
        with open(os.path.join(pxrf_path, 'scripts','fake_pxrf', filename), 'r') as file:
            fakeConcentrationRanges = file.read()

        concentrations = [] 
        for line in fakeConcentrationRanges.splitlines(): 
            elementConcentration = self.generateIndividualFakeConcentration(line)
            concentrations.append(elementConcentration)

        total = sum(concentrations)
        concentrations = [((concentrations[i]/total)*100) for i in range(len(concentrations))]
        return concentrations

if __name__ == '__main__':
    FakePXRFHandler()
