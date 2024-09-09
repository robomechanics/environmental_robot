#!/usr/bin/env python3

#Generates fake PXRF data for use in simulation and testing purposes
#Uploads this data into fake_chemistry.csv for general use
#Incorporated into a rosservice and that can be launched and run 
#Mimics the data in chemistry.csv
import csv
import rospy
import rospkg
from std_msgs.msg import String, Float32, Header
from pxrf.msg import CompletedScanData
from pxrf.srv import GetPxrf, GetPxrfResponse
from pxrf.msg import CompletedScanData
import sys
import os
import cv2
import numpy as np
import datetime, random, copy
from autonomy_manager.srv import Complete, CompleteResponse
from sensor_msgs.msg import NavSatFix, Image
from cv_bridge import CvBridge

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

        # Publishers
        self.fake_pxrf_command_pub = rospy.Publisher(self._fake_pxrf_cmd_topic, String,
                                                     queue_size=1)
        self.fake_scan_completed_pub = rospy.Publisher(self._fake_scan_completed_topic,
                                                       CompletedScanData, queue_size=1)
        self.image_pub = rospy.Publisher(self._fake_pxrf_img_topic, Image, queue_size=10)

        # Services
        self._fake_start_scan_service = rospy.Service(self._fake_start_scan_service_name,
                                                      Complete, self.fake_scan_start_callback)
        #above service is the one to call to generate fake pxrf data

        # Subscribers
        rospy.Subscriber(self._gps_topic, NavSatFix, self.gps_callback)
        rospy.Subscriber(self._fake_pxrf_img_width_topic, Float32, self.width_callback)

        self.bridge = CvBridge()
        self.width = None
        self.latitude = 0.0
        self.longitude = 0.0
        self.fileContainingFakeData = 'fake_chemistry.csv'
        self.scan = []
        self.measuredElements = ['Mg','Al','Si','P','S','Cl','Ca','Ti','V','Cr','Mn',
                                 'Fe','Co','Ni','Cu','Zn','As','Se','Rb','Sr','Y','Zr',
                                 'Nb','Mo','Ag','Cd','Sn','Sb','Ba','La','Ce','Pr','Nd',
                                 'W','Hg','Pb','Bi','Th','U','LE'] #referenced from the chemistry.csv file

        rospy.loginfo("Init fake pxrf handler")
        rospy.spin()
    
    def load_fake_ros_params(self): 
        #see constants.yaml for the names of topics, services and nodes when calling in terminal
        self._fake_pxrf_cmd_topic = rospy.get_param("fake_pxrf_cmd_topic")
        self._fake_pxrf_response_topic = rospy.get_param("fake_pxrf_response_topic")
        self._fake_pxrf_data_topic = rospy.get_param("fake_pxrf_data_topic")
        self._gps_topic = rospy.get_param("gq7_ekf_llh_topic") 
        self._fake_scan_completed_topic = rospy.get_param("fake_scan_completed_topic")
        self._fake_start_scan_service_name = rospy.get_param("fake_start_scan_service_name")
        self._algorithm_type_param_name = rospy.get_param("algorithm_type_param_name")
        self._fake_pxrf_img_topic = rospy.get_param("fake_pxrf_img_topic")
        self._fake_pxrf_img_width_topic = rospy.get_param("fake_pxrf_img_width_topic")

        self.algorithm_type = rospy.get_param(self._algorithm_type_param_name)
        self.fake_root_data_dir = os.path.expanduser(rospy.get_param("data_dir"))
        self.fake_element_of_interest = rospy.get_param("fake_element_of_interest")
    
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

        concentrations = [((concentrations[i]/sum(concentrations))*100) for i in range(len(concentrations))]
        return concentrations
    
    #add the error [list of predicted element error range] currently modelled after a basic exponential distribution
    #methodology could certaintly be improved
    def generate_fake_pxrf_errors(self):
        averageError = 1 / 10**3
        return [random.expovariate(1/averageError) for i in range(len(self.measuredElements))]
    
    def write_to_fake_chemistry(self):
        with open(os.path.join(pxrf_path, 'scripts', 'fake_pxrf', self.fileContainingFakeData), mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(self.scan)

    #used to update the dailyID and scanID
    def getAndUpdateFakeScans(self, filepath, mode):
        with open(os.path.join(pxrf_path, 'scripts','fake_pxrf', filepath), 'r') as file: 
            totalFakeScans = file.read()
        descriptor, fakeScans = totalFakeScans.split(':')
        fakeScans = int(fakeScans)

        if mode == "daily" and descriptor != str(datetime.date.today()):
            fakeScans = 0
        fakeScans += 1

        with open(os.path.join(pxrf_path, 'scripts','fake_pxrf',filepath), 'w') as file: #rewrite to the file
            if mode == "daily": file.write(f'{datetime.date.today()}:{fakeScans}')
            else: file.write(f'numFakeScansMade:{fakeScans}')
        return fakeScans
    
    #generates fake_pxrf_data and writes to the fake_chemistry.csv file, does not publish to a topic
    def generate_fake_pxrf_data(self):

        dailyID = self.getAndUpdateFakeScans("dailyFakeScansMade.txt", "daily")
        numFakeScans = self.getAndUpdateFakeScans("numFakeScansMade.txt", "num")

        self.scan.append([dailyID, numFakeScans, #header
                          f'{datetime.date.today()} {datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]}',
                          f'Lon: {self.longitude} Lat: {self.latitude}'])

        self.scan.append(self.measuredElements)
        self.scan.append(self.generateFakeConcentrations("fakeSoilConcentrationRanges.txt")) #path to a file containing ranges        
        self.scan.append(self.generate_fake_pxrf_errors())
        FakePXRFHandler.fakeScans.append(self.scan)

        self.write_to_fake_chemistry()
        
        fake_data = copy.copy(self.scan)
        self.scan = [] #reset the scan 
        
        return fake_data

    def fake_scan_start_callback(self, req):
        self.fake_data_file = os.path.join(self.fake_root_data_dir, "fake_scan_results_" + self.algorithm_type + ".csv")
        
        self.fake_pxrf_command_pub.publish("start fake scan")
        
        # generate and process fake data
        fake_data = self.generate_fake_pxrf_data()
        header, element, concentration, error = fake_data[0], fake_data[1], fake_data[2], fake_data[3]
        
        

        #write to fake_data_file, different file than fake_chemistry.csv but both are written to
        #this file has one element, fake_chemistry.csv has all the elements
        os.makedirs(self.fake_root_data_dir, exist_ok=True) # Make data dir if it doesn't exist
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
    
        return CompleteResponse(success=True)

    def gps_callback(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude

    def width_callback(self, msg):
        self.width = int(msg.data)
        rospy.loginfo("Fake pxrf received width: %d", self.width)
        self.pub_pxrf_img()

    def pub_pxrf_img(self, ):
        if self.width is None:
            rospy.logwarn("Width not received yet.")
            return

        # Gaussian function
        def gaussian(x, y, cx, cy, sigma):
            return np.exp(-((x - cx) ** 2 + (y - cy) ** 2) / (2 * sigma ** 2))

        # Create a heatmap image
        image = np.zeros((self.width, self.width), dtype=np.float32)

        # Define Gaussian parameters for the heatmap
        center1 = (self.width // 3, self.width // 3)
        center2 = (2 * self.width // 3, 2 * self.width // 3)
        sigma = self.width // 6

        # Fill the image with Gaussian distributions
        for x in range(self.width):
            for y in range(self.width):
                image[x, y] = (gaussian(x, y, *center1, sigma) +
                               gaussian(x, y, *center2, sigma))

        # Normalize image to 0-255
        image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
        image = np.uint8(image)

        # Apply a colormap to the image
        color_image = cv2.applyColorMap(image, cv2.COLORMAP_JET)

        # Convert to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        # Assign header with frame_id
        ros_image.header = Header()
        ros_image.header.stamp = rospy.Time.now()
        ros_image.header.frame_id = "base_link"

        # Publish image
        self.image_pub.publish(ros_image)
        rospy.loginfo("Published pxrf image.")


if __name__ == '__main__':
    FakePXRFHandler()
