#!/usr/bin/env python3
import rospy
import rosservice
from pxrf.msg import PxrfMsg, CompletedScanData
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from autonomy_manager.srv import Complete
import os
from sensor_msgs.msg import NavSatFix
import csv
import time
from tf.transformations import euler_from_quaternion
import numpy as np
from datetime import datetime
from std_srvs.srv import Empty
from autonomy_manager.srv import AutonomyParams
import yaml

def chemistry_parser(chemistry):
    s = chemistry.strip()
    s = s.replace(" ", "")  # remove whitespace
    s = s.replace("\"", "")  # remove random backslashes
    s = s.replace("\\", "")  # remove quotes
    s = s.replace("[", "")  # remove array beginning
    s = s.replace("]", "")  # remove array ending
    s = s.replace("{", "")
    s = s.replace("}", "")
    s = s.replace("\n", "")
    s = s.replace("concentration:", "")
    s = s.replace("elementName:", "")
    s = s.replace("error:", "")
    arr = s.split(",")
    concentration = []
    element = []
    error = []
    for i in range(0, len(arr)):
        if i % 3 == 0:
            arr[i] = float(arr[i])
            concentration.append(arr[i])
        elif i % 3 == 1:
            # string element name
            element.append(arr[i])
        else:  # i % 3 == 2
            arr[i] = float(arr[i])
            error.append(arr[i])
    return element, concentration, error

class PXRFHandler(object):
    def __init__(self):
        rospy.init_node("pxrf_handler", anonymous=True)
        self.load_ros_params()
        
        self.create_log_file()

        self.pxrf_command_pub = rospy.Publisher(self._pxrf_cmd_topic, String, queue_size=1)
        self.scan_completed_pub = rospy.Publisher(self._scan_completed_topic, CompletedScanData, queue_size=1)
        
        self._pxrf_response_sub = rospy.Subscriber(self._pxrf_response_topic, String, self.response_listener)
        self._pxrf_data_sub = rospy.Subscriber(self._pxrf_data_topic, PxrfMsg, self.data_listener)
        self._gps_sub = rospy.Subscriber(self._gps_topic, NavSatFix, self.gps_callback)
        self.utm_odom_sub = rospy.Subscriber(
            self._gps_odom_topic, Odometry, self.gps_odom_callback
        )
        
        self._start_scan_service = rospy.Service(self._start_scan_service_name, Complete, self.scan_start)
        self._rotate_log_file_service = rospy.Service(self._rotate_scan_log_file_service_name, Empty, self.rotate_log_file_callback)
        self._autonomy_params_service = rospy.Service(self._autonomy_params_service_name, AutonomyParams, self.autonomy_params_callback)
        
        self.scanning = False
        self.gps_location = [0, 0]
        self.gps_odom_location = [0, 0]
        self.gps_quaternion = np.zeros(4)
        self.gps_odom_heading = 0
        
        rospy.loginfo("Started PXRF Handler...")
        
        rospy.spin()

    def load_ros_params(self):
        self._pxrf_cmd_topic = rospy.get_param("pxrf_cmd_topic")
        self._pxrf_response_topic = rospy.get_param("pxrf_response_topic")
        self._pxrf_data_topic = rospy.get_param("pxrf_data_topic")
        self._gps_topic = rospy.get_param("gq7_ekf_llh_topic")
        self._scan_completed_topic = rospy.get_param("scan_completed_topic")
        self._start_scan_service_name = rospy.get_param("start_scan_service_name")
        self._gps_odom_topic = rospy.get_param("gps_odom_topic")
        self._rotate_scan_log_file_service_name = rospy.get_param("rotate_scan_log_file_service_name")
        self._autonomy_params_service_name = rospy.get_param("autonomy_params_service_name")
        
        self.root_data_dir = os.path.expanduser(rospy.get_param("data_dir"))
        self.element_of_interest = rospy.get_param("element_of_interest")
    
    def gps_odom_callback(self, data: Odometry):
        
        self.gps_odom_location = [data.pose.pose.position.x, data.pose.pose.position.y]
        
        self.gps_quaternion[0] = data.pose.pose.orientation.x
        self.gps_quaternion[1] = data.pose.pose.orientation.y
        self.gps_quaternion[2] = data.pose.pose.orientation.z
        self.gps_quaternion[3] = data.pose.pose.orientation.w
        self.gps_odom_heading = euler_from_quaternion(self.gps_quaternion, "sxyz")[2]

    def gps_callback(self, data:NavSatFix):
        self.gps_location[0] = data.latitude
        self.gps_location[1] = data.longitude

    def scan_start(self, data):
        if data.status:
            self.scanning = True
            self.pxrf_command_pub.publish("start")
        else:
            self.pxrf_command_pub.publish("stop")
            self.scanning = False
        return True

    def create_log_file(self):
        self.data_dir = os.path.join(self.root_data_dir, datetime.now().strftime("%d-%m-%Y_%H:%M:%S"))
        self.data_file = os.path.join(self.data_dir, "scan_results.csv")
        self.yaml_file = os.path.join(self.data_dir, "params.yaml")
        
        if not os.path.exists(self.data_dir):
            rospy.loginfo(f" |Creating directory {self.data_dir}")
            os.makedirs(self.data_dir)
        
        if not os.path.exists(self.data_file):
            rospy.loginfo(f" |Creating file {self.data_file}")
            with open(self.data_file, 'w') as fp:
                pass
    
    def autonomy_params_callback(self, data: AutonomyParams):
        data_info = {
            "boundary_lat": list(data.boundary_lat),
            "boundary_lon": list(data.boundary_lon),
            "width": data.width,
            "height": data.height,
            "total_samples_count": data.total_samples_count,
            "start_utm_x": data.start_utm_x,
            "start_utm_y": data.start_utm_y,
            "start_utm_lat": data.start_utm_lat,
            "start_utm_lon": data.start_utm_lon
            }
        
        rospy.loginfo(f"{data_info}")
        
        self.create_log_file()
        
        with open(self.yaml_file, "w") as file:
            yaml.dump(data_info, file)
        rospy.loginfo(f"Saved Params into {self.yaml_file}")
        
        return True
     
    def rotate_log_file_callback(self, data):
        self.create_log_file()
        return True

    def data_listener(self, data: PxrfMsg):
        if self.scanning and self.test_stopped:
            self.scanning = False
            
            # get ros and system time
            self.system_time = str(datetime.now())
            self.ros_time = rospy.Time.now()
            
            # record and process received response
            element, concentration, error = chemistry_parser(data.chemistry)
            header = [
                data.dailyId,
                data.testId,
                data.testDateTime,
                self.system_time,
                self.ros_time,
                self.gps_location,
                self.gps_odom_location,
                self.gps_odom_heading,
            ]
            
            with open(self.data_file, "a+") as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerow(element)
                writer.writerow(concentration)
                writer.writerow(error)

            # Publish results to manager and GUI
            i = element.index(self.element_of_interest)
            completed_scan_data_msg = CompletedScanData()
            completed_scan_data_msg.status = True
            completed_scan_data_msg.element = self.element_of_interest
            completed_scan_data_msg.mean = concentration[i]
            completed_scan_data_msg.error = error[i]
            completed_scan_data_msg.file_name = self.data_file
            self.scan_completed_pub.publish(completed_scan_data_msg)

    def response_listener(self, data):
        rospy.loginfo("Test complete")
        self.test_stopped = (data.data == "201")


if __name__ == "__main__":
    PXRFHandler()
