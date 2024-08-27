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

from env_utils.algo_constants import *
from env_utils.pxrf_utils import PXRF, chemistry_parser

class DataRecorder(object):
    def __init__(self):
        rospy.init_node("data_recorder", anonymous=True)
        self.load_ros_params()
        
        self._rotate_log_file_service = rospy.Service(self._rotate_scan_log_file_service_name, Empty, self.rotate_log_files_callback)
        
        
        self._gps_sub = rospy.Subscriber(self._gps_topic, NavSatFix, self.gps_callback)
        self.utm_odom_sub = rospy.Subscriber(self._gps_odom_topic, Odometry, self.gps_odom_callback)
        
        self._pxrf_data_sub = rospy.Subscriber(self._pxrf_data_topic, PxrfMsg, self.pxrf_data_callback)
        self._autonomy_params_service = rospy.Service(self._autonomy_params_service_name, AutonomyParams, self.autonomy_params_data_callback)
        
        self.scan_recorded_to_disk_pub = rospy.Publisher(self._scan_recorded_to_disk_topic, CompletedScanData, queue_size=10, latch=True)
        
        self.gps_location = [0, 0]
        self.gps_odom_location = [0, 0]
        self.gps_quaternion = np.zeros(4)
        self.gps_odom_heading = 0
        
        self.create_logging_files()
        
        rospy.loginfo("Started Data Recorder...")
        rospy.spin()
        
    def load_ros_params(self):
        self._pxrf_cmd_topic = rospy.get_param("pxrf_cmd_topic")
        self._pxrf_response_topic = rospy.get_param("pxrf_response_topic")
        self._pxrf_data_topic = rospy.get_param("pxrf_data_topic")
        self._gps_topic = rospy.get_param("gq7_ekf_llh_topic")
        self._scan_recorded_to_disk_topic = rospy.get_param("scan_recorded_to_disk_topic")
        self._start_scan_service_name = rospy.get_param("start_scan_service_name")
        self._gps_odom_topic = rospy.get_param("gps_odom_topic")
        self._rotate_scan_log_file_service_name = rospy.get_param("rotate_scan_log_file_service_name")
        self._autonomy_params_service_name = rospy.get_param("autonomy_params_service_name")
        self._algorithm_type_param_name = rospy.get_param("algorithm_type_param_name")
        
        self.algorithm_type = rospy.get_param(self._algorithm_type_param_name)
        self.root_data_dir = os.path.expanduser(rospy.get_param("data_dir"))
        self.element_of_interest = rospy.get_param("element_of_interest")
    
    def gps_odom_callback(self, data: Odometry):
        self.gps_odom_location = [data.pose.pose.position.x, data.pose.pose.position.y]
        self.gps_odom_heading = euler_from_quaternion([data.pose.pose.orientation.x, 
                                                       data.pose.pose.orientation.y,
                                                       data.pose.pose.orientation.z,
                                                       data.pose.pose.orientation.w], "sxyz")[2]

    def gps_callback(self, data:NavSatFix):
        self.gps_location[0] = data.latitude
        self.gps_location[1] = data.longitude

    def create_logging_files(self):
        self.algorithm_type = rospy.get_param(self._algorithm_type_param_name)
        self.data_dir = os.path.join(self.root_data_dir, datetime.now().strftime("%d-%m-%Y_%H:%M:%S"))
        self.data_file = os.path.join(self.data_dir, "scan_results_" + self.algorithm_type + ".csv")
        self.yaml_file = os.path.join(self.data_dir, "autonomy_params.yaml")
        
        # Create directories
        if not os.path.exists(self.data_dir):
            rospy.loginfo(f" |Creating directory {self.data_dir}")
            os.makedirs(self.data_dir)
        
        # Create empty scan results files
        for algo_type in VALID_ALGOS:
            data_file = os.path.join(self.data_dir, "scan_results_" + algo_type + ".csv")
            if not os.path.exists(data_file):
                rospy.loginfo(f" |Creating file {data_file}")
                with open(data_file, 'w') as fp:
                    pass
    
    def rotate_log_files_callback(self, data):
        self.create_logging_files()
        return True
    
    def autonomy_params_data_callback(self, data: AutonomyParams):
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
        
        self.create_logging_files()
        
        with open(self.yaml_file, "w") as file:
            yaml.dump(data_info, file)
        rospy.loginfo(f"Saved Params into {self.yaml_file}")
        
        return True

    def pxrf_data_callback(self, data: PxrfMsg):
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
        
        # Get algorithm type before saving
        self.algorithm_type = rospy.get_param(self._algorithm_type_param_name)
        self.data_file = os.path.join(self.data_dir, "scan_results_" + self.algorithm_type + ".csv")
        
        # save scan results
        with open(self.data_file, "a+") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerow(element)
            writer.writerow(concentration)
            writer.writerow(error)

        # Publish results to manager and GUI
        try:
            i = element.index(self.element_of_interest)
            completed_scan_data_msg = CompletedScanData()
            completed_scan_data_msg.status = True
            completed_scan_data_msg.element = self.element_of_interest
            completed_scan_data_msg.mean = concentration[i]
            completed_scan_data_msg.error = error[i]
            completed_scan_data_msg.file_name = self.data_file
            self.scan_recorded_to_disk_pub.publish(completed_scan_data_msg)
        except:
            rospy.logwarn(f'{self.element_of_interest} is not in scan!')

if __name__ == "__main__":
    DataRecorder()
