#!/usr/bin/env python3
import rospy
import rosservice
from pxrf.msg import PxrfMsg, CompletedScanData
from std_msgs.msg import String
from autonomy_manager.srv import Complete, ScanData
import os
from sensor_msgs.msg import NavSatFix
from datetime import date
import csv
import time


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
        
        self.data_file = os.path.join(self.data_dir, str(date.today()) + ".csv")

        self.pxrf_command_pub = rospy.Publisher(self._pxrf_cmd_topic, String, queue_size=1)
        self.scan_completed_pub = rospy.Publisher(self._scan_completed_topic, CompletedScanData, queue_size=1)
        
        self._pxrf_response_sub = rospy.Subscriber(self._pxrf_response_topic, String, self.response_listener)
        self._pxrf_data_sub = rospy.Subscriber(self._pxrf_data_topic, PxrfMsg, self.data_listener)
        self._start_scan_sub = rospy.Service(self._start_scan_service_name, Complete, self.scan_start)
        self._gps_sub = rospy.Subscriber(self._gps_topic, NavSatFix, self.gps_callback)
        
        self.scanning = False
        self.gps_location = [0, 0]
        
        rospy.loginfo("Started PXRF Handler...")
        
        rospy.spin()

    def load_ros_params(self):
        self._pxrf_cmd_topic = rospy.get_param("pxrf_cmd_topic")
        self._pxrf_response_topic = rospy.get_param("pxrf_response_topic")
        self._pxrf_data_topic = rospy.get_param("pxrf_data_topic")
        self._gps_topic = rospy.get_param("gq7_ekf_llh_topic")
        self._scan_completed_topic = rospy.get_param("scan_completed_topic")
        self._start_scan_service_name = rospy.get_param("start_scan_service_name")
        
        self.data_dir = rospy.get_param("data_dir")
        self.element_of_interest = rospy.get_param("element_of_interest")

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

    def file_check(self):
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)

    def data_listener(self, data: PxrfMsg):
        if self.scanning and self.test_stopped:
            self.scanning = False
            
            # get ros and system time
            self.system_time = time.localtime()
            self.ros_time = rospy.Time.now()
            
            # record and process received response
            element, concentration, error = chemistry_parser(data.chemistry)
            header = [
                data.dailyId,
                data.testId,
                data.testDateTime,
                self.gps_location,
                self.system_time,
                self.ros_time,
            ]
            
            self.file_check()
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
