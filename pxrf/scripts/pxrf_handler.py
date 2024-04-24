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
    def __init__(self, data_dir, element_of_interest):
        self.data_dir = data_dir
        self.element_of_interest = element_of_interest

        rospy.init_node("pxrf_handler", anonymous=True)
        self.load_ros_params()

        self.pxrf_command = rospy.Publisher(self._pxrf_cmd_topic, String, queue_size=1)
        rospy.Subscriber(self._pxrf_response_topic, String, self.response_listener)
        rospy.Subscriber(self._pxrf_data_topic, PxrfMsg, self.data_listener)

        rospy.Service(self._start_scan_service_name, Complete, self.scan_start)
        rospy.Subscriber(self._odometry_topic, Odometry, self.odometry_callback)
        self.scanning = False
        self.location = [0, 0]
        
        rospy.loginfo("Started PXRF Handler...")
        
        rospy.spin()

    def load_ros_params(self):
        self._pxrf_cmd_topic = rospy.get_param("pxrf_cmd_topic")
        self._pxrf_response_topic = rospy.get_param("pxrf_response_topic")
        self._pxrf_data_topic = rospy.get_param("pxrf_data_topic")
        self._start_scan_service_name = rospy.get_param("start_scan_service_name")
        self._odometry_topic = rospy.get_param("odometry_topic")
        self._pxrf_complete_service_name = rospy.get_param("pxrf_complete_service_name")

    def odometry_callback(self, data):
        self.location[0] = data.pose.pose.position.y
        self.location[1] = data.pose.pose.position.x

    def scan_start(self, data):
        if data.status:
            self.scanning = True
            self.pxrf_command.publish("start")
        else:
            self.pxrf_command.publish("stop")
            self.scanning = False
        return True

    def file_check(self):
        if not os.path.exists(self.data_dir):
            os.mkdir(self.data_dir)
        if not os.path.exists(os.path.join(self.data_dir, str(date.today()) + ".csv")):
            a = 0  # placeholder for making a csv
        return None

    def data_listener(self, data):
        if self.scanning and self.test_stopped:
            self.scanning = False
            
            # get ros and system time
            self.systemTime = time.localtime()
            self.rosTime = rospy.Time.now()
            
            # record and process received response
            element, concentration, error = chemistryParser(data.chemistry)
            header = [
                data.dailyId,
                data.testId,
                data.testDateTime,
                self.location,
                self.systemTime,
                self.rosTime,
            ]
            self.file_check()
            with open(
                os.path.join(self.data_dir, str(date.today()) + ".csv"), "a+"
            ) as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerow(element)
                writer.writerow(concentration)
                writer.writerow(error)

            # return result to manager
            if not self._pxrf_complete_service_name in rosservice.get_service_list():
                return
            scan_result_proxy = rospy.ServiceProxy(
                self._pxrf_complete_service_name, ScanData
            )
            i = element.index(self.element_of_interest)
            scan_result_proxy(
                True, self.element_of_interest, concentration[i], error[i]
            )

    def response_listener(self, data):
        rospy.loginfo("Test complete")
        self.test_stopped = (data.data == "201")


if __name__ == "__main__":
    data_dir = os.path.expanduser("~/pxrf_results")
    # if os.path.isdir(data_dir):
    #     os.mkdir(data_dir)
    element_of_interest = "Cl"
    PXRFHandler(data_dir, element_of_interest)
