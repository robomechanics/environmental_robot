#!/usr/bin/env python
import rospy
from autonomy_manager.srv import Complete

def handle_dummy_service(req):
    print("Scan service called")
    return True

def dummy_service_server():
    rospy.init_node('dummy_service_server')
    
    start_scan_service_name = rospy.get_param("start_scan_service_name")
    
    
    s = rospy.Service(start_scan_service_name, Complete, handle_dummy_service)
    rospy.loginfo("Ready to respond to dummy scan service calls.")
    rospy.spin()

if __name__ == "__main__":
    dummy_service_server()
