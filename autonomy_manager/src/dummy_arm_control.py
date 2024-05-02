#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse

def handle_dummy_service(req):
    print("Arm Lower service called")
    return SetBoolResponse(
        success=True,
        message="Dummy service response: Service called successfully!"
    )

def dummy_service_server():
    rospy.init_node('dummy_service_server')
    
    lower_arm_service_name = rospy.get_param("lower_arm_service_name")
    
    
    s = rospy.Service(lower_arm_service_name, SetBool, handle_dummy_service)
    print("Ready to respond to dummy service calls.")
    rospy.spin()

if __name__ == "__main__":
    dummy_service_server()
