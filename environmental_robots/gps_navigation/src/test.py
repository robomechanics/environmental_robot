#!/usr/bin/python3
import rospy
from std_srvs.srv import SetBool

def handle_parking(req):
    # Service code here
    SetBoolResponse = SetBool()
    SetBoolResponse.success = True
    SetBoolResponse.message = "Parking brake activated"
    return [True, 'Success']

def parking_brake():
    rospy.init_node('parking_brake', anonymous=True)
    service = rospy.Service('/parking_brake', SetBool, handle_parking)
    rospy.loginfo("Ready to receive service calls")
    rospy.spin()

if __name__ == "__main__":
    parking_brake()