#!/usr/bin/env python3
import rospy
from autonomy_manager.srv import DeployAutonomy,NavigateGPS, RunSensorPrep
from std_srvs.srv import SetBool
class dummy_search(object):
    def __init__(self,boundary):
        self.boundary = boundary
    def next(self):
        return [0,0]
    def update(self,data):
        pass

class dummy_services(object):
    def __init__(self):
        rospy.init_node('dummy_services', anonymous=True)
        rospy.Service('/run_sensor_prep', RunSensorPrep,self.dummy)
        rospy.Service('/next_goal_nav', NavigateGPS,self.dummy)
        rospy.Service('/deploy_sensor', SetBool, self.dummy2)
        rospy.spin()

    def dummy(self,data):
        #print(data)
        return True

    def dummy2(self,data):
        #print(data)
        return (True, "")
if __name__ == '__main__':
    dummy_services()
