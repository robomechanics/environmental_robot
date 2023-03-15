#!/usr/bin/env python3
import rospy
from autonomy_manager.srv import DeployAutonomy,NavigateGPS

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
        rospy.Service('/NavigateToGPS', NavigateGPS,self.navigate)
        rospy.spin()

    def navigate(self,data):
        print(data)
        return True

if __name__ == '__main__':
    dummy_services()
