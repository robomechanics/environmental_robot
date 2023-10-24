#!/usr/bin/env python3
import roslaunch
import rospy
import rospkg
import rostopic
import os
import itertools
from sensor_msgs.msg import Joy
import re

class camera_switcher():
    global camera_num
    
    def __init__(self):
        self.camera = []
        pubs, subs = rostopic.get_topic_list()
        for item in itertools.chain.from_iterable(pubs):
            if (isinstance(item, str)):
                if re.search("/webcam_\d+/image_raw$", item):
                    self.camera.append(item)
        self.camera_num = 1
        self.process = None
        self.total_num = len(self.camera)
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        #self.launchConfig = ['/home/cvx/catkin_ws/src/kraton/launch/control.launch']
        #self.args = None
        #self.launchFile = [roslaunch.rlutil.resolve_launch_arguments(self.launchConfig)[0]]
        self.launch = roslaunch.scriptapi.ROSLaunch()
        #self.launch.parent = roslaunch.parent.ROSLaunchParent(self.uuid, self.launchFile)
        self.launch.start()
        #self.launchNode()
        if (len(self.camera)>0):
        	self.node = roslaunch.core.Node("image_view","image_view", name = "cam", remap_args = [("image","{cam}".format(cam =str(self.camera[self.camera_num]))),("_image_transport","theora")])
        	self.process = self.launch.launch(self.node) #, ("_autosize","true")
        rospy.init_node('camera_switch',anonymous=True)
        rospy.Subscriber("/joy", Joy, self.callback)
        rospy.spin()
        
    def callback(self, data):
        left = data.buttons[4]
        right = data.buttons[5]
        if (left != 0 or right != 0):
            if self.process.is_alive():
                self.process.stop()
            #self.launch.shutdown()
            if (left == 1):
                self.camera_num = self.camera_num - 1
                if self.camera_num < 0:
                    self.camera_num = self.total_num - 1
            elif (right == 1):
                self.camera_num = (self.camera_num + 1)% (self.total_num)
            self.launch.start()
            self.node = roslaunch.core.Node("image_view","image_view", name = "camera".format(number = self.camera_num), remap_args = [("image","{cam}".format(cam =str(self.camera[self.camera_num]))), ("_image_transport","theora"),("_autosize","true")])
            #self.launchNode()
            self.process = self.launch.launch(self.node)
        #self.launchConfig = ['/home/cvx/catkin_ws/src/kraton/launch/camera.launch',"image:={cam}".format(cam =str(self.camera[self.camera_num]))] 
        #self.args = self.launchConfig[1:]
        #self.launchFile = [(roslaunch.rlutil.resolve_launch_arguments(self.launchConfig)[0],self.args)]
        #self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, self.launchFile)
        #self.launch.start()
    def launchNode(self):
        self.node = roslaunch.core.Node("image_view","image_view",
                                        name = "camera".format(number = self.camera_num),
                                        remap_args = [("image","{cam}".format(cam =str(self.camera[self.camera_num]))),
                                        ("_image_transport","theora")])


if __name__ =='__main__':
    camera_switcher()
