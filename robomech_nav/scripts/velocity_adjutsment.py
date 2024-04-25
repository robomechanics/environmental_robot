#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Joy

class VelocityAdjust:

    def __init__(self):
        rospy.init_node('velocity_adjust', anonymous=True)
        self.twistMsg = Twist()
        self.rate = rospy.Rate(10)
        
        
        self.use_ros_params = False
        self.load_ros_params()
      
        self.joySubscriber = rospy.Subscriber(self._cmd_vel_nav_topic,Twist,self.velocityAdjust)
        self.twistPublisher = rospy.Publisher(self._cmd_vel_base_topic, Twist, queue_size=0)

        self.angularMultiplier = 6.0
        self.linearMultiplier = 2.0

        rospy.spin()
    
    def load_ros_params(self):
        # Load topic names into params
        if self.use_ros_params:
            self._cmd_vel_nav_topic = rospy.get_param('cmd_vel_nav')
            self._cmd_vel_base_topic = rospy.get_param('cmd_vel_base')
        else:
            self._cmd_vel_nav_topic = '/cmd_vel_nav'
            self._cmd_vel_base_topic = '/cmd_vel'
        
    def velocityAdjust(self,data):
        self.twistMsg = data
        self.twistMsg.linear.x = self.twistMsg.linear.x * self.linearMultiplier
        self.twistMsg.angular.z = self.twistMsg.angular.z * self.angularMultiplier

        self.twistPublisher.publish(self.twistMsg)
        print('published!')
        


if __name__ == '__main__':
    VelocityAdjust()