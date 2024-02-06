#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class PatrickTeleop:

    def __init__(self):
        rospy.init_node('patrick_teleop', anonymous=True)
        self.twistMsg = Twist()
        self.joyResults = Joy()
        self.joyAxis = None
        self.joyButtons = None
        self.rate = rospy.Rate(10)
        
        self.load_ros_params()
      
        self.joySubscriber = rospy.Subscriber(self._joy_topic,Joy,self.joyCallback)
        self.twistPublisher = rospy.Publisher(self._cmd_managed_topic, Twist, queue_size=0)

        self.drive_factor = 1.0
        self.turn_factor = 3.0

        while not rospy.is_shutdown():
            self.twistMsgConstructor()
            self.twistPublisher.publish(self.twistMsg)
            self.rate.sleep()
    
    def load_ros_params(self):
        # Load topic names into params
        self._joy_topic = rospy.get_param('joy_topic')
        self._cmd_managed_topic = rospy.get_param('cmd_vel_managed_topic')
        
        print ("Subscribing to: ", self._joy_topic)
        print ("Publishing  to: ", self._cmd_managed_topic)
        
    def twistMsgConstructor(self):
        if self.joyAxis:
            self.twistMsg.linear.x = (1-self.joyAxis[2])*self.drive_factor
            self.twistMsg.angular.z = (1-self.joyAxis[5])*self.turn_factor
        if self.joyButtons:
            if self.joyButtons[3]:
                self.drive_factor += 0.5
                print("Drive Factor: ", self.drive_factor)
            elif self.joyButtons[0]:
                self.drive_factor -= 0.5
                print("Drive Factor: ", self.drive_factor)
            elif self.joyButtons[1]:
                self.turn_factor += 0.5
                print("Turn Factor: ", self.turn_factor)
            elif self.joyButtons[2]:
                self.turn_factor -= 0.5
                print("Turn Factor: ", self.turn_factor)

    def joyCallback(self,data):
        self.joyResults = data
        self.joyAxis = self.joyResults.axes
        self.joyButtons = self.joyResults.buttons

if __name__ == '__main__':
    PatrickTeleop()