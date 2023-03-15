#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class teleop_drive(object):
    def __init__(self):
        rospy.init_node('teleop_drive', anonymous=True)
        drivePub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # scale parameters
        self.throttleScale = 0.4
        self.steerScale = 1
        
        # set up joystick subscriber
        rospy.Subscriber("/joy", Joy, self.joyCallback)
        self.joytimeout = 0.5
        self.lastJoyTime = rospy.get_time()
        self.joyCommand = [0,0] # throttle, steer
        self.joyInit = [False,False] # has triggers been initialized

        # publish
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.checkJoyValid()
            driveCommand = Twist()
            driveCommand.linear.x = self.joyCommand[0]*self.throttleScale
            driveCommand.angular.z = self.joyCommand[1]*self.steerScale
            drivePub.publish(driveCommand)
            rate.sleep()

    def joyCallback(self,data):
        self.lastJoyTime = rospy.get_time()
        if data.axes[2]!=0:
            self.joyInit[0] = True
        if data.axes[5]!=0:
            self.joyInit[1] = True
        throttle = 0
        if self.joyInit[0]:
            throttle += (data.axes[2]-1.0)/2.0
        if self.joyInit[1]:
            throttle += (1.0-data.axes[5])/2.0
        steering = data.axes[3]
        self.joyCommand = [throttle,steering]
    
    def checkJoyValid(self):
        if rospy.get_time() - self.lastJoyTime > self.joytimeout:
            self.joyInit= [False,False]
            self.joyCommand = [0,0]

if __name__ == '__main__':
    teleop_drive()
