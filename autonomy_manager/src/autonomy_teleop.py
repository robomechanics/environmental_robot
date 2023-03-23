#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time

class joyProcessor(object):
    def __init__(self):
        rospy.Subscriber("/joy", Joy, self.joyCallback)
        self.joytimeout = 0.25
        self.lastJoyTime = rospy.get_time()
        self.joyThrottleScale = 0.4
        self.joySteerScale = 1
        self.manualOverride = True # whether to use joystick instead of autonomy
        self.joyCommand = [0,0] # throttle, steer
        self.joyInit = [False,False] # has triggers been initialized

    def statusCheck(self):
        # first check if joystick has timed out
        if rospy.get_time() - self.lastJoyTime > self.joytimeout:
            # if joy timed out, set controls to 0 and set manual override to stop robot
            self.manualOverride = True
            self.joyCommand = [0,0]
            self.joyInit = [False,False]

    def joyCallback(self,data):
        self.lastJoyTime = rospy.get_time()
        if data.axes[2]!=0:
            self.joyInit[0] = True
        if data.axes[5]!=0:
            self.joyInit[1] = True
	# convert throttle message
        throttle = 0
        if self.joyInit[0]:
            throttle += (data.axes[2]-1.0)/2.0
        if self.joyInit[1]:
            throttle += (1.0-data.axes[5])/2.0
        steering = data.axes[3]
        self.joyCommand = [throttle*self.joyThrottleScale,
                            steering*self.joySteerScale]
        self.manualOverride = data.buttons[5] == 1

class autonomy_teleop(object):
    def __init__(self):
        rospy.init_node('autonomy_teleop', anonymous=True)
        self.joy = joyProcessor()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.joy.statusCheck()
            #print(self.joy.joyCommand, self.joy.manualOverride)
            print('hi')
            rate.sleep()

if __name__ == '__main__':
    autonomy_teleop()
