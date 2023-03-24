#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import numpy as np

class joyProcessor(object):
    def __init__(self):
        rospy.Subscriber("/joy", Joy, self.joyCallback)
        self.joytimeout = 0.25
        self.lastJoyTime = rospy.get_time()
        self.joyThrottleScale = 0.3
        self.joySteerScale = 0.3/0.235
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
        
        self.autonomyTimeout = 0.25
        self.lastAutoCommandTime = rospy.get_time()
        self.autonomyCommand = [0,0]
        rospy.Subscriber('/cmd_vel_auto',Twist,self.autoDriveCallback)
        
        self.drivePub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.maxMotorMag = 0.3

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.joy.statusCheck()
            if self.joy.manualOverride:
                self.managedDrive(self.joy.joyCommand)
            elif rospy.get_time() - self.lastAutoCommandTime > self.autonomyTimeout:
                self.managedDrive(self.autonomyCommand)
            else:
                self.managedDrive([0,0])
            rate.sleep()
    def managedDrive(self,driveCommand):
        motorMag = np.abs(driveCommand[0]) + np.abs(driveCommand[1])*0.235
        if motorMag > self.maxMotorMag:
            driveCommand = (driveCommand[0]*self.maxMotorMag/motorMag,driveCommand[1]*self.maxMotorMag/motorMag)
        if motorMag < 0.01:
            driveCommand = (0,0)
        pubCommand = Twist()
        pubCommand.linear.x = driveCommand[0]
        pubCommand.angular.z = driveCommand[1]
        self.drivePub.publish(pubCommand)
    def autoDriveCallback(self,data):
        self.lastAutoCommandTime = rospy.get_time()
        self.autonomyCommand = [data.linear.x,data.angular.z]

if __name__ == '__main__':
    autonomy_teleop()
