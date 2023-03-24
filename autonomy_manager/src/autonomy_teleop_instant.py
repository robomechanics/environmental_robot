#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import numpy as np

class autonomy_teleop(object):
    def __init__(self):
        rospy.init_node('autonomy_teleop', anonymous=True)
        
        self.commandTimeout = 0.25
        self.lastCommandTime = rospy.get_time()
        
        rospy.Subscriber('/cmd_vel_auto',Twist,self.autoDriveCallback)
        rospy.Subscriber("/joy", Joy, self.joyDriveCallback)
        self.joyTimeout = 0.25
        self.lastJoyTime = rospy.get_time()
        self.joyThrottleScale = 0.3
        self.joySteerScale = 0.3/0.235
        self.manualOverride = True # whether to use joystick instead of autonomy
        self.joyInit = [False,False]
        
        self.drivePub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.maxMotorMag = 0.3

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if rospy.get_time() - self.lastCommandTime > self.commandTimeout:
                self.managedDrive((0,0))
            rate.sleep()
    def managedDrive(self,driveCommand):
        self.lastCommandTime = rospy.get_time()
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
        self.lastAutoCommandTime = rospy.get_time()-self.lastJoyTime 
        autonomyCommand = (data.linear.x,data.angular.z)
        if not self.manualOverride and rospy.get_time()-self.lastJoyTime < self.joyTimeout:
            self.managedDrive(autonomyCommand)
    def joyDriveCallback(self,data):
        print('here')
        if rospy.get_time() - self.lastJoyTime > self.joyTimeout:
            self.joyInit = [False,False]
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
        joyCommand = (throttle*self.joyThrottleScale,
                        steering*self.joySteerScale)
        self.manualOverride = data.buttons[5] == 1
        if self.manualOverride:
            self.managedDrive(joyCommand)

if __name__ == '__main__':
    autonomy_teleop()
