#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import numpy as np
from std_srvs.srv import SetBool
from std_msgs.msg import Float64

class autonomy_teleop(object):
    def __init__(self):
        rospy.init_node('autonomy_teleop', anonymous=True)
        
        self.commandTimeout = 0.25
        self.lastCommandTime = rospy.get_time()
        
        rospy.Service('/deploy_sensor_auto', SetBool, self.deploy_sensor_auto)
        self.lowerPXRF = rospy.ServiceProxy('/deploy_sensor',SetBool)
        rospy.Service('/deploy_tool_auto', SetBool, self.deploy_tool_auto)
        self.lowerRake = rospy.ServiceProxy('/deploy_tool',SetBool)

        rospy.Service('/deploy_home_auto', SetBool, self.deploy_home_auto)
        self.homeRake = rospy.ServiceProxy('/deploy_home',SetBool)

        self.dig_torque = -2.5
        self.dig_torque_pub = rospy.Publisher('/dig_torque',Float64, queue_size=10,latch=True)
        
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
        origSteer = driveCommand[1]
        self.lastCommandTime = rospy.get_time()
        if driveCommand[0]!=0 and np.abs(driveCommand[1]*0.235/driveCommand[0]) > 0.25:
            driveCommand = (0,driveCommand[1])
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
            if data.axes[7]>0.5:
                self.lowerPXRF(False)
            elif data.axes[7]<-0.5:
                self.lowerPXRF(True)
            if data.axes[6] > 0.5:
                msg = Float64()
                msg.data = self.dig_torque
                self.dig_torque_pub.publish(msg)
                self.lowerRake(False)
            elif data.axes[6] < -0.5:
                msg = Float64()
                msg.data = self.dig_torque
                self.dig_torque_pub.publish(msg)
                self.lowerRake(True)
            if data.buttons[4] > 0.5: # Added this in -Ian
                self.homeRake(True)
    def deploy_sensor_auto(self,data):
        if not self.manualOverride and rospy.get_time()-self.lastJoyTime < self.joyTimeout:
            self.lowerPXRF(data.data)
            return True,''
        return False,''
    def deploy_tool_auto(self,data):
        if not self.manualOverride and rospy.get_time()-self.lastJoyTime < self.joyTimeout:
            self.lowerRake(data.data)
            return True,''
        return False,''
    def deploy_home_auto(self,data):
        if not self.manualOverride and rospy.get_time()-self.lastJoyTime < self.joyTimeout:
            self.homeRake(data.data)
            return True,''
        return False,''

if __name__ == '__main__':
    autonomy_teleop()
