#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import numpy as np
from std_srvs.srv import SetBool
from std_msgs.msg import Float64


class AutonomyTeleop(object):

    def __init__(self):
        rospy.init_node("autonomy_teleop", anonymous=True)
        rospy.sleep(0.5)
        self.load_ros_params()

        self.commandTimeout = 0.25
        self.lastCommandTime = rospy.get_time()

        rospy.Service(self._deploy_sensor_auto_service_name, SetBool,
                      self.deploy_sensor_auto)
        self.lowerPXRF = rospy.ServiceProxy(self._deploy_sensor_service_name,
                                            SetBool)

        rospy.Service(self._deploy_tool_auto_service_name, SetBool,
                      self.deploy_tool_auto)
        self.lowerRake = rospy.ServiceProxy(self._deploy_tool_service_name,
                                            SetBool)

        rospy.Service(self._deploy_home_auto_service_name, SetBool,
                      self.deploy_home_auto)
        self.homeRake = rospy.ServiceProxy(self._deploy_home_service_name,
                                           SetBool)

        self.dig_torque = -5.0
        self.dig_torque_pub = rospy.Publisher(self._dig_torque_topic,
                                              Float64,
                                              queue_size=10,
                                              latch=True)

        rospy.Subscriber(self._cmd_vel_auto_topic, Twist,
                         self.auto_drive_callback)
        rospy.Subscriber(self._joy_topic, Joy, self.joy_drive_callback)

        self.joyTimeout = 0.25
        self.lastJoyTime = rospy.get_time()
        self.joyThrottleScale = 1.0
        self.joySteerScale = 1.0 / 0.235
        self.manualOverride = True  # whether to use joystick instead of autonomy
        self.joyInit = [False, False]
        self.turnFactor = 3.5
        self.curbMotors = False

        self.drivePub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.maxMotorMag = 0.3

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if rospy.get_time() - self.lastCommandTime > self.commandTimeout:
                self.managed_drive((0, 0))
            rate.sleep()

    def load_ros_params(self):
        self._deploy_sensor_auto_service_name = rospy.get_param(
            "deploy_sensor_auto_service_name")
        self._deploy_sensor_service_name = rospy.get_param(
            "deploy_sensor_service_name")
        self._deploy_tool_auto_service_name = rospy.get_param(
            "deploy_tool_auto_service_name")
        self._deploy_tool_service_name = rospy.get_param(
            "deploy_tool_service_name")
        self._deploy_home_auto_service_name = rospy.get_param(
            "deploy_home_auto_service_name")
        self._deploy_home_service_name = rospy.get_param(
            "deploy_home_service_name")
        self._dig_torque_topic = rospy.get_param("dig_torque_topic")
        self._cmd_vel_auto_topic = rospy.get_param("cmd_vel_auto_topic")
        self._cmd_vel_topic = rospy.get_param("cmd_vel_topic")
        self._joy_topic = rospy.get_param("joy_topic")

    def managed_drive(self, driveCommand):
        origSteer = driveCommand[1]
        self.lastCommandTime = rospy.get_time()
        # if driveCommand[0]!=0 and np.abs(driveCommand[1]*0.235/driveCommand[0]) > 0.25:
        #     driveCommand = (0,driveCommand[1])
        # motorMag = np.abs(driveCommand[0]) + np.abs(driveCommand[1])*0.235
        # if motorMag > self.maxMotorMag:
        #     driveCommand = (driveCommand[0]*self.maxMotorMag/motorMag,driveCommand[1]*self.maxMotorMag/motorMag)
        # if motorMag < 0.01:
        #     driveCommand = (0,0)
        if not self.curbMotors:
            driveCommand = (driveCommand[0], driveCommand[1])

        pubCommand = Twist()
        pubCommand.linear.x = driveCommand[0]
        pubCommand.angular.z = driveCommand[1]
        self.drivePub.publish(pubCommand)

    def auto_drive_callback(self, data):
        self.lastAutoCommandTime = rospy.get_time() - self.lastJoyTime
        autonomyCommand = (data.linear.x, data.angular.z * self.turnFactor)
        if (not self.manualOverride
                and rospy.get_time() - self.lastJoyTime < self.joyTimeout):
            self.managed_drive(autonomyCommand)

    def joy_drive_callback(self, data):
        if rospy.get_time() - self.lastJoyTime > self.joyTimeout:
            self.joyInit = [False, False]
        self.lastJoyTime = rospy.get_time()
        if data.axes[2] != 0:
            self.joyInit[0] = True
        if data.axes[5] != 0:
            self.joyInit[1] = True
        # convert throttle message
        throttle = 0
        if self.joyInit[0]:
            throttle += (data.axes[2] - 1.0) / 2.0
        if self.joyInit[1]:
            throttle += (1.0 - data.axes[5]) / 2.0
        steering = data.axes[3]
        joyCommand = (throttle * self.joyThrottleScale,
                      steering * self.joySteerScale)
        self.manualOverride = data.buttons[5] == 1
        if self.manualOverride:
            self.managed_drive(joyCommand)
            if data.axes[7] > 0.5:
                self.lowerPXRF(False)
            elif data.axes[7] < -0.5:
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
            if data.buttons[4] > 0.5:  # Added this in -Ian
                self.homeRake(True)

    def deploy_sensor_auto(self, data):
        if (not self.manualOverride
                and rospy.get_time() - self.lastJoyTime < self.joyTimeout):
            self.lowerPXRF(data.data)
            return True, ""
        return False, ""

    def deploy_tool_auto(self, data):
        if (not self.manualOverride
                and rospy.get_time() - self.lastJoyTime < self.joyTimeout):
            self.lowerRake(data.data)
            return True, ""
        return False, ""

    def deploy_home_auto(self, data):
        if (not self.manualOverride
                and rospy.get_time() - self.lastJoyTime < self.joyTimeout):
            self.homeRake(data.data)
            return True, ""
        return False, ""


if __name__ == "__main__":
    AutonomyTeleop()
