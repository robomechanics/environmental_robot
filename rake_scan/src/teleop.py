#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from sensor_msgs.msg import Joy

class rake_scan_teleop(object):
    def __init__(self):
        # start node
        rospy.init_node('rake_scan_teleop', anonymous=True)

        # set up joystick subscriber
        rospy.Subscriber("/joy", Joy, self.joyCallback)
        self.joytimeout = 0.5
        self.lastJoyTime = rospy.get_time()
        self.deployCommand = [False,False] # deploy rake, deploy pxrf
        self.lastDeployCommand = [False,False]

        # set up dig torque publisher
        dig_torque_publisher = rospy.Publisher('/dig_torque',Float64, queue_size=10)
        dig_torque_message = Float64()
        dig_torque_message.data = -10

        # set up rake controller
        lowerRake = rospy.ServiceProxy('/deploy_tool',SetBool)

        # set up pxrf controller
        lowerPXRF = rospy.ServiceProxy('/deploy_sensor',SetBool)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            dig_torque_publisher.publish(dig_torque_message)
            self.checkJoyValid()
            if self.deployCommand[0]!=self.lastDeployCommand[0]:
                lowerRake(self.deployCommand[0])
            if self.deployCommand[1]!=self.lastDeployCommand[1]:
                lowerPXRF(self.deployCommand[1])
            self.lastDeployCommand = self.deployCommand.copy()
            rate.sleep()

    def joyCallback(self,data):
        self.lastJoyTime = rospy.get_time()
        if data.axes[7]<-0.5:
            self.deployCommand[0] = True
        if data.axes[7]>0.5:
            self.deployCommand[0]=False
        if data.axes[1]<-0.5:
            self.deployCommand[1]=True
        if data.axes[1]>0.5:
            self.deployCommand[1]=False

    def checkJoyValid(self):
        if rospy.get_time() - self.lastJoyTime > self.joytimeout:
            self.deployCommand = [False,False]

if __name__ == '__main__':
    rake_scan_teleop()
