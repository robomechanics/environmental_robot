#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from sensor_msgs.msg import Joy

class camera_teleop(object):
    def __init__(self):
        # start node
        rospy.init_node('camera_teleop', anonymous=True)

        # set up joystick subscriber
        rospy.Subscriber("/joy", Joy, self.joyCallback)
        self.joytimeout = 0.5
        self.lastJoyTime = rospy.get_time()
        self.TwistZ = 0.0
        self.TwistX = 0.0
        self.printValues = True
        self.publishTwist = True
        self.TwistMsg = Twist()
        self.TwistFactor = 1.0
        self.twistPublisher = rospy.Publisher('pan_tilt_vel',Twist,queue_size=10)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.checkJoyValid()
            print("Loop:")

            if self.printValues == True:
                print("[",self.TwistX," 0.0 ",self.TwistZ,"]")
                if self.TwistZ > 0.0:
                    print("Pan Left")
                elif self.TwistZ < 0.0:
                    print("Pan right")
                else:
                    print("Pan steady")
                if self.TwistX > 0.0:
                    print("Tilt up")
                elif self.TwistX < 0.0:
                    print("Tilt down")
                else:
                    print("Tilt steady")

            if self.publishTwist == True:
                self.Twist = Twist()
                self.Twist.angular.x = self.TwistX*self.TwistFactor
                self.Twist.angular.z = self.TwistZ*self.TwistFactor
                self.twistPublisher.publish(self.Twist)
                if self.printValues == True:
                    print("Publishing to topic!")
                    print(self.Twist)

            else:
                print("Not publishing!")

            rate.sleep()

    def joyCallback(self,data):
        self.lastJoyTime = rospy.get_time()
        if data.axes[0] != 0.0 or data.axes[1] != 0.0:
            if data.axes[0]<-0.5:
                self.TwistZ = data.axes[0]
            if data.axes[0]>0.5:
                self.TwistZ = data.axes[0]
            if data.axes[1]<-0.5:
                self.TwistX = data.axes[1]
            if data.axes[1]>0.5:
                self.TwistX = data.axes[1]
        else:
            self.TwistX = 0.0
            self.TwistZ = 0.0

    def checkJoyValid(self):
        if rospy.get_time() - self.lastJoyTime > self.joytimeout:
            self.TwistX = 0.0
            self.TwistZ = 0.0

if __name__ == '__main__':
    camera_teleop()
