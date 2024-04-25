#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class PatrickTeleop:

    def __init__(self):
        rospy.init_node("patrick_teleop", anonymous=True)
        self.twist_msg = Twist()
        self.joy_results = Joy()
        self.joy_axis = None
        self.joy_buttons = None
        self.rate = rospy.Rate(10)

        self.load_ros_params()

        self.joy_subscriber = rospy.Subscriber(self._joy_topic, Joy, self.joy_callback)
        self.twist_publisher = rospy.Publisher(
            self._cmd_managed_topic, Twist, queue_size=0
        )
        self.estop_enable_publisher = rospy.Publisher(
            self._estop_enable_topic, Bool, queue_size=0
        )
        self.estop_reset_publisher = rospy.Publisher(
            self._estop_reset_topic, Bool, queue_size=0
        )

        self.drive_factor = 1.0
        self.turn_factor = 3.0

        self.pub_twist_cmd = False

        while not rospy.is_shutdown():
            self.twist_msg_constructor()
            if self.pub_twist_cmd:
                self.twist_publisher.publish(self.twist_msg)
            self.rate.sleep()

    def load_ros_params(self):
        # Load topic names into params
        self._joy_topic = rospy.get_param("joy_topic")
        self._cmd_managed_topic = rospy.get_param("cmd_vel_managed_topic")
        self._estop_enable_topic = rospy.get_param("estop_enable_topic")
        self._estop_reset_topic = rospy.get_param("estop_reset_topic")

        rospy.loginfo("Subscribing to: %s", self._joy_topic)
        rospy.loginfo("Publishing  to: %s", self._cmd_managed_topic)

    def twist_msg_constructor(self):
        if self.joy_axis:
            self.twist_msg.linear.x = (self.joy_axis[1]) * self.drive_factor
            self.twist_msg.angular.z = (self.joy_axis[3]) * self.turn_factor
        if self.joy_buttons:
            # Soft E-stop Enable
            if self.joy_buttons[8]:
                rospy.loginfo("E-STOP Enabled")
                self.estop_enable_publisher.publish(True)
            # Soft E-stop Reset
            if self.joy_buttons[7]:
                rospy.loginfo("E-STOP Reset")
                self.estop_reset_publisher.publish(True)
            
            if self.joy_buttons[3]:
                self.drive_factor = min(3, self.drive_factor + 0.5)
                rospy.loginfo("Drive Factor: %s", self.drive_factor)
            elif self.joy_buttons[0]:
                self.drive_factor = max(0, self.drive_factor - 0.5)
                rospy.loginfo("Drive Factor: %s", self.drive_factor)
            elif self.joy_buttons[1]:
                self.turn_factor = min(6, self.turn_factor + 0.5)
                rospy.loginfo("Turn Factor: %s", self.turn_factor)
            elif self.joy_buttons[2]:
                self.turn_factor = max(4, self.turn_factor - 0.5)
                rospy.loginfo("Turn Factor: %s", self.turn_factor)

            if self.joy_buttons[5]:
                self.pub_twist_cmd = True
            else:
                self.pub_twist_cmd = False

    def joy_callback(self, data):
        self.joy_results = data
        self.joy_axis = self.joy_results.axes
        self.joy_buttons = self.joy_results.buttons


if __name__ == "__main__":
    PatrickTeleop()
