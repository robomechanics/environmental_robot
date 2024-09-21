#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse

t = None
cmd_vel = None

def handle_publish_cmd_vel(data):
    # Create a Twist message
    twist = Twist()
    twist.linear.x = cmd_vel["linear"]["x"]
    twist.linear.y = cmd_vel["linear"]["y"]
    twist.linear.z = cmd_vel["linear"]["z"]
    twist.angular.x = cmd_vel["angular"]["x"]
    twist.angular.y = cmd_vel["angular"]["y"]
    twist.angular.z = cmd_vel["angular"]["z"]

    # Publish the message for t seconds
    rospy.loginfo(f'Publishing cmd_vel={twist.linear.x, twist.angular.x} for t={t} secs')
    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration(t):
        pub.publish(twist)
        rate.sleep()
    
    return EmptyResponse()

if __name__ == "__main__":
    try:
        rospy.init_node('publish_cmd_vel_server')
        
        # Create a publisher
        cmd_vel_topic = rospy.get_param("cmd_vel_topic", "/cmd_vel")
        pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    
        t = rospy.get_param("/constant_velocity/time", 2)
        cmd_vel = rospy.get_param(
            "/constant_velocity/cmd_vel",
            {
                "linear": {"x": -0.2, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
        )

        constant_velocity_commander_service_name = rospy.get_param("constant_velocity_commander_service_name")
        
        s = rospy.Service(constant_velocity_commander_service_name, Empty, handle_publish_cmd_vel)
        rospy.loginfo("Ready to publish cmd_vel...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
