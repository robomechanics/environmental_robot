#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

if __name__ == '__main__':
    # start node
    rospy.init_node('skeleton', anonymous=True)

    # setting rake torque
    dig_torque_publisher = rospy.Publisher('/dig_torque',Float64, queue_size=1)
    msg = Float64()
    msg.data = -10
    dig_torque_publisher.publish(msg)
    
    # lowering rake
    lowerRake = rospy.ServiceProxy('/deploy_tool',SetBool)
    
    drive_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    msg = Twist()
    rate = rospy.Rate(10) 
    for i in range(5):
        lowerRake(True)
        rospy.sleep(1)
        msg.linear.x = -1
        drive_publisher.publish(msg)
        rate.sleep()
        lowerRake(False)
        rospy.sleep(1)
        msg.linear.x = 1
        drive_publisher.publish(msg)        

    #commanding drive, backward
    drive_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    msg = Twist()
    msg.linear.x = -1
    rate = rospy.Rate(10) # set rate to 10hz
    for i in range(10):
        drive_publisher.publish(msg)
        rate.sleep()
    drive_publisher.publish(Twist())
    
    # lower pXRF
    lowerPXRF = rospy.ServiceProxy('/deploy_sensor',SetBool)
    for i in range(1):
        lowerPXRF(True)
        rospy.sleep(2)
        lowerPXRF(False)
        rospy.sleep(2)
            
    # commanding drive, forward
    drive_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    msg = Twist()
    msg.linear.x = 1
    rate = rospy.Rate(10) # set rate to 10hz
    for i in range(10):
        drive_publisher.publish(msg)
        rate.sleep()
    drive_publisher.publish(Twist())


