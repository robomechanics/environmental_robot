#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Quaternion, Twist, TwistStamped, PoseStamped, PointStamped, Vector3
from sensor_msgs.msg import Imu, NavSatFix
import tf
from std_msgs.msg import Float64MultiArray
from math import *
from scipy.spatial.transform import Rotation as R
import numpy as np
import utm

class gps_navigation():
    
    def __init__(self):
        self.goal = None
        self.pos = None
        self.kp = 1
        self.mapOrigin = None
        self.pathMessage = Path()
        rospy.init_node('gps_navigation',anonymous=True)
        self.mapOrigin_sub = rospy.Subscriber('/gps/fix',NavSatFix,self.readMapOrigin)
        self.odom_sub = rospy.Subscriber('/gx5/nav/odom',Odometry,self.readOdom)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.setGoal)
        self.goal_pub = rospy.Publisher('/gps_navigation/goal',PointStamped,queue_size=5)
        self.cmd_pub = rospy.Publisher('/cmd_vel/managed',Twist,queue_size=5) # what queue_size to use?
        self.robotPose_pub = rospy.Publisher('/gps_navigation/robotPose',PoseStamped,queue_size=5)
        self.robotTF_broadcaster = tf.TransformBroadcaster()
        self.nextPathTime = None
        self.robotPath_pub = rospy.Publisher('gps_navigation/robotPath',Path,queue_size=5)
        rospy.spin()

    def readMapOrigin(self,data):
        mapLat = data.latitude
        mapLon = data.longitude
        mapx,mapy,_,_ = utm.from_latlon(mapLat,mapLon)
        self.mapOrigin = np.array([mapx,mapy])

    def readOdom(self,data):
        if self.mapOrigin is None: # don't do anything unless know map origin
            return
        # get robot's current position (relative to map)
        lat = data.pose.pose.position.y
        lon = data.pose.pose.position.x
        x,y,_,_ = utm.from_latlon(lat,lon)
        self.pos = np.array([x,y]) - self.mapOrigin
        quat = data.pose.pose.orientation
        # convert quaternion to heading angle
        r = R.from_quat([quat.x,quat.y,quat.z,quat.w])
        xVec = r.as_dcm()[:,0]
        self.heading = np.mod(atan2(xVec[1],xVec[0])+np.pi/3.0,2*np.pi)
        # publish robot pose for map
        robotPoseMessage = PoseStamped()
        robotPoseMessage.header = data.header
        robotPoseMessage.header.frame_id='map'
        robotPoseMessage.pose.position.x=self.pos[0]
        robotPoseMessage.pose.position.y=self.pos[1]
        quat = r.from_euler('z',np.pi/2.0+self.heading).as_quat()
        robotPoseMessage.pose.orientation.x=quat[0]
        robotPoseMessage.pose.orientation.y=quat[1]
        robotPoseMessage.pose.orientation.z=quat[2]
        robotPoseMessage.pose.orientation.w=quat[3]
        self.robotPose_pub.publish(robotPoseMessage)
        self.robotTF_broadcaster.sendTransform((self.pos[0],self.pos[1],0),quat,data.header.stamp,'base_link','map')
        testPoint = PointStamped()
        testPoint.header.frame_id='base_link'
        # publish robot path for map
        if self.nextPathTime is None:
            self.nextPathTime = data.header.stamp.secs
        if data.header.stamp.secs >= self.nextPathTime:
            while data.header.stamp.secs >= self.nextPathTime:
                self.nextPathTime += 1
            self.pathMessage.header = data.header
            self.pathMessage.poses.append(robotPoseMessage)
            self.robotPath_pub.publish(self.pathMessage)
        # run navigation
        self.navigate()

    def setGoal(self,data):
        self.goal = np.array([data.pose.position.x,data.pose.position.y])
        self.lastGoalMessage = PointStamped()
        self.lastGoalMessage.header = data.header
        self.lastGoalMessage.header.frame_id = 'map'
        self.lastGoalMessage.point.x = self.goal[0]
        self.lastGoalMessage.point.y = self.goal[1]
        self.goal_pub.publish(self.lastGoalMessage)

    def navigate(self):
        if self.goal is None:
            return
        relGoal = self.goal-self.pos
        print(relGoal)
        if np.linalg.norm(relGoal)<1.0:
            self.lastGoalMessage.point.z = 10
            self.goal_pub.publish(self.lastGoalMessage)
            self.goal=None
            print('done')
        goalAngle = np.mod(atan2(-relGoal[0],relGoal[1]),2*np.pi)
        print('---')
        print(goalAngle*180/np.pi)
        print(self.heading*180/np.pi)

        yaw_diff = np.mod(goalAngle - self.heading,2*np.pi)
        if yaw_diff > np.pi:
            yaw_diff = yaw_diff-2*np.pi
        print("Yaw Difference: %.2f radians" %(yaw_diff))
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.5
        cmd_msg.angular.z = self.kp*yaw_diff
        self.cmd_pub.publish(cmd_msg)

if __name__ == '__main__':
    gps_navigation()