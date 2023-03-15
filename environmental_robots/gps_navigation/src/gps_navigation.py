#!/usr/bin/python3
import rospy
from microstrain_inertial_msgs.msg import FilterHeading
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Quaternion, Twist, TwistStamped, PoseStamped, PointStamped, Vector3
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64MultiArray,Bool
from math import *
from scipy.spatial.transform import Rotation as R
from scipy import signal, linalg
import numpy as np
from sklearn import preprocessing

#constrain the angle to be within pi
def wrapTopi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

#Main function to control the robot using a PID controller
class gps_navigation():
    
    def __init__(self):
        #init parameters needed for waypoint nav
        rospy.init_node('gps_navigation', anonymous=True)
        self.X = 0
        self.Y = 0
        self.xdot = 0
        self.ydot = 0
        self.vdot = 0
        self.psi = 0
        self.psidot = 0
        self.t_prev = 0 
        self.t_curr = 0
        self.dt = 0
        self.prev = rospy.get_rostime()
        
        #init controller
        self.controller = PIDController()

        self.goalPos = None # in lon lat degree
        self.odom_sub = rospy.Subscriber('/nav/odom', Odometry, self.navigate) # do navigation when odom is published
        self.goal_sub = rospy.Subscriber('/gps_navigation/goal', PoseStamped, self.setGoal) # for setting goal
        self.cmd_pub = rospy.Publisher('/cmd_vel/managed', Twist, queue_size=5) # for publishing drive commands
        #need a subscriber to find the yaw
        self.goal_pub = rospy.Publisher('/gps_navigation/current_goal',PoseStamped,queue_size=5) # for publishing status of navigation (running/done)
        self.heading_sub = rospy.Subscriber('nav/heading', FilterHeading, self.set_heading)
       # publish status of navigation
        rate = rospy.Rate(10)
        self.lastmsg = PoseStamped()

        while not rospy.is_shutdown():
            msg = PoseStamped()
            # if navigation is off
            if self.goalPos is None:
                msg.pose.position.x = self.lastmsg.pose.position.x
                msg.pose.position.y = self.lastmsg.pose.position.y
                msg.pose.position.z = -1
            # if navigation is on
            else:
                msg.pose.position.x = self.goalPos[0]
                msg.pose.position.y = self.goalPos[1]
                msg.pose.position.z = 1
            self.goal_pub.publish(msg)
            self.lastmsg = msg
            rate.sleep()
    #set the heading of the robot
    def set_heading(self, data):
        if(data.heading_rad == 0 and data.heading_deg == 0):
            return
        self.psi = data.heading_rad

    #set the goal of x and y
    def setGoal(self,data):
        if np.isnan(data.pose.position.x) or np.isnan(data.pose.position.y):
            self.lastmsg.pose.position.x = float('nan')
            self.lastmsg.pose.position.y = float('nan')
        if data.pose.position.z < 0:
            self.goalPos = None
        else:
            self.goalPos = np.array([data.pose.position.y,data.pose.position.x])
    #main function to compute the outputs and drive the robot
    def navigate(self,data):

        if (data.pose.pose.position.x != 0 or data.pose.pose.position.y != 0):   
            self.X = data.pose.pose.position.x
            self.Y = data.pose.pose.position.y
            self.xdot = data.twist.twist.linear.x
            self.ydot = data.twist.twist.linear.y
            self.vdot = sqrt(abs(self.xdot)**2 + abs(self.ydot)**2)
            self.psidot = data.twist.twist.angular.z
        if self.goalPos is None: # don't need to do anything if no goal set
            cmd_msg_reset = Twist()
            self.cmd_pub.publish(cmd_msg_reset)
            return

        # calculate robot position and heading
        robotPos = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        #quat = data.pose.pose.orientation
        #r = R.from_quat([quat.x,quat.y,quat.z,quat.w])
        #xVec = r.as_dcm()[:,0]
        #robotHeading = np.mod(atan2(xVec[1],xVec[0])+np.pi/3.0,2*np.pi)
        #self.psi = robotHeading
        # run navigation
        relGoal = self.goalPos - robotPos
        print('----Navigating----')
        #print('relative goal: '+ str(relGoal))
        #relGoal[1] = relGoal[1]*np.cos(robotPos[0]*np.pi/180)
        #print('relative goal scaled: ' +str(relGoal))
        
        #define tolerance
        if np.linalg.norm(relGoal)<1e-5:
            self.goalPos=None
            print('----Finished Navigation----')
        #goalAngle = np.mod(atan2(relGoal[1],relGoal[0]),2*np.pi)
        #print('relative goal angle: %.2f degrees' %(goalAngle*180/np.pi))
        print(f'robot heading angle: {self.psi*180/np.pi:.2f} degrees')
        current_time = rospy.get_rostime()
        self.dt = (current_time - self.prev).to_sec()
        angle, speed = self.controller.update(self.goalPos,
                                              self.psi,
                                              self.dt,
                                              self.X,
                                              self.Y,
                                              self.xdot,
                                              self.ydot,
                                              self.psidot,
                                              self.vdot)
        cmd_msg = Twist()
        cmd_msg.linear.x = speed
        cmd_msg.angular.z = angle 
        self.cmd_pub.publish(cmd_msg)
        self.prev = current_time

#PID controller class
class PIDController():
    def __init__(self):
        #try to get the vehicle information, if no file is found then load the default parameters. Note these parameters are only used for LQR 
        try:
            self.lr = rospy.get_param('~lr')
            self.lf = rospy.get_param('~lf')
            self.m = rospy.get_param('~m')
            self.ca = rospy.get_param('~ca')
            self.g = rospy.get_param('~g')

        except:
            print("couldn't find yaml file. load default parameters")
            self.lr = 0.2
            self.lf = 0.3
            self.ca = 80
            self.g  = 9.81
            self.m = 4
        self.error_angle_cum = 0
        self.error_angle_prev = 0
        self.error_dist_cum = 0
        self.error_dist_prev = 0

    #update the motor output and angle
    def update(self, goal, psi, dt, X, Y, xdot, ydot, psidot, vdot):
        if goal.all() == None:
            return
        lr = self.lr
        lf = self.lf
        ca = self.ca
        m = self.m
        g = self.g
        error_dist_cum = self.error_dist_cum
        error_dist_prev = self.error_dist_prev
        error_angle_cum = self.error_angle_cum
        error_angle_prev = self.error_angle_prev
        
        # PID Tuning parameters
        P_dist = 0.08
        I_dist = 0.06
        D_dist = 0.02
        P_angle = 0.15
        I_angle = 0.08
        D_angle = 0.05
        scaling = 2e3 

        #error_dist = V_desire - vdot
        #print(error_dist)
        #error_dist_cum = error_dist * dt + error_dist_cum
        #error_dist_diff = (error_dist - error_dist_prev) / dt
        #error_dist_prev = error_dist
        #throttle = (P_dist * error_dist + I_dist * error_dist_cum + D_dist * error_dist_diff)
        #comment out / this is constant speed

        #throttle = 0.3
        
        #throttle should be a function of distance to the goal location
        throttle = scaling*(np.linalg.norm((np.array([goal[0] - X, goal[1] - Y]))))
        print("Original throttle:" + str(throttle))
        throttle = max(min(throttle, 0.6),0.15)
        #Longitudal Controller

        #get the next point to go = to do
        print(goal)
        #angle_desire = np.arctan2((goal[0] - Y),(goal[1] - X)) this is very confusing, need rework
        #angle_desire = np.arctan2((goal[1] - X), (goal[0] - Y))


        #The following formulas are used to compensate the shape of Earth
        X_rel = np.cos(np.deg2rad(goal[0])) * np.sin(np.deg2rad(goal[1] - Y))
        Y_rel = np.cos(np.deg2rad(X)) * np.sin(np.deg2rad(goal[0])) - np.sin(np.deg2rad(X)) * np.cos(np.deg2rad(goal[0])) * np.cos(np.deg2rad(goal[1] - Y))
        #print("first part: " + str(np.cos(np.deg2rad(goal[0]))))
        #print("second part: " + str(np.sin(np.deg2rad(goal[1] - Y))))
        #print(" X_rel " + str(X_rel) + " Y_rel " + str(Y_rel))
        angle_desire = np.arctan2(X_rel, Y_rel)

        print("current pos:" + str(X) + " " + str(Y))
        print("goal: " + str(goal[0]) + " " + str(goal[1]))
        print("angle_desire: " + str(angle_desire) ) 
        error_angle = wrapTopi(angle_desire - psi)

        error_angle_cum = error_angle * dt + error_angle_cum
        error_angle_diff = (error_angle - error_angle_prev) / dt
        error_angle_prev = error_angle

        delta = max(min((P_angle * error_angle + I_angle * error_angle_cum + D_angle * error_angle_diff), 1), -1) #limit the output between 1 and -1. This is the input into the controller
        
        print("angle command: " + str(delta))
        print("Throttle command: " + str(throttle))
        return -delta,throttle #reverse the direction of the angle controller

#currently, LQR controller is not used due to some tuning issues but the infrastructure is developed for the future use.
class LQRController():
    def __init__(self,traj):
        #try to get the vehicle information
        try:
            self.lr = rospy.get_param('~lr')
            self.lf = rospy.get_param('~lf')
            self.m = rospy.get_param('~m')
            self.ca = rospy.get_param('~ca')
            self.g = rospy.get_param('~g')

        except:
            print("couldn't find yaml file. load default parameters")
            self.lr = 0.2
            self.lf = 0.3
            self.ca = 80
            self.g  = 9.81
            self.m = 4
        self.error1_prev = 0
        self.error2_prev = 0
        self.error_dist_cum = 0
        self.error_dist_prev = 0

    def update(self, traj):
        traj = self.traj
        lr = self.lr
        lf = self.lf
        ca = self.ca
        m = self.m
        g = self.g
        error_dist_cum = self.error_dist_cum
        error_dist_prev = self.error_dist_prev
        error_angle_cum = self.error_angle_cum
        error_angle_prev = self.error_angle_prev
        # PID Tuning parameters
        P_dist = 0.8
        I_dist = 0.6
        D_dist = 0.2
        P_angle = 0.6
        I_angle = 0.0
        D_angle = 0.4
        V_desire = 10

        #get updated states, need to be implemented
        #dt time step

        #Lateral Controller
        error_dist = V_desire - vdot
        error_dist_cum = error_dist * dt + error_dist_cum
        error_dist_diff = (error_dist - error_dist_prev) / dt
        error_dist_prev = error_dist
        throttle = (P_dist * error_dist + I_dist * error_dist_cum + D_dist * error_dist_diff)

        #Longitudal Controller

        #get the next point to go = to do
        angle_desire = np.arctan2((traj[index][1] - Y),(traj[index][0] - X))
        e1 = point #this is the next point
        e1_dot = (e1 - error1_prev) / dt
        e2 = wrapTopi(-angle_desire + psi)
        e2_dot = (e2 - error2_prev ) / dt
        A = np.matrix([[0, 1, 0, 0], [0, -(4 * ca / (m * xdot)), (4 * ca / m), -(2 * ca * (lf - lr) / (m * xdot))], [0, 0, 0, 1], [0, -(2 * ca * (lf - lr) / (z * xdot)), (2 * ca * (lf - lr) / Z), -(2 * ca * (lf ** 2 + lr ** 2)) / (Z * xdot), 0]])
        B = np.matrix([[0 , 0], [(2 * ca / m), 0], [0, 0], [(2 * ca * lf / Z), 0]])
        C = np.identity(4)
        D = 0 
        d_system = signal.cont2discrete((A, B, C, D), dt)
        A_dis = d_system[0]
        B_dis = d_system[1]

        #Tunable
        Q = np.matrix([[0.1, 0, 0, 0],[0, 0.1, 0, 0],[0, 0, 20, 0],[0, 0, 0, 6]])
        R = np.matrix([[1,0],[0,2]])
        S = np.matrix(linalg.solve_discrete_are(A_dis, B_dis, Q, R))
        K = np.matrix(linalg.inv(B_dis.T@S@B + R) @ (B_dis.T@S@A_dis))
        error_angle_cum = error_angle * dt + error_angle_cum
        error_angle_diff = (error_angle - error_angle_prev) / dt
        error_angle_prev = error_angle
        delta = (P_angle * error_angle + I_angle * error_angle_cum + D_angle * error_angle_diff)

        return delta,throttle


if __name__ == '__main__':
    gps_navigation()
