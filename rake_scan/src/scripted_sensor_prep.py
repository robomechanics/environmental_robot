#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry
import tf
import math
import numpy as np
from std_msgs.msg import Float64, Bool
from std_srvs.srv import SetBool
from autonomy_manager.srv import RunSensorPrep

# script used for rake and scan and pxrf only
sensorPrepScript_broom = (
        #('drive',(1,0,0)),
        ('rake',-2.5,2), # lower torque (non-negative for raise), time to wait for raise
        ('drive',(0.5,0,0)),
        ('rake',1,2),
        ('drive',(0,0,0)),
        ('rake',-2.5,2), 
        ('drive',(0.5,0,0)),
        ('rake',1,2),
        ('drive',(-0.8,0,0)),
        ('pxrf',-1,5),
        #('wait',5),
        #('pxrf',1,2),
        #('drive',(0,0,0)),
        )

sensorPrepScript_rake = (
        #('drive',(1,0,0)),
        ('rake',-2.5,2), # lower torque (non-negative for raise), time to wait for raise
        ('drive',(0.5,0,0)),
        ('rake',1,2),
        ('drive',(0,0,0)),
        ('rake',-2.5,2), 
        ('drive',(0.5,0,0)),
        ('rake',1,2),
        ('drive',(-1,0,0)),
        ('pxrf',-1,5),
        #('wait',5),
        #('pxrf',1,2),
        #('drive',(0,0,0)),
        )

sensorPrepScript_pxrf_only = (
        #('drive',(1,0,0)),
        ('pxrf',-1,5),
        #('wait',5),
        #('pxrf',1,2),
        #('drive',(0,0,0)),
        )

sensorPrepScript = sensorPrepScript_pxrf_only

# move forward
scriptForward = (
        ('drive',(2,0,0)),
)
# move backward
scriptBackward = (
     ('drive',(-2,0,0)),
)

def wrap(angle):
    angle = angle%(2*math.pi)
    if angle > math.pi:
        angle = angle-2*math.pi
    return angle

class scripted_sensor_prep(object):
    def __init__(self):
        # start node
        rospy.init_node('scripted_sensor_prep', anonymous=True)
        rospy.Subscriber('/odom',Odometry,self.odom_callback)
        self.pose = None
        while self.pose is None:
            rospy.sleep(0.1)
        self.dig_torque_pub = rospy.Publisher('/dig_torque',Float64, queue_size=10,latch=True)
        self.lowerRake = rospy.ServiceProxy('/deploy_tool_auto',SetBool)
        self.lowerPXRF = rospy.ServiceProxy('/deploy_sensor_auto',SetBool)
        self.drivePub = rospy.Publisher('/cmd_vel_auto', Twist, queue_size=1)
        self.statusPub = rospy.Publisher('/sensor_prep_status',Bool, queue_size=1,latch=True)
        self.goalPub = rospy.Publisher('/sensor_prep_goal',PoseStamped,queue_size=1,latch=True)
        self.logSensorPrep(False)
        rospy.Service('/run_sensor_prep',RunSensorPrep,self.logSensorPrep)
        rospy.Service('/move_fb', RunSensorPrep, self.moveFB)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.runOrStop:
                self.runScript(self.scriptToRun)
            rate.sleep()
    def logSensorPrep(self,runOrStop):
        if type(runOrStop) == bool:
            self.runOrStop = runOrStop
        else:
            self.runOrStop = runOrStop.run
        self.scriptToRun = sensorPrepScript
        msg = Bool()
        msg.data = self.runOrStop
        self.statusPub.publish(msg)
        return True

    def moveFB(self,move):
        if move.run:
            self.scriptToRun = scriptForward
        else:
            self.scriptToRun = scriptBackward
        self.runOrStop = True
        msg = Bool()
        msg.data = self.runOrStop
        self.statusPub.publish(msg)
        return True
        
    def runScript(self,script):
        oFrame = self.pose
        for task in script:
            if self.checkStopCondition():
                return
            if task[0] == 'drive':
                self.driveTo(self.convert2global(task[1],oFrame))
            if task[0] == 'rake':
                self.rake(task[1],task[2])
            if task[0] == 'pxrf':
                self.pXRF(task[1],task[2])
            if task[0] == 'wait':
                rospy.sleep(task[1])
        self.logSensorPrep(False)

    def rake(self,torque,waitTime):
        if torque >= 0:
            self.lowerRake(False)
        else:
            msg = Float64()
            msg.data = torque
            self.dig_torque_pub.publish(msg)
            self.lowerRake(True)
        rospy.sleep(waitTime)

    def pXRF(self,direction,waitTime):
        self.lowerPXRF(direction<=0)
        rospy.sleep(waitTime)

    def driveTo(self,goal):
        msg = PoseStamped()
        msg.pose.position.x,msg.pose.position.y = goal[0],goal[1]
        quat = tf.transformations.quaternion_from_euler(0,0,goal[2])
        msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w = quat
        self.goalPub.publish(msg)
        rate = rospy.Rate(10)
        reachCount = 0
        reachRequirement = 10
        while reachCount < reachRequirement:
            xDir,yDir,_ = self.convert2local(goal)
            msg = Twist()
            msg.linear.x = np.clip(xDir,-0.3,0.3)
            if xDir < 0:
                yDir = -yDir
            msg.angular.z = np.clip(yDir,-0.4,0.4)
            self.drivePub.publish(msg)
            rate.sleep()
            if self.checkStopCondition(): return
            if (goal[0]-self.pose[0])**2 + (goal[1]-self.pose[1])**2 < 0.05:
                reachCount+=1
            else:
                reachCount = 0
        reachCount = 0
        print('reached position')
        while reachCount < reachRequirement:
            angErr = goal[2]-self.pose[2]
            msg = Twist()
            msg.angular.z = np.clip(angErr,-0.5,0.5)
            self.drivePub.publish(msg)
            rate.sleep()
            if self.checkStopCondition(): return
            if np.abs(goal[2]-self.pose[2])<0.1:
                reachCount+=1
            else:
                reachCount = 0
        print('reached orientation')

    def odom_callback(self,data):
        pose = data.pose.pose
        quat = (pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        #heading = -wrap(euler[0]+math.pi)
        heading = euler[2]
        self.pose = (pose.position.x,pose.position.y,heading)

    def convert2global(self,relativePose,frame=None):
        if frame is None:
            frame = self.pose
        theta = frame[2]
        x = frame[0] + relativePose[0]*math.cos(theta) - relativePose[1]*math.sin(theta)
        y = frame[1] + relativePose[0]*math.sin(theta) + relativePose[1]*math.cos(theta)
        theta = wrap(theta + relativePose[2])
        return (x,y,theta)

    def convert2local(self,absolutePose,frame=None):
        if frame is None:
            frame = self.pose
        theta = frame[2]
        xDiff = absolutePose[0] - frame[0]
        yDiff = absolutePose[1] - frame[1]
        rel_x = xDiff*math.cos(theta) + yDiff*math.sin(theta)
        rel_y  = -xDiff*math.sin(theta) + yDiff*math.cos(theta)
        rel_theta = wrap(absolutePose[2] - theta)
        return (rel_x,rel_y,rel_theta)

    def checkStopCondition(self):
        if rospy.is_shutdown():
            return True
        if not self.runOrStop:
            return True
        return False


if __name__ == "__main__":
    scripted_sensor_prep()
