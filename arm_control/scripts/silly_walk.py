#!/usr/bin/env python3

import time
import math
import hebi
import rospy
import rospkg
import copy
import numpy as np

from pathlib import Path

from std_msgs.msg import Header, Int16, Float32, String, Bool, Time
from sensor_msgs.msg import JointState, Joy
from anakin_control.msg import ForceMsg
from geometry_msgs.msg import Twist

class Controller:

        def __init__(self):

                # initialize node
                rospy.init_node('Controller', anonymous=True)
                rospy.logwarn("initializing controller...")

                # initialize publishers
                self.pubRototillerHeight = rospy.Publisher('rototiller_height_commands', JointState, queue_size=10)
                self.pubRototillerSpeed = rospy.Publisher('rototiller_speed_commands', Int16, queue_size=10)
                self.pubArmCommands = rospy.Publisher('hebi_arm_commands', JointState, queue_size=10)
                self.pubMode = rospy.Publisher('manip_mode', String, queue_size=1)
                self.pubDrive = rospy.Publisher('/cmd_vel/managed', Twist, queue_size=1) 

                # set kinematics model
                # path = Path('/home/rover/catkin_ws/src/anakin_control/config/hrdf/anakin.hrdf') # ---- real robot
                rospack = rospkg.RosPack()
                ak = rospack.get_path('anakin_control')
                path = Path(ak + '/config/hrdf/anakin.hrdf')
                self.arm = hebi.robot_model.import_from_hrdf(path)

                # states
                self.armPosition = None
                self.armPositionCmd = [0, 0, 0, 0]
                self.armVelocity = [0, 0, 0, 0]
                self.armVelocityCmd = [0, 0, 0, 0]
                self.armEffort = [0, 0, 0, 0] # don't really care about initializing this one
                self.armEffortCmd = [0, 0, 0, 0] # always 0's
                self.rForce = 0
                self.zForce = 0
                self.rotoHeight = None
                self.rotoHeightCmd = None
                self.rotoSpeedCmd = 0
                self.lipoVIN = 0
                self.mode = "arm"
                self.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                self.oldButtons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                self.axes = [0, 0, 0, 0, 0, 0, 0, 0]
                self.oldAxes = [0, 0, 0, 0, 0, 0, 0, 0]
                self.stowed = True              
                self.estop = False
                self.initialized = False
                self.homing = False
                self.sillyWalking = False
                self.sillyHoming = False

                # wifi connection status variables
                self.maxOffTime = 2 # seconds; if disconnected longer than this, trigger e-stop
                self.lastComm = rospy.get_time()
                self.comms = True
        
                # joint limits and home positions
                self.maxRotoSpeed = 100
                self.minRotoSpeed = -100
                self.minRotoHeight = -100
                self.maxRotoHeight = 0.6
                self.rMin, self.rMax = 0.20, 0.55
                self.zMin, self.zMax = -0.30, 0.5#0.375
                self.thetaMin, self.thetaMax = 1.60, 5.10
                self.phiMin, self.phiMax = -6, 6
                self.homeOpen = [4.75, 2.0797, -1.6, -3.36]
                self.homeClosed = [4.75, 0.11, -2.90, -3.38]

                # arm trajectory execution states
                self.executing = False
                self.executingSilly = False
                self.trajectory = None
                self.executionBegin = 0 # ROS time when execution began
                self.sillyDuration = 10

                # arm math variables
                self.r = 0 # radius from center
                self.z = 0 # height
                self.theta = 0 # base angle
                self.phi = 0 # scoop angle (kept constant during teleoperation)
                self.phiOffset = -self.homeOpen[3] # added to self.phi when scoop angle adjustment commanded
                self.alpha = 0 # angle used to calculate self.phi
                self.x = 0
                self.y = 0
                self.rVec = np.array([0, 0])
                self.unitR = np.array([0, 0])

                # position increments
                self.rotoHeightInc = 0.005
                self.rotoSpeedInc = 1
                self.rInc = 0.00075
                self.zInc = 0.00075 
                self.thetaInc = 0.0025
                self.phiInc = 0.005

                # initialize subscribers
                rospy.Subscriber('joystick2', Joy, self.updateJoystick) 
                rospy.Subscriber('hebi_joint_states', JointState, self.updateArm) # REPLACE WITH HEBI API get_next_feedback or something
                # rospy.Subscriber('rz_forces', ForceMsg, self.updateForces)
                rospy.Subscriber('rototiller_hebi_fbk', JointState, self.updateRototillerHeight)
                rospy.Subscriber('rz_forces', ForceMsg, self.updateForces)
                rospy.Subscriber('soft_estop/enable', Bool, self.enableEstop)
                rospy.Subscriber('soft_estop/reset', Bool, self.resetEstop)
                rospy.Subscriber('comms_check', Bool, self.updateComms)
                # rospy.Subscriber('talker', Bool, self.updateCmd)
        
                # timers for publishing commands to hardware/mode to gui, checking wifi signal
                rospy.Timer(rospy.Duration(0.01), self.updateCmd)
                rospy.Timer(rospy.Duration(1.0/10.0), self.publishMode)
                rospy.Timer(rospy.Duration(0.1), self.checkComms)

                # finish initialization and spin
                rospy.logwarn('controller initialized')
                rospy.spin()

        
        ### CALLBACKS / PUBLISHERS ...............................................................................

        def publishMode(self, event=None):
                self.pubMode.publish(self.mode)

        def updateJoystick(self, data):
                self.buttons = data.buttons
                self.axes = data.axes
        
        def updateArm(self, data):
                self.armPosition = list(data.position)
                self.armVelocity = list(data.velocity)
                self.armEffort = list(data.effort)
                self.updateArmCoordinates()
        
        def updateForces(self, data):
                self.rForce = data.r
                self.zForce = data.z

        def updateRototillerHeight(self, data):
                self.rotoHeight = list(data.position)
        
        def enableEstop(self, data):
                if not self.estop and data.data:
                        self.estop = True
                        self.rotoSpeedCmd = 0
                        self.homing = False
                        rospy.logwarn("enabled e-stop")

        def resetEstop(self, data):
                if self.estop and data.data:            
                        self.estop = False      
                        rospy.logwarn("reset e-stop")

        def updateComms(self, data):
                self.lastComm = rospy.get_time()

        def checkComms(self, data):
                now = rospy.get_time()
                if self.comms and now - self.lastComm >= self.maxOffTime:
                        self.comms = False
                        self.enableEstop(Bool(True))
                        rospy.logwarn("lost wifi network, e-stop enabled")
                elif not self.comms and now - self.lastComm <= self.maxOffTime:
                        self.comms = True
                        self.resetEstop(Bool(True))
                        rospy.logwarn("re-connected to wifi network, e-stop reset")
        
        def checkStow(self):
                jBound = [0.05, 0.08, 0.06, 0.05]
                setpoint = [4.74, 0.001, -2.99, -3.43]
                fbk = self.armPosition
                for i in range(3):
                        if fbk[i] < setpoint[i] - jBound[i] or fbk[i] > setpoint[i] + jBound[i]:
                                return False
                return True
                
        def publishCmd(self):

                # format and send JointState msg to arm
                armCmd = JointState()
                armCmd.header = Header()
                armCmd.header.stamp = rospy.Time.now()
                armCmd.name = ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist"]
                # armCmd.position = self.armPositionCmd
                # armCmd.velocity = self.armVelocityCmd
                if self.stowed and self.checkStow():            
                        armCmd.position = [np.nan, np.nan, np.nan, self.armPositionCmd[3]]
                        armCmd.velocity = [np.nan, np.nan, np.nan, self.armVelocityCmd[3]]
                else:
                        armCmd.position = self.armPositionCmd
                        armCmd.velocity = self.armVelocityCmd
                armCmd.effort = self.armEffortCmd # this is a dummy cmd
                self.pubArmCommands.publish(armCmd)
                
                # format and send JointState msg to rototiller Hebi module
                rotoHeightCmd = JointState()
                rotoHeightCmd.header = Header()
                rotoHeightCmd.header.stamp = rospy.Time.now()
                rotoHeightCmd.name = "rototiller_hebi_module"
                rotoHeightCmd.position = self.rotoHeightCmd
                rotoHeightCmd.velocity = [0]
                rotoHeightCmd.effort = [0]
                self.pubRototillerHeight.publish(rotoHeightCmd)
        
                # format and send rototiller speed command
                rotoSpeedCmd = Int16(self.rotoSpeedCmd)
                self.pubRototillerSpeed.publish(rotoSpeedCmd)

        # update command state variables using joystick and other state variables
        def updateCmd(self, data):

                # don't send a command to hardware until position/speed estimates are available
                if not self.initialized:

                        # give the arm/roto states initial values from feedback
                        if self.armPosition != None:
                                self.armPositionCmd = self.armPosition
                        if self.rotoHeight != None:
                                self.rotoHeightCmd = self.rotoHeight
        
                        # if not yet initialized, print message and return, otherwise unlock teleop
                        if (self.armPositionCmd == [0, 0, 0, 0] or self.rotoHeightCmd == None):
                                # rospy.logwarn("waiting for hardware feedback")
                                pass
                        else:
                                self.initialized = True
                                rospy.logwarn("ready")
                        return
                                        
                # keep track of buttons changing state vs. buttons "being pressed" --> one event vs. many
                a = self.axes
                b = self.buttons
                a0 = self.oldAxes
                b0 = self.oldButtons

                if self.executing:
                        if (rospy.get_time() - self.executionBegin >= self.trajectory.duration) or self.estop:
                                self.stopExecution()
                        else:
                                self.executeHebiTrajectory()

                elif self.homing:
                        self.homing = False
                        self.goHome(0)

                elif self.executingSilly:
                        if (rospy.get_time() - self.executionBegin >= self.trajectory.duration) or self.estop:
                                self.stopExecution()
                                self.executionBegin = rospy.get_time()
                        else:
                                self.executeHebiTrajectory()

                elif self.sillyWalking:
                        if (rospy.get_time() - self.executionBegin >= self.sillyDuration) or self.estop:
                                self.stopExecution()
                                self.sillyWalking = False
                        else:
                                self.executeSillyTrajectory()

                elif self.sillyHoming:
                        self.sillyHoming = False
                        self.goHome(0)

                elif (not self.estop):
                
                        # switch modes
                        if b[8] == 1 and b0[8] == 0: # LOGITECH BUTTON
                                if self.mode == 'arm':
                                        self.mode = 'rototiller'
                                elif self.mode == 'rototiller':
                                        self.mode = 'arm'
                                rospy.logwarn("Current mode: %s", self.mode)

                        # rototiller mode
                        if self.mode == "rototiller":
                                #if a[3] > 0.5 and self.rotoSpeedCmd + self.rotoSpeedInc <= self.maxRotoSpeed: # ..... right joystick up
                                #        # self.rotoSpeedCmd += self.rotoSpeedInc
                                #        pass
                                #elif a[3] < -0.5 and self.rotoSpeedCmd - self.rotoSpeedInc >= self.minRotoSpeed: # .. right joystick down
                                #        # self.rotoSpeedCmd -= self.rotoSpeedInc
                                #        pass
                                """ start new commands """
                                if a[3] > 0.5 and self.rotoHeightCmd[0] <= self.maxRotoHeight: # ..................... right joystick up
                                        self.rotoHeightCmd[0] = self.rotoHeightCmd[0] + self.rotoHeightInc
                                if a[3] < -0.5 and self.rotoHeightCmd[0] >= self.minRotoHeight: # ..................... right joystick down
                                        self.rotoHeightCmd[0] = self.rotoHeightCmd[0] - self.rotoHeightInc
                                if a[1] > 0.1 or a[1] < -0.1:
                                    driveCmd = Twist()
                                    driveCmd.linear.x = a[1]/3
                                    self.pubDrive.publish(driveCmd)
                                """ end new commands """
                                if a[7] == 1 and abs(a0[7]) == 0: # .................................................... arrow pad up
                                        self.rotoSpeedCmd = max(self.rotoSpeedCmd-100,-100)
                                if a[7] == -1 and abs(a0[7]) == 0: # ................................................... arrow pad down
                                        self.rotoSpeedCmd = min(self.rotoSpeedCmd+100,100)
                                #if a[1] > 0.5 and self.rotoHeightCmd[0] <= self.maxRotoHeight: # ..................... left joystick up
                                #        self.rotoHeightCmd[0] = self.rotoHeightCmd[0] + self.rotoHeightInc
                                #if a[1] < -0.5 and self.rotoHeightCmd[0] >= self.minRotoHeight: # ..................... left joystick down
                                #        self.rotoHeightCmd[0] = self.rotoHeightCmd[0] - self.rotoHeightInc
                                if a[1] > -0.5 and a[1] < 0.5: # .................................................. left joystick inactive
                                        pass

                        # arm mode
                        elif self.mode == "arm":
                                if self.stowed:
                                        # unstow the arm (the only thing you can do if it's stowed)
                                        if b[7] == 1 and b0[7] == 0: # ................................................... START
                                                self.goHome(1)
                                        if b[3] == 1:
                                                rospy.logwarn(self.armPosition)
                                        if b[2] == 1:
                                                self.sillyWalking = True
                                                self.sillyHoming = True
                                                self.goHomeSilly()
                                else:
                                        if b[6] == 1 and b0[6] == 0: # ................................................... BACK
                                                self.homing = True
                                                self.goHome(1)
                                        if a[3] > 0.5 and self.r + self.rInc <= self.rMax: # ........................... right joystick up
                                                self.incrementR(1) # ee out
                                        if a[3] < -0.5 and self.r - self.rInc >= self.rMin: # .......................... right joystick down
                                                self.incrementR(-1) # ee in
                                        if a[2] > 0.5 and self.theta + self.thetaInc <= self.thetaMax: # ............... right joystick left
                                                self.incrementTheta(1) # base counterclockwise
                                        if a[2] < -0.5 and self.theta - self.thetaInc >= self.thetaMin: # .............. right joystick right
                                                self.incrementTheta(-1) # base clockwise
                                        if a[1] > 0.5 and self.z + self.zInc <= self.zMax: # ........................... left joystick up
                                                self.incrementZ(1) # ee up
                                        if a[1] < -0.5 and self.z - self.zInc >= self.zMin: # .......................... left joystick down
                                                self.incrementZ(-1) # ee down
                                        if a[4] < 0 and self.phi + self.phiInc <= self.phiMax: # ....................... RT
                                                self.incrementPhi(-1) # positive scoop angle
                                        if a[5] < 0 and self.phi - self.phiInc >= self.phiMin: # ....................... LT
                                                self.incrementPhi(1) # negative scoop angle
                                        if b[3] == 1: # ................................................................ Y
                                                rospy.logwarn(self.armPosition)

                                        # LEAVE OUT TRAJECTORIES/FORCE CONTROLLERS FOR NOW, JUST FOCUS ON BASICS
                                        # WHEN RE-IMPLEMENTING THESE, FIT CODE BELOW TO NEW FRAMEWORK - THIS IS OLD CODE
                                        # switch trajectory type
                                        """if b[5] == 1 and b0[5] == 0: # RB
                                                self.trajectoryIndex = (self.trajectoryIndex + 1) % len(self.trajectoryTypes)
                                                self.trajectoryType = self.trajectoryTypes[self.trajectoryIndex]
                                                rospy.logwarn("%s trajectory selected", self.trajectoryType)

                                        # execute trajectory
                                        if b[2] == 1: # X
                                                rLog = []
                                                zLog = []
                                                if self.trajectoryType == 'forcePID':
                                                        self.detectContactForceObserver()
                                                else:
                                                        self.executePrescribedTrajectory()
                                                        rospy.logwarn("done")
                                                # plt.plot(rLog, zLog)
                                                # plt.show()"""

                                        

                self.oldAxes = a
                self.oldButtons = b
                self.publishCmd() # send command states to hardware

        ### UTILITY FUNCTIONS ...........................................................................................

        def updateArmCoordinates(self):

                # XYZTheta
                self.transforms = self.arm.get_forward_kinematics('output', self.armPositionCmd)
                xyz = self.transforms[6]
                self.x = xyz[0][3]
                self.y = xyz[1][3]
                self.z = xyz[2][3]
                self.theta = self.armPositionCmd[0]

                # alpha (angle between last arm link and horizontal) and phi
                L = 0.325 # link length between J3 and J4
                z3 = self.transforms[4][2][3]
                z4 = self.transforms[6][2][3]
                deltaZ = z3 - z4
                self.alpha = math.asin(deltaZ/L)
                self.holdPhi()

                # radial stuff
                L = 0.0535 # offset of ee from origin in [m]
                offsetOriginX = L*math.cos(self.armPositionCmd[0])
                offsetOriginY = L*math.sin(self.armPositionCmd[0])
                self.r = math.sqrt((self.x - offsetOriginX)**2 + (self.y - offsetOriginY)**2)
                self.rVec = np.array([self.x - offsetOriginX, self.y - offsetOriginY])
                self.unitR = self.rVec/self.r

        def doIK(self, x, y, z): #, excludeJ1=False):
                # takes target x, y, z, updates self.armPositionCmd accordingly
                target_xyz = [x, y, z]
                initial_joint_angles = np.array(self.armPositionCmd)
                end_effector_position_objective = hebi.robot_model.endeffector_position_objective(target_xyz)
                armPositionCmd = self.arm.solve_inverse_kinematics(initial_joint_angles, end_effector_position_objective)
                """if excludeJ1:
                        self.armPositionCmd[1:] = armPositionCmd[1:]
                else:
                        self.armPositionCmd = armPositionCmd"""
                self.armPositionCmd = armPositionCmd

                # maintain constant scoop angle phi
                self.holdPhi()

        def incrementR(self, direction):
                # move ee radially on joystick input
                # direction: (1, -1) = (out, in)
                L = 0.0535 # offset of ee from origin in [m]
                newX = self.x + direction*self.unitR[0]*self.rInc
                newY = self.y + direction*self.unitR[1]*self.rInc

                # send target x, y, z to ik function
                self.doIK(newX, newY, self.z)

        def incrementTheta(self, direction):
                # adjust base theta on joystick input
                # direction: (1, -1) = (counterclockwise, clockwise)
                self.armPositionCmd[0] = self.armPositionCmd[0] + direction*self.thetaInc

        def incrementZ(self, direction):
                # adjust ee height on joystick input
                # direction: (1, -1) = (up, down)
                newZ = self.z + direction*self.zInc
                self.doIK(self.x, self.y, newZ)

        def incrementPhi(self, direction):
                # adjust scoop angle on joystick input
                # direction: (1, -1) = (positive, negative)
                self.phiOffset += direction*self.phiInc
                self.holdPhi()

        def holdPhi(self):
                self.phi = self.alpha - self.phiOffset 
                self.armPositionCmd[3] = self.phi

        def syncPhi(self):
                self.phiOffset = self.alpha - self.armPositionCmd[3]

        # NEED TO FINISH GOING OVER THESE, THEN ADD INCREMENTATION FUNCTIONS
        def generateHebiTrajectory(self, startPos, endPos, duration): 
                # generate a trajectory as defined in the Hebi Python API
                # endPos should be a numpy array of lenth 4

                numWaypoints = 2 # keep it simple - start and end
                numJoints = 4

                # empty waypoint arrays
                pos = np.empty((numJoints, numWaypoints))
                vel = np.empty((numJoints, numWaypoints))
                acc = np.empty((numJoints, numWaypoints))

                # set velocity/acceleration to be zero at endpoints, nan for the rest if more than 2 waypoints
                vel[:,0] = acc[:,0] = 0.0
                vel[:,-1] = acc[:,-1] = 0.0
                vel[:,1:-1] = acc[:,1:-1] = np.nan # this does nothing given < 3 waypoints

                # set up position waypoints
                pos[:,0] = startPos
                pos[:,1] = endPos

                # time things
                time = np.linspace(0, duration, numWaypoints)

                # create the damm thing and return it
                trajectory = hebi.trajectory.create_trajectory(time,pos,vel,acc)
                return trajectory

        def executeSillyTrajectory(self):
                elapsed = rospy.get_time() - self.executionBegin
                f = 2
                amp = 0.75
                j0 = self.armPositionCmd[0]
                j1 = 1.57 + amp*math.sin(f*elapsed)
                j2 = amp*math.cos(f*elapsed)
                j3 = math.cos(4*f*elapsed)
                self.armPositionCmd = [j0, j1, j2, j3]
                rospy.logwarn(self.armPositionCmd)
                self.syncPhi()

                driveCmd = Twist()
                driveCmd.linear.x = 0.5*math.sin(0.75*f*elapsed) + 1
                self.pubDrive.publish(driveCmd)

        def executeHebiTrajectory(self):
                elapsed = rospy.get_time() - self.executionBegin # float in seconds
                positionCmd, velocityCmd, _ = self.trajectory.get_state(elapsed)
                self.armPositionCmd = positionCmd
                self.armVelocityCmd = velocityCmd
                self.syncPhi()
        
        def stopExecution(self):
                self.executing = False
                self.executingSilly = False
                self.armVelocityCmd = [0, 0, 0, 0]

        def goHome(self, variant): # FIRST GENERATE A TRAEJECTORY, THEN EXECUTE IT WITHIN FCN
                if variant == 0:
                        goal = copy.copy(self.homeClosed)
                        self.stowed = True
                        state = "closed"
                else:
                        goal = copy.copy(self.homeOpen)
                        self.stowed = False
                        state = "open"

                rospy.logwarn("going to %s home position...", state)

                self.trajectory = self.generateHebiTrajectory(self.armPositionCmd, goal, 7)
                self.executing = True
                self.executionBegin = rospy.get_time() # float in seconds

        def goHomeSilly(self):
                goal = [4.75, 1.57, 0, 0]
                self.stowed = False
                self.executingSilly = True
                self.trajectory = self.generateHebiTrajectory(self.armPositionCmd, goal, 7)
                self.executionBegin = rospy.get_time()

if __name__ == '__main__':
        node = Controller()


                








