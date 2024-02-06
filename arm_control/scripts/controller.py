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
                #self.pubAugerHeight = rospy.Publisher('auger_height_commands', JointState, queue_size=10)
                #self.pubAugerSpeed = rospy.Publisher('auger_speed_commands', Int16, queue_size=10)
                self.pubArmCommands = rospy.Publisher('hebi_arm_commands', JointState, queue_size=10)
                self.pubMode = rospy.Publisher('manip_mode', String, queue_size=1)
                self.pubDrive = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 

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
                self.augerHeight = None
                self.augerHeightCmd = None
                self.augerSpeedCmd = 0
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

                # wifi connection status variables
                self.maxOffTime = 5 # seconds; if disconnected longer than this, trigger e-stop
                self.lastComm = rospy.get_time()
                self.comms = True
        
                # joint limits and home positions
                self.maxRotoSpeed = 100
                self.minRotoSpeed = -100
                self.minRotoHeight = -100
                self.maxRotoHeight = 0.45
                self.maxAugerSpeed = 100
                self.minAugerSpeed = -100
                self.rMin, self.rMax = 0.20, 0.55
                self.zMin, self.zMax = -0.30, 0.5 #0.375
                self.thetaMin, self.thetaMax = 1.60, 5.10
                self.phiMin, self.phiMax = -6, 6
                # self.homeOpen = [4.75, 2.0797, -1.6, 0.30] # RIC Demo
                # self.homeClosed = [4.75, 0.11, -2.90, 0.30] # RIC Demo

                self.homeOpen = [4.75, 2.0797, -1.6, 0.0] # PXRF Sensor
                self.homeClosed = [4.75, 0.11, -2.90, 0.0] # PXRF Sensor

                self.activationThreshold = 0.2 # magnitude between 0 and 1 above which joystick becomes active
                self.triggerActivationThreshold = 0.75 # must be > 0

                # arm trajectory execution states
                self.executing = False
                self.trajectory = None
                self.executionBegin = 0 # ROS time when execution began

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
                self.psi = 0
                self.psiOffset = 0

                # position increments
                self.raiseRotoHeightInc = 0.005
                self.lowerRotoHeightInc = 0.0025
                self.rotoHeightInc = 0.0025
                self.rotoHeightIncMax = 0.005
                self.rotoSpeedInc = 1
                self.augerSpeedInc = 1
                self.rInc = 0.00075
                self.zInc = 0.00075 
                self.thetaInc = 0.0025
                self.phiInc = 0.020
                self.rIncMax = 0.001
                self.zIncMax = 0.001 
                self.thetaIncMax = 0.005
                self.phiIncMax = 0.01
                self.augerHeightInc = 0.007
                self.augerHeightIncMax = 0.01
                self.augerVelFactor = 0.0

                # initialize subscribers
                rospy.Subscriber('joy', Joy, self.updateJoystick) 
                rospy.Subscriber('hebi_joint_states', JointState, self.updateArm) # REPLACE WITH HEBI API get_next_feedback or something
                # rospy.Subscriber('rz_forces', ForceMsg, self.updateForces)
                rospy.Subscriber('rototiller_hebi_fbk', JointState, self.updateRototillerHeight)
                #rospy.Subscriber('auger_hebi_fbk', JointState, self.updateAugerHeight)
                rospy.Subscriber('rz_forces', ForceMsg, self.updateForces)
                rospy.Subscriber('soft_estop/enable', Bool, self.enableEstop)
                rospy.Subscriber('soft_estop/reset', Bool, self.resetEstop)
                #rospy.Subscriber('comms_check', Bool, self.updateComms)
                #rospy.Subscriber('talker', Bool, self.updateCmd)
        
                # timers for publishing commands to hardware/mode to gui, checking wifi signal
                rospy.Timer(rospy.Duration(0.01), self.updateCmd)
                rospy.Timer(rospy.Duration(1.0/10.0), self.publishMode)
                #rospy.Timer(rospy.Duration(0.1), self.checkComms)

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

        def updateAugerHeight(self, data):
                self.augerHeight = list(data.velocity)
        
        def enableEstop(self, data):
                if not self.estop and data.data:
                        self.estop = True
                        self.rotoSpeedCmd = 0
                        self.augerSpeedCmd = 0
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
                #setpoint = [4.74, 0.001, -2.99, -3.43]
                setpoint = self.homeClosed
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

                # format and send JointState msg to auger Hebi module
                #augerHeightCmd = JointState()
                #augerHeightCmd.header = Header()
                #augerHeightCmd.header.stamp = rospy.Time.now()
                #augerHeightCmd.name = "auger_hebi_module"
                #augerHeightCmd.position = [float(self.augerHeightCmd[0])]
                #augerHeightCmd.velocity = [0]
                #augerHeightCmd.effort = [0]
                #self.pubAugerHeight.publish(augerHeightCmd)

                # format and send rototiller speed command
                #augerSpeedCmd = Int16(self.augerSpeedCmd)
                #self.pubAugerSpeed.publish(augerSpeedCmd)

        # update command state variables using joystick and other state variables
        def updateCmd(self, data):
                # don't send a command to hardware until position/speed estimates are available
                if not self.initialized:

                        # give the arm/roto states initial values from feedback
                        if self.armPosition != None:
                                self.armPositionCmd = self.armPosition
                        if self.rotoHeight != None:
                                self.rotoHeightCmd = self.rotoHeight
                        #if self.augerHeight != None:
                        #        self.augerHeightCmd = self.augerHeight
        
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
                                rotoHeightDirection = 0
                                if a[3] > self.activationThreshold: # and self.rotoHeightCmd[0] <= self.maxRotoHeight: # ..................... right joystick up
                                        # self.rotoHeightCmd[0] = self.rotoHeightCmd[0] + self.raiseRotoHeightInc
                                        rotoHeightDirection += 1
                                        self.rotoHeightInc = self.rotoHeightIncMax*(abs(a[3]) - self.activationThreshold)/(1 - self.activationThreshold)
                                if a[3] < -self.activationThreshold: # and self.rotoHeightCmd[0] >= self.minRotoHeight: # ..................... right joystick down
                                        # self.rotoHeightCmd[0] = self.rotoHeightCmd[0] - self.lowerRotoHeightInc
                                        rotoHeightDirection -= 1
                                        self.rotoHeightInc = self.rotoHeightIncMax*(abs(a[3]) - self.activationThreshold)/(1 - self.activationThreshold)
                                driveCmd = Twist()
                                driveRobot = False
                                driveMultiplier = 2.0
                                if b[5] > self.triggerActivationThreshold and b[5] != 0.0: # ....................... RT
                                    driveMultiplier *=2.0
                                if a[0] > self.activationThreshold or a[0] < -self.activationThreshold:
                                    driveCmd.angular.z = a[0]*1.5*driveMultiplier
                                    driveRobot = True
                                if a[1] > self.activationThreshold or a[1] < -self.activationThreshold:
                                    driveCmd.linear.x = a[1]/2.0*driveMultiplier
                                    driveRobot = True
                                if driveRobot:
                                    self.pubDrive.publish(driveCmd)
                                self.incrementRototillerHeight(rotoHeightDirection)
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
                                else:
                                        thetaDirection = 0
                                        rDirection = 0
                                        zDirection = 0
                                        phiDirection = 0
                                        augerHeightDirection = 0
                                        if b[6] == 1 and b0[6] == 0: # ................................................... BACK
                                                self.homing = True
                                                self.goHome(1)
                                        if a[4] > self.activationThreshold: # and self.r + self.rInc <= self.rMax: # ........................... right joystick up
                                                rDirection+=1
                                                self.rInc = self.rIncMax*(abs(a[4]) - self.activationThreshold)/(1 - self.activationThreshold)
                                                #self.incrementR(1) # ee out
                                        if a[4] < -self.activationThreshold: # and self.r - self.rInc >= self.rMin: # .......................... right joystick down
                                                rDirection-=1
                                                self.rInc = self.rIncMax*(abs(a[4]) - self.activationThreshold)/(1 - self.activationThreshold)
                                                #self.incrementR(-1) # ee in
                                        if a[3] > self.activationThreshold: # and self.theta + self.thetaInc <= self.thetaMax: # ............... right joystick left
                                                thetaDirection+=1
                                                self.thetaInc = self.thetaIncMax*(abs(a[3]) - self.activationThreshold)/(1 - self.activationThreshold)
                                                #self.incrementTheta(1) # base counterclockwise
                                        if a[3] < -self.activationThreshold: # and self.theta - self.thetaInc >= self.thetaMin: # .............. right joystick right
                                                thetaDirection-=1
                                                self.thetaInc = self.thetaIncMax*(abs(a[3]) - self.activationThreshold)/(1 - self.activationThreshold)
                                                #self.incrementTheta(-1) # base clockwise
                                        if a[5] < self.activationThreshold: # and self.z + self.zInc <= self.zMax: # ........................... left joystick up
                                                zDirection+=1
                                                self.zInc = self.zIncMax*(abs(a[5]) - self.activationThreshold)/(1 - self.activationThreshold)
                                                #self.incrementZ(1) # ee up
                                        if a[2] < -self.activationThreshold: # and self.z - self.zInc >= self.zMin: # .......................... left joystick down
                                                zDirection-=1
                                                self.zInc = self.zIncMax*(abs(a[2]) - self.activationThreshold)/(1 - self.activationThreshold)
                                                #self.incrementZ(-1) # ee down
                                        if a[1] < -self.activationThreshold and a[1] != 0.0: # self.phi + self.phiInc <= self.phiMax: # ....................... RT
                                                # self.incrementPhi(-1) # positive scoop angle
                                                phiDirection -= 1
                                                self.phiInc = self.phiIncMax*(self.activationThreshold - abs(a[1]))/(1 + self.triggerActivationThreshold)
                                        if a[1] > self.triggerActivationThreshold and a[1] != 0.0: # self.phi - self.phiInc >= self.phiMin: # ....................... LT
                                                # self.incrementPhi(1) # negative scoop angle
                                                phiDirection += 1
                                                self.phiInc = self.phiIncMax*(self.activationThreshold - abs(a[1]))/(1 + self.triggerActivationThreshold)
                                        #if b[5] > self.triggerActivationThreshold and b0[5] == 0 and self.augerVelFactor < 2.0: # and self.rotoHeightCmd[0] <= self.maxRotoHeight: # ..................... right joystick up
                                                # self.rotoHeightCmd[0] = self.rotoHeightCmd[0] + self.raiseRotoHeightInc
                                        #        augerHeightDirection = 1.0
                                        #        self.augerVelFactor = self.augerVelFactor + 1.0
                                        #if b[4] > self.triggerActivationThreshold and b0[4] == 0 and self.augerVelFactor > -2.0: # and self.rotoHeightCmd[0] >= self.minRotoHeight: # ..................... right joystick down
                                                # self.rotoHeightCmd[0] = self.rotoHeightCmd[0] - self.lowerRotoHeightInc
                                        #        augerHeightDirection = -1.0
                                        #        self.augerVelFactor = self.augerVelFactor - 1.0
                                        if b[3] == 1: # ................................................................ Y
                                                rospy.logwarn(self.armPosition)
                                        self.incrementArmPos(thetaDirection,rDirection,zDirection,phiDirection)
                                        #self.incrementAugerHeight(augerHeightDirection)

                                        # LEAVE OUT TRAJECTORIES/FORCE CONTROLLERS FOR NOW, JUST FOCUS ON BASICS
                                        # WHEN RE-IMPLEMENTING THESE, FIT CODE BELOW TO NEW FRAMEWORK - THIS IS OLD CODE
                                        # switch trajectory type
                                        """if b[5] == 1 and b0[5] == 0: # RB
                                                self.trajectoryIndex = (self.trajectoryIndex + 1) % len(self.trajectoryTypes)
                                                self.trajectoryType = self.trajectoryTypes[self.trajectoryIndex]
                                                rospy.logwarn("%s trajectory selected", self.trajectoryType)
                                        
                                        # execute trajectory to dump soil
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
                aPC = self.arm.solve_inverse_kinematics(initial_joint_angles, end_effector_position_objective)
                """if excludeJ1:
                        self.armPositionCmd[1:] = armPositionCmd[1:]
                else:
                        self.armPositionCmd = armPositionCmd"""
               
                self.armPositionCmd = aPC

                # maintain constant scoop angle phi
                self.holdPhi()

        def incrementArmPos(self,thetaDirection=0,rDirection=0,zDirection=0,phiDirection=0):
                
                if ((thetaDirection > 0 and self.theta + self.thetaInc > self.thetaMax) or 
                    (thetaDirection < 0 and self.theta - self.thetaInc < self.thetaMin)): 
                        thetaDirection = 0               
                if ((rDirection > 0 and self.r + self.rInc > self.rMax) or 
                    (rDirection < 0 and self.r - self.rInc < self.rMin)):
                        rDirection = 0
                if ((zDirection > 0 and self.z + self.zInc > self.zMax) or 
                    (zDirection < 0 and self.z - self.zInc < self.zMin)):
                        zDirection = 0
                if ((phiDirection > 0 and self.phi + self.phiInc > self.phiMax) or 
                    (phiDirection < 0 and self.phi - self.phiInc < self.phiMin)):
                        phiDirection = 0
                       
                L = 0.0535 # offset of ee from origin in [m]
                newX = self.x + rDirection*self.unitR[0]*self.rInc
                newY = self.y + rDirection*self.unitR[1]*self.rInc
                newZ = self.z + zDirection*self.zInc
                self.phiOffset += phiDirection*self.phiInc
                
                 # self.holdPhi()
                self.doIK(newX, newY, newZ)
                self.armPositionCmd[0] = self.armPositionCmd[0] + thetaDirection*self.thetaInc

        def incrementRototillerHeight(self,direction=0):
                if ((direction > 0 and self.rotoHeight[0] + self.rotoHeightInc > self.maxRotoHeight) or
                    (direction < 0 and self.rotoHeight[0] - self.rotoHeightInc < self.minRotoHeight)):
                        direction = 0
                self.rotoHeightCmd[0] = self.rotoHeightCmd[0] + direction*self.rotoHeightInc

        #def incrementAugerHeight(self,direction=0):
        #        self.augerHeightCmd[0] = self.augerHeightCmd[0] + self.augerVelFactor*self.augerHeightInc

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
        def generateHebiTrajectory(self, startPos, endPos, duration, midpoints=[]): 
                # generate a trajectory as defined in the Hebi Python API
                # endPos should be a numpy array of lenth 4
                # midpoints is a n x 5 2D array with columns [j0, j1, j2, j3, t]^T

                numWaypoints = 2 
                if len(midpoints) > 0:
                        numWaypoints += len(midpoints[0]) # start and end + midpoints
                numJoints = 4

                # empty waypoint arrays
                pos = np.empty((numJoints, numWaypoints))
                vel = np.empty((numJoints, numWaypoints))
                acc = np.empty((numJoints, numWaypoints))

                # set velocity/acceleration to be zero at endpoints, nan for the rest if more than 2 waypoints
                vel[:,0] = acc[:,0] = 0.0
                vel[:,-1] = acc[:,-1] = 0.0
                vel[:,1:-1] = acc[:,1:-1] = 0.0 # this does nothing given < 3 waypoints

                # set up position waypoints
                # if numWaypoints <= 2:
                pos[:,0] = startPos
                pos[:,-1] = endPos
                # else:
                i = 1
                while i < numWaypoints - 1:
                        pos[:,i] = midpoints[:-1,i-1]
                        i += 1

                # time things
                if numWaypoints <= 2:
                        time = np.linspace(0, duration, numWaypoints)
                else:
                        time = [0] + list(midpoints[-1]) + [duration]

                # create the damm thing and return it
                trajectory = hebi.trajectory.create_trajectory(time,pos,vel,acc)
                return trajectory

        def executeHebiTrajectory(self):
                elapsed = rospy.get_time() - self.executionBegin # float in seconds
                positionCmd, velocityCmd, _ = self.trajectory.get_state(elapsed)
                self.armPositionCmd = positionCmd
                self.armVelocityCmd = velocityCmd
                self.syncPhi()
        
        def stopExecution(self):
                print("Execution stopped")
                self.executing = False
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

if __name__ == '__main__':
        node = Controller()


                
























                




                

        

        
                



