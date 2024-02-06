#!/usr/bin/env python3

import time
import math
import hebi
import rospy
import copy
import numpy as np

from std_msgs.msg import Header, Float64, Time, String, Bool, Int64
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point
from anakin_control.msg import ForceMsg, ModuleMsg, MotorMsg, FeedbackMsg
from pathlib import Path
from trajectories import workspaceTrajectory

class CommandPublisherHebiAPI:

	# __init__
	def __init__(self):

		# initialize node
		rospy.init_node('CommandPublisherHebiAPI', anonymous=True)
		rospy.logwarn("initializing command node...")

		# initialize publishers
		# self.pubJointStates = rospy.Publisher('hebi_joint_states', JointState, queue_size=10) # NEED TO DEFINE JsMsg
		self.pubAuger = rospy.Publisher('auger_commands', ModuleMsg, queue_size=10)
		self.pubRototillerHeight = rospy.Publisher('rototiller_height_commands', ModuleMsg, queue_size=10)
		self.pubRototillerSpeed = rospy.Publisher('other_topic', MotorMsg, queue_size=10)
		self.pubCommands = rospy.Publisher('hebi_arm_commands', JointState, queue_size=10)
		self.pubMode = rospy.Publisher('manip_mode', String, queue_size=1)

		# set kinematics model
		#path = Path('/home/nick/Desktop/cmu/research/robomechanics/chevron/'\
		#	'hebi_ws/src/hebi_cpp_api_examples/config/hrdf/A-2085-04.hrdf') # ---- sim A-2085-04
		# path = Path('/home/nick/Desktop/cmu/research/robomechanics/chevron/'\
		# 	'hebi_ws/src/anakin_control/config/hrdf/anakin.hrdf') # ---- sim anakin
		# path = Path('/home/rover/catkin_ws/src/anakin_control/config/hrdf/anakin.hrdf') # ---- real robot
		path = Path('/home/patrick/catkin_ws/src/anakin_control/config/hrdf/anakin.hrdf') # ---- real robot
		self.arm = hebi.robot_model.import_from_hrdf(path)

		# # initialize Hebi to Python connection
		# lookup = hebi.Lookup()
		# family = ["Arm_RML"] # DOUBLE CHECK THIS
		self.names = ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist"]
		# self.armGroup = lookup.get_group_from_names(family, self.names)
		# self.armFbk = self.armGroup.get_next_feedback(timeout_ms=1000)
		

		# set up a bunch o' states
		self.jointStates = [0, 0, 0, 0]
		self.pos = [0, 0, 0, 0] # may not need vel, acc variables
		self.vel = [0, 0, 0, 0] # they're here for consistency
		self.acc = [0, 0, 0, 0]
		self.eff = [0, 0, 0, 0]
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.phiOffset = 0
		self.dist = 0 # magnitude of self.r
		self.r = np.array([0, 0])
		self.unitR = np.array([0, 0])
		self.alpha = 0
		self.stowed = True
		self.rForce = 0
		self.zForce = 0
		self.trajectoryBacktracking = False
		self.motorSpeed = 0 # rototiller speed
		self.motorStop = False # rototiller kill switch
		# self.rototillerHeight = 0 # rototiller module angle
		self.rototillerHeightDirection = 0 # (-1, 0, 1) for (down, hold, up)
		self.rototillerHeight = 0
		self.augerTwistDirection = 0 # (-1, 0, 1) for (cw, hold, ccw)
		self.estop = False

		# initialize mode
		self.mode = 'scoop' # options: 'scoop', 'auger', 'rototiller'

		# initialize forward kinematics
		self.transforms = self.arm.get_forward_kinematics('output', self.pos)
		self.realTransforms = self.arm.get_forward_kinematics('output', self.jointStates)

		# initialize joystick states
		self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

		# set time to get from one point to the next
		# CHECK IF a) THERE ARE NECESSARY ANYMORE AND b) IF SO, WHETHER THEY NEED TO CHANGE
		self.updateRate = 0.01 # changed from 0.001
		self.updateRateRef = 0.01 # changed from 0.001

		# set perturbation magnitudes
		self.rPert = 0.00075 # m, last value: 0.00075
		self.zPert = 0.00075 # m, last value: 0.00075
		self.tPert = 0.0025 # rad, last value: 0.00025
		self.pPert = 0.005 # rad, last value: 0.005
		self.augerPert = 0.00075 # m [TUNE]
		# self.rototillerPert = 0.0025 # m [TUNE]

		# set home positions [TUNE FOR AUGER COMPATIBILITY]
		# self.homeOpen =  [4.71, 2.0797, -1.6, 3.4037] # real J3: -1.8406 --- this line for original scoop orientation
		self.homeOpen = [4.71, 2.0797, -1.6, -3.36] # open to -3(pi)/2 to play nice with the auger
		self.homeClosed = [4.71, 0.11, -2.90, 0.10]
		self.digPos = ["dummy", 2.5908, -1.1095, 4.8388] # first element is irrelevant
		self.digPos2 = ["dummy", 2.3103, -1.7679, 3.7617]
		self.digPos3 = ["dummy", 2.5908, -1.1095, 3.4206]

		# set limits
		self.thetaMin, self.thetaMax = 1.60, 5.10
		self.rMin, self.rMax = 0.20, 0.55
		self.zMin, self.zMax = -0.30, 0.375
		self.phiMin, self.phiMax = -6, 6
		self.rotoHeightMin, self.rotoHeightMax = -3.14, 3.14 
		self.motorSpeedMax = 600

		# set force limits and trajectory depth
		self.rForceMax = 100
		self.zForceMax = 150 
		self.tDepth = 0.15 # [m]

		# set trajectory types and variables
		self.trajectoryTypes = ['elliptical', 'parabolic', 'rectangular', 'forcePID']
		self.trajectoryIndex = 0
		self.trajectoryType = "rectangular" # default
		self.trajectoryLength = 0.250 # [m]
		self.trajectoryDepth = 0.150 # [m]
		self.trajectoryDuration = 5 # [s]

		# set up joystick blocking
		self.joyBlocked = False
		self.joyBlockTimeCurr = 0
		self.joyBlockTime = 0

		# initialize log arrays
		self.rLog = []
		self.zLog = []

		# seed self.pos variable on init
		self.init = False # will cause joystickHandler() to seed self.pos

		rospy.Subscriber('joystick2', Joy, self.joystickUpdater) 
		rospy.Subscriber('hebi_joint_states', JointState, self.joystickHandler) # REPLACE WITH HEBI API get_next_feedback or something
		rospy.Subscriber('rz_forces', ForceMsg, self.forceUpdater)
		rospy.Subscriber('rototiller_speed_feedback', FeedbackMsg, self.printRotoSpeed)
		rospy.Subscriber('rototiller_height_feedback', Float64, self.rototillerHeightUpdater)
		# rospy.Subscriber('e_stop', StopMsg, self.killMotion)
		rospy.Subscriber('single_msgs', FeedbackMsg, self.publishAll) # lets us know module node is running
		rospy.Subscriber('soft_estop/enable', Bool, self.estopEnable)
		rospy.Subscriber('soft_estop/reset', Bool, self.estopDisable)

		# Timed Publisher for Mode
		rospy.Timer(rospy.Duration(1.0/10.0), self.publishMode)

		# print a message
		rospy.logwarn('commmand node initialized')

		# loop de doop
		rospy.spin()

	# PUBLICATIONS ------------------------------------------------------------------

	def publishStates(self):
		
		cmd = JointState()
		cmd.header = Header()
		cmd.header.stamp = rospy.Time.now()
		cmd.name = self.names
		cmd.position = self.pos
		cmd.velocity = self.vel
		cmd.effort = self.eff
		self.pubCommands.publish(cmd)

		# LET'S MAKE SURE THIS IS ACTUALLY NECESSARY, KEEPING IT HERE FOR NOW THOUGH
		if self.updateRate != self.updateRateRef:
			time.sleep(self.updateRate)
			self.updateRate = self.updateRateRef
		else:
			time.sleep(self.updateRate)

	def publishAugerCommands(self):
		moduleCmd = ModuleMsg(self.augerTwistDirection, 0, 0, 0)
		self.pubAuger.publish(moduleCmd)

	def publishRototillerHeightCommands(self):
		moduleCmd = ModuleMsg(self.rototillerHeightDirection, 0, 0, 0)
		self.pubRototillerHeight.publish(moduleCmd)

	def publishRototillerSpeedCommands(self):
		speedStop = MotorMsg(self.motorSpeed, self.motorStop)
		self.pubRototillerSpeed.publish(speedStop)

	def publishAll(self, data): # technically also a callback
		#while not rospy.is_shutdown():
                self.publishRototillerSpeedCommands()
                self.publishRototillerHeightCommands()
                self.publishAugerCommands()
                #time.sleep(self.updateRateRef)
	
	def publishMode(self, event=None):
		self.pubMode.publish(self.mode)


	# CALLBACKS -------------------------------------------------------------------

	def forceUpdater(self, data):
		# update force state variables
		self.rForce = data.r
		self.zForce = data.z

	def joystickUpdater(self, data):
		# take joystick input, update joystick states
		print(data.axes)
		print(data.buttons)
		self.axes = data.axes
		self.buttons = data.buttons

	def estopEnable(self, data):
		if data.data == True:
			rospy.logwarn("estop enabled")
			self.estop = True
			self.motorSpeed = 0
			
	def estopDisable(self, data):
		if data.data == True:
			rospy.logwarn("estop reset")
			self.estop = False

	def printRotoSpeed(self, data):
		rospy.logwarn("rototiller speed: %f rpm", data.speed)

	def rototillerHeightUpdater(self, data):
		self.rototillerHeight = data

	def joystickHandler(self, data):
		# run appropriate callback based on current joystick states
		# run on each /joint_state publication for consistent refresh rate

		# check if self.pos is initialized
		if not self.init:
			d = data.position
			eff = data.effort
			self.jointStates = [d[0], d[1], d[2], d[3]]
			self.pos = [d[0], d[1], d[2], d[3]]
			self.eff = [0, 0, 0, 0] # [eff[0], eff[1], eff[2], eff[3]]
			self.init = True
			self.updateStates()

		if ((not self.joyBlocked) and (not self.estop)):
			# shorthand
			a = self.axes
			b = self.buttons 

			# switch modes
			if b[8] == 1: # LOGITECH
				self.blockJoystick(0.5)
				if self.mode == 'scoop':
					self.mode = 'auger'
				elif self.mode == 'auger':
					self.mode = 'rototiller'
				elif self.mode == 'rototiller':
					self.mode = 'scoop'
				rospy.logwarn("Current mode: %s", self.mode)

			if self.mode == "rototiller":
				if a[3] > 0.5 and self.motorSpeed < self.motorSpeedMax: # right joystick up
					self.motorSpeed += 5
					# rospy.logwarn("speed: %f", self.motorSpeed)
					# self.blockJoystick(0.5)
				if a[3] < -0.5 and self.motorSpeed > -1*self.motorSpeedMax: # right joystick down
					self.motorSpeed -= 5
					# rospy.logwarn("speed: %f", self.motorSpeed)
					# self.blockJoystick(0.5)
				if a[7] == 1: # arrow pad up
					self.motorSpeed = 600 # speed up gradually
					#rospy.logwarn("speed: %f", self.motorSpeed)
					self.blockJoystick(0.5)
				if a[7] == -1: # arrow pad down
					self.motorSpeed = 0 # slow down gradually
					#rospy.logwarn("speed: %f", self.motorSpeed)
					self.blockJoystick(0.5)
				if b[1] == 1: # B
					# self.motorStop = True
					# self.motorSpeed = 0
					#rospy.logwarn("quick stop")
					pass
				if a[1] > 0.5: # left joystick up
					self.rototillerHeightDirection = 1
				if a[1] < -0.5: # left joystick down
					self.rototillerHeightDirection = -1
				if a[1] > -0.5 and a[1] < 0.5: # left joystick inactive
					self.rototillerHeightDirection = 0 # hold height
				if b[2] == 1:
					self.rototillerFindGround()
				if b[3] == 1:
					self.rototillerReturnHome()
					
			else:
				# toggle stowed/unstowed position
				if b[6] == 1: # BACK
					self.goHome(1)
					self.goHome(0)
				if b[7] == 1: # START
					self.goHome(1)

				# modify states
				if not self.stowed:
					if a[3] > 0.5 and self.dist < self.rMax: # right joystick up
						self.perturbR(1) # ee out
					if a[3] < -0.5 and self.dist > self.rMin: # right joystick down
						self.perturbR(-1) # ee in
					if a[2] > 0.5 and self.pos[0] < self.thetaMax: # right joystick left
						self.perturbTheta(1) # base counterclockwise
					if a[2] < -0.5 and self.pos[0] > self.thetaMin: # right joystick right
						self.perturbTheta(-1) # base clockwise
					if a[1] > 0.5 and self.z < self.zMax: # left joystick up
						self.perturbZ(1) # ee up
					if a[1] < -0.5 and self.z > self.zMin: # left joystick down
						self.perturbZ(-1) # ee down
					if a[4] < 0 and self.pos[3] < self.phiMax: # RT
						self.perturbPhi(-1) # positive scoop angle
					if a[5] < 0 and self.pos[3] > self.phiMin: # LT
						self.perturbPhi(1) # negative scoop angle
					if a[0] > 0.5: # left joystick left
						pass
					if a[0] < -0.5: # left joystick right
						pass
					if a[7] == -1 and self.mode == 'auger': # arrow pad down
						self.augerTwistDirection = 1
						# self.perturbZ(-1)
					if a[7] == 1 and self.mode == 'auger': # arrow pad up
						self.augerTwistDirection = -1
					if a[7] == 0 and self.mode == 'auger': # arrow pad untouched
						self.augerTwistDirection = 0
					if a[6] > 0: # arrow pad left
						pass
					if a[6] < 0: # arrow pad right
						pass

					if b[0] == 1: # A
						pass
					# emergency stop
					if b[1] == 1: # B
						pass
					# execute trajectory
					if b[2] == 1: # X
						if self.mode == 'scoop':
							self.rLog = []
							self.zLog = []
							if self.trajectoryType == 'forcePID':
								self.detectContactForceObserver()
							else:
								self.executePrescribedTrajectory()
								rospy.logwarn("done")
							# plt.plot(self.rLog, self.zLog)
							# plt.show()
						else:
							rospy.logwarn("switch from auger mode to scoop mode in order to execute trajectories")
					# print positions (for now)
					if b[3] == 1: # Y
						rospy.logwarn(self.pos)
					# toggle trajectory backtracking
					if b[4] == 1: # LB
						self.trajectoryBacktracking = not self.trajectoryBacktracking
						self.blockJoystick(0.5)
						if self.trajectoryBacktracking:
							rospy.logwarn("trajectory backtracking on")
						else:
							rospy.logwarn("trajectory backtracking off")	
					# switch trajectory type
					if b[5] == 1: # RB
						self.trajectoryIndex = (self.trajectoryIndex + 1) % len(self.trajectoryTypes)
						self.trajectoryType = self.trajectoryTypes[self.trajectoryIndex]
						rospy.logwarn("%s trajectory...", self.trajectoryType)
						self.blockJoystick(0.5)
						rospy.logwarn("set")
					
					if b[9] == 1: # LEFT JOYSTICK DEPRESSED
						pass
					if b[10] == 1: # RIGHT JOYSTICK DEPRESSED
						pass

		# publish states
		self.publishStates()
		self.updateStates()
		self.checkJoystickBlock()
		"""self.publishRototillerSpeedCommands()
		self.publishRototillerHeightCommands()
		"""

	# ACTIONS ----------------------------------------------------------------

	def blockJoystick(self, joyBlockTime):
		if not self.joyBlocked:
			self.joyBlocked = True
			self.joyBlockTimeCurr = 0
			self.joyBlockTime = joyBlockTime

	def checkJoystickBlock(self): 
		if self.joyBlocked:
			self.joyBlockTimeCurr += self.updateRate
			if self.joyBlockTimeCurr >= self.joyBlockTime:
				self.joyBlocked = False	

	def updateStates(self):

		# XYZ
		self.transforms = self.arm.get_forward_kinematics('output', self.pos)
		# self.realTransforms = self.arm.get_forward_kinematics('output', self.jointStates)
		xyz = self.transforms[6]
		self.x = xyz[0][3]
		self.y = xyz[1][3]
		self.z = xyz[2][3]

		# Alpha
		L = 0.325 # link length between J3 and J4
		z3 = self.transforms[4][2][3]
		z4 = self.transforms[6][2][3]
		deltaZ = z3 - z4
		self.alpha = math.asin(deltaZ/L)

		# radial stuff
		L = 0.0535 # offset of ee from origin in [m]
		offsetOriginX = L*math.cos(self.pos[0])
		offsetOriginY = L*math.sin(self.pos[0])
		self.dist = math.sqrt((self.x - offsetOriginX)**2 + (self.y - offsetOriginY)**2)
		self.r = np.array([self.x - offsetOriginX, self.y - offsetOriginY])
		self.unitR = self.r/self.dist

	def holdPhi(self):
		self.phiOffset = -1*self.pos[3] + self.alpha

	def housekeeping(self):
		"""self.updateXYZ()
		self.updateAlpha()"""
		self.updateStates()
		self.holdPhi()

	def doIK(self, x, y, z, excludeJ1=False):
		# takes target x, y, z, updates self.pos accordingly
		target_xyz = [x, y, z]
		initial_joint_angles = np.array(self.pos)
		end_effector_position_objective = hebi.robot_model.endeffector_position_objective(target_xyz)
		newPos = self.arm.solve_inverse_kinematics(initial_joint_angles, end_effector_position_objective)
		if excludeJ1:
			self.pos[1:] = newPos[1:]
		else:
			self.pos = newPos

		self.preserveGlobalPhi()

	def preserveGlobalPhi(self):
		# preserves scoop angle relative to ground while not being explicity commanded
		self.pos[3] = 1*self.alpha - self.phiOffset

	def generateHebiTrajectory(self, startPos, endPos, duration): 
		# generate a trajectory as defined in the Hebi Python API
		# endPos should be a numpy array of lenth 4

		numWaypoints = 2 # keep it simple - start and end
		numJoints = 4

		# empty waypoint arrays
		pos = np.empty((numJoints, numWaypoints))
		vel = np.empty((numJoints, numWaypoints))
		acc = np.empty((numJoints, numWaypoints))

		# set velocity/acceleration to be zero at endpoints
		vel[:,0] = acc[:,0] = 0.0
		vel[:,-1] = acc[:,-1] = 0.0
		vel[:,1:-1] = acc[:,1:-1] = np.nan # this probably does nothing given two endpoints

		# set up position waypoints
		pos[:,0] = startPos
		pos[:,1] = endPos

		# time things
		time = np.linspace(0, duration, numWaypoints)

		# create the damm thing and return it
		trajectory = hebi.trajectory.create_trajectory(time,pos,vel,acc)
		return trajectory

	def executeHebiTrajectory(self, trajectory):
		# this method is for specific position trajectories
		# trajectory should be output struct from hebi.trajectory.create_trajectory()
		duration = trajectory.duration
		t = 0
		while (t < duration):
			posCmd, velCmd, accCmd = trajectory.get_state(t)
			self.pos = posCmd
			self.vel = velCmd
			self.acc = accCmd
			self.publishStates() # this should sleep for self.updateRate long
			t += self.updateRate

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

		traj = self.generateHebiTrajectory(self.pos, goal, 7)
		self.executeHebiTrajectory(traj)
		self.publishStates()
		self.housekeeping()

		rospy.logwarn("home")

	def perturbR(self, direction):
		# move ee radially on joystick input
		# direction: (1, -1) = (out, in)
		L = 0.0535 # offset of ee from origin in [m]
		newX = self.x + direction*self.unitR[0]*self.rPert
		newY = self.y + direction*self.unitR[1]*self.rPert

		# send target x, y, z to ik function
		self.doIK(newX, newY, self.z, True)

	def perturbTheta(self, direction):
		# adjust base theta on joystick input
		# direction: (1, -1) = (counterclockwise, clockwise)
		self.pos[0] = self.pos[0] + direction*self.tPert

	def perturbZ(self, direction):
		# adjust ee height on joystick input
		# direction: (1, -1) = (up, down)
		newZ = self.z + direction*self.zPert
		self.doIK(self.x, self.y, newZ)

	def perturbPhi(self, direction):
		# adjust scoop angle on joystick input
		# direction: (1, -1) = (positive, negative)
		self.phiOffset += direction*self.pPert
		self.preserveGlobalPhi()


	# CONTROLLERS ---------------------------------------------------------------------------

	def findGround(self):

		rospy.logwarn("initializing contact detection...")		

		# get in safe position
		goal = [self.pos[0]] 
		goal = goal.append(self.digPos2[1:]) # copying first elt of self.digPos is unsafe
		traj = self.generateHebiTrajectory(self.pos, goal, 3)
		self.executeHebiTrajectory(traj)
		self.publishStates()
		self.housekeeping()

		rospy.logwarn("looking...")

		# find ground
		while (abs(self.zForce) < 20):
			self.perturbZ(-1)
			self.publishStates()
			self.updateStates()
			time.sleep(0.01)
			# self.rLog.append(self.dist)
			# self.zLog.append(self.z)

		rospy.logwarn("done")

	def detectContactForceObserver(self):
		
		rospy.logwarn("intializing force observer contact detection...")

		# get in safe position
		goal = [self.pos[0]] 
		goal = goal.append(self.digPos2[1:]) # copying first elt of self.digPos is unsafe
		traj = self.generateHebiTrajectory(self.pos, goal, 3)
		self.executeHebiTrajectory(traj)
		self.publishStates()
		self.housekeeping()

		rospy.logwarn("looking...")
		
		# find something - FIX THIS 
		iterations = 0
		prevR = self.rForce
		prevZ = self.zForce
		forceMetric = math.sqrt((self.rForce-prevR)**2 + (self.zForce-prevZ)**2)
		forceMetricThreshold = 2
		while (forceMetric < forceMetricThreshold and abs(self.zForce < 15)):
			if (iterations > 50):
				self.perturbZ(-1)
				self.publishStates()
				self.updateStates()
				if (iterations % 10 == 0):
					forceMetric = math.sqrt((self.rForce-prevR)**2 + (self.zForce-prevZ)**2)
					prevR = self.rForce
					prevZ = self.zForce
				# rospy.logwarn("forceMetric: %f",forceMetric)
				time.sleep(0.001)
			iterations += 1

		rospy.logwarn("found something!")

	# just position feedback
	def rototillerFindGround(self):

		rospy.logwarn("bringing rotitller to ground...")

		setpoint = 1 # THIS NEEDS TO CHANGE
		while (self.rototillerHeight > setpoint):
			self.rototillerHeightDirection = -1
			self.publishStates()
			self.updateStates()
			time.sleep(0.001)

			rospy.logwarn("done.")

	def rototillerReturnHome(self):

		rospy.logwarn("raising rototiller...")

		setpoint = 1 # THIS NEEDS TO CHANGE
		while (self.rototillerHeight < setpoint):
			self.rototillerHeightDirection = 1
			self.publishStates()
			self.updateStates()
			time.sleep(0.001)

		rospy.logwarn("done.")



# TO DO: ADD MORE CONTROLLERS

if __name__ == '__main__':
	node = CommandPublisherHebiAPI()
