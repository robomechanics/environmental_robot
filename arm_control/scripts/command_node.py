#!/usr/bin/env python3

import rospy
import time
import math
import hebi
import copy
import numpy as np
# import matplotlib.pyplot as plt
from std_msgs.msg import Header
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point
from anakin_control.msg import ForceMsg, ModuleMsg, MotorMsg, FeedbackMsg
from pathlib import Path
from trajectories import workspaceTrajectory

class CommandPublisher:

	# INIT -----------------------------------------------------------------------------

	def __init__(self):

		# initialize node
		rospy.init_node('CommandPublisher', anonymous=True)

		# initialize publisher(s)
		self.pubJw = rospy.Publisher('joint_waypoints', JointTrajectory, queue_size=10)
		self.pubAuger = rospy.Publisher('auger_commands', ModuleMsg, queue_size=10)
		self.pubRototillerHeight = rospy.Publisher('rototiller_height_commands', ModuleMsg, queue_size=10)
		self.pubRototillerSpeed = rospy.Publisher('rototiller_speed_commands', MotorMsg, queue_size=10)

		# set kinematics model
		#path = Path('/home/nick/Desktop/cmu/research/robomechanics/chevron/'\
		#	'hebi_ws/src/hebi_cpp_api_examples/config/hrdf/A-2085-04.hrdf') # ---- sim A-2085-04
		#path = Path('/home/nick/Desktop/cmu/research/robomechanics/chevron/'\
			# 'hebi_ws/src/anakin_control/config/hrdf/anakin.hrdf') # ---- sim anakin
		path = Path('/home/rover/catkin_ws/src/anakin_control/config/hrdf/anakin.hrdf') # ---- real robot
		self.arm = hebi.robot_model.import_from_hrdf(path) 

		# initialize state variables
		self.jointStates = [0, 0, 0, 0]
		self.pos = [0, 0, 0, 0] # may not need vel, acc variables
		self.vel = [0, 0, 0, 0] # they're here for consistency
		self.acc = [0, 0, 0, 0]
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
		self.augerTwistDirection = 0 # (-1, 0, 1) for (cw, hold, ccw)

		# initialize mode
		self.mode = 'scoop' # options: 'scoop', 'auger', 'rototiller'

		# initialize forward kinematics
		self.transforms = self.arm.get_forward_kinematics('output', self.pos)
		self.realTransforms = self.arm.get_forward_kinematics('output', self.jointStates)

		# initialize joystick states
		self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

		# set time to get from one point to the next
		self.updateRate = 0.001
		self.updateRateRef = 0.001

		# set perturbation magnitudes
		self.rPert = 0.00025 # m, last value: 0.00075
		self.zPert = 0.00025 # m, last value: 0.00075
		self.tPert = 0.00125 # rad, last value: 0.00025
		self.pPert = 0.0025 # rad, last value: 0.005
		self.augerPert = 0.00075 # m [TUNE]
		# self.rototillerPert = 0.0025 # m [TUNE]

		# set home positions [TUNE FOR AUGER COMPATIBILITY]
		# self.homeOpen =  [4.71, 2.0797, -1.6, 3.4037] # real J3: -1.8406 --- this line for original scoop orientation
		self.homeOpen = [4.71, 2.0797, -1.6, -3.36] # open to -3(pi)/2 to play nice with the auger
		self.homeClosed = [4.71, 0.11, -2.90, 0.10]
		self.digPos = ["dummy", 2.5908, -1.1095, 4.8388] # first element is irrelevant
		self.digPos2 = ["dummy", 2.3103, -1.7679, -3.7617]
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

		# initialize log arrays
		self.rLog = []
		self.zLog = []

		# seed self.pos variable on init
		self.init = False # will cause joystickHandler() to seed self.pos

		# initialize subscriber(s)
		rospy.Subscriber('joy', Joy, self.joystickUpdater)
		rospy.Subscriber('joint_states', JointState, self.joystickHandler)
		rospy.Subscriber('rz_forces', ForceMsg, self.forceUpdater)
		rospy.Subscriber('rototiller_speed_feedback', FeedbackMsg, self.printRotoSpeed)
		# rospy.Subscriber('e_stop', StopMsg, self.killMotion)
		rospy.Subscriber('single_msgs', FeedbackMsg, self.publishAll) # let's hope this works

		# print a message
		print('initialized')

		# loop de doop
		rospy.spin()


	# PUBLICATIONS --------------------------------------------------------------------

	def publishStates(self):
		# load state variables into JointTrajectory object and publish
		jt = JointTrajectory()
		jt.header = Header()
		jt.joint_names = ['Arm/J1_base', 'Arm/J2_shoulder', 'Arm/J3_elbow', 'Arm/J4_wrist']
		jtp = JointTrajectoryPoint()
		jtp.positions = self.pos
		jtp.velocities = self.vel
		jtp.accelerations = self.acc
		jtp.time_from_start = rospy.Duration(self.updateRate)
		jt.points.append(jtp)
		self.pubJw.publish(jt)
		if self.updateRate != self.updateRateRef:
			time.sleep(self.updateRate)
			self.updateRate = self.updateRateRef

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
		while not rospy.is_shutdown():
			self.publishRototillerSpeedCommands()
			self.publishRototillerHeightCommands()
			self.publishAugerCommands()
			time.sleep(self.updateRateRef)
		

	# CALLBACKS -----------------------------------------------------------------

	def forceUpdater(self, data):
		# update force state variables
		self.rForce = data.r
		self.zForce = data.z

	def joystickUpdater(self, data):
		# take joystick input, update joystick states
		self.axes = data.axes
		self.buttons = data.buttons

	def printRotoSpeed(self, data):
		print("rototiller speed: ", data.speed, " rpm")

	def joystickHandler(self, data):
		# run appropriate callback based on current joystick states
		# run on each /joint_state publication for consistent refresh rate

		# check if self.pos is initialized
		if not self.init:
			d = data.position
			self.jointStates = [d[0], d[1], d[2], d[3]]
			self.pos = [d[0], d[1], d[2], d[3]]
			self.init = True
			self.updateStates()

		# shorthand
		a = self.axes
		b = self.buttons 

		# switch modes
		if b[8] == 1: # LOGITECH
			time.sleep(0.5)
			if self.mode == 'scoop':
				self.mode = 'auger'
			elif self.mode == 'auger':
				self.mode = 'rototiller'
			elif self.mode == 'rototiller':
				self.mode = 'scoop'
			print("Current mode: ", self.mode)

		if self.mode == "rototiller":
			if a[3] > 0.5 and self.motorSpeed < self.motorSpeedMax: # right joystick up
				self.motorSpeed += 5
				# print("speed: ", self.motorSpeed)
				time.sleep(0.05)
			if a[3] < -0.5 and self.motorSpeed > -1*self.motorSpeedMax: # right joystick down
				self.motorSpeed -= 5
				# print("speed: ", self.motorSpeed)
				time.sleep(0.05)
			if a[7] == 1: # arrow pad up
				self.motorSpeed = 600 # speed up gradually
				#print("speed: ", self.motorSpeed)
			if a[7] == -1: # arrow pad down
				self.motorSpeed = 0 # slow down gradually
				#print("speed: ", self.motorSpeed)
				time.sleep(0.05)
			if b[1] == 1: # B
				self.motorStop = True
				# self.motorSpeed = 0
				#print("quick stop")
			if a[1] > 0.5: # left joystick up
				self.rototillerHeightDirection = 1
			if a[1] < -0.5: # left joystick down
				self.rototillerHeightDirection = -1
			if a[1] > -0.5 and a[1] < 0.5: # left joystick inactive
				self.rototillerHeightDirection = 0 # hold height
				


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
							# self.digWithForceControl()
							# self.findGround()
							self.detectContactForceObserver()
						else:
							self.executePrescribedTrajectory()
							print("done")
						# plt.plot(self.rLog, self.zLog)
						# plt.show()
					else:
						print("switch from auger mode to scoop mode in order to execute trajectories")
				# print positions (for now)
				if b[3] == 1: # Y
					print(self.pos)
				# toggle trajectory backtracking
				if b[4] == 1: # LB
					self.trajectoryBacktracking = not self.trajectoryBacktracking
					time.sleep(0.5)
					print("trajectory backtracking: ", "on" if self.trajectoryBacktracking else "off")
				# switch trajectory type
				if b[5] == 1: # RB
					self.trajectoryIndex = (self.trajectoryIndex + 1) % len(self.trajectoryTypes)
					self.trajectoryType = self.trajectoryTypes[self.trajectoryIndex]
					print(self.trajectoryType, " trajectory...")
					time.sleep(0.5)
					print("set")
				
				if b[9] == 1: # LEFT JOYSTICK DEPRESSED
					pass
				if b[10] == 1: # RIGHT JOYSTICK DEPRESSED
					pass

		# publish states
		self.publishStates()
		self.updateStates()
		"""self.publishRototillerSpeedCommands()
		self.publishRototillerHeightCommands()
		"""

	# ACTIONS ---------------------------------------------------------------------------------------

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

	def goHome(self, variant):

		# go to specified home position
		if variant == 0:
			self.pos = copy.copy(self.homeClosed)
			self.stowed = True
			state = "closed"
		else:
			self.pos = copy.copy(self.homeOpen)
			self.stowed = False
			state = "open"

		print("going to ", state, " home position...")

		oldRate = self.updateRate
		self.updateRate = 7
		self.publishStates()
		# time.sleep(self.updateRate)
		# self.updateRate = oldRate

		# housekeeping
		self.housekeeping()

		print("home")

	def goHomeClosed(self):
		# goes to closed (stowed) home position
		self.pos = self.homeClosed

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

	# DEPRECATED
	def perturbRototiller(self, direction):
		# adjust position of rototiller's Hebi module
		# direction (1, -1) = (counterclockwise/up, clockwise/down)
		# self.rototillerHeight += direction*self.rototillerPert
		self.publishRototillerHeightCommands(direction)

	def executePrescribedTrajectory(self):
		# execute a prescibed trajectory (helper functions located in trajectories.py)

		print('getting in position...')

		# get in digging position
		if self.trajectoryType == "rectangular":
			self.pos[1:] = copy.copy(self.digPos3[1:])
		else:
			self.pos[1:] = copy.copy(self.digPos[1:]) # copying first elt of self.digPos is unsafe
		oldRate = self.updateRate
		self.updateRate = 3
		self.publishStates()
		time.sleep(self.updateRate)
		self.updateRate = oldRate
		self.housekeeping()

		print('ready')
		time.sleep(0.5)

		# generate a trajectory
		trajectory = workspaceTrajectory(self.pos, self.arm, self.trajectoryType, self.tDepth)

		# initialize some stuff
		t = 0.0
		period = 0.01
		duration = trajectory.duration
		pos_cmd = np.array(4, dtype=np.float64)
		startPhi = self.pos[3]

		print('executing trajectory...')
		#trajectoryLog = open("trajectory_log.txt", "w")

		# loop
		while (t < duration):
			if self.trajectoryBacktracking and (abs(self.rForce) > self.rForceMax or abs(self.zForce) > self.zForceMax):
				# backtrack to beginnning of trjectory
				print("encountered obstactle; backtracking...")
				while (t > 0):
					pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(t)
					self.pos = pos_cmd
					self.publishStates()
					self.updateStates()
					t -= period
					time.sleep(period)
					#trajectoryLog.write("[%s,%s,%s]" % (t, self.dist, self.z))
					self.rLog.append(self.dist)
					self.zLog.append(self.z)
				self.tDepth -= 0.1	
				print("attempting to execute trajectory with a smaller depth")
				self.executePrescribedTrajectory()
				break
			else:
				pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(t)
				self.pos = pos_cmd
				self.publishStates()
				self.updateStates()
				t += period
				time.sleep(period)
				#trajectoryLog.write("[%s,%s,%s]" % (t, self.dist, self.z))
				self.rLog.append(self.dist)
				self.zLog.append(self.z)

		# housekeeping
		self.housekeeping()
		self.tDepth = 0.2

	def findGround(self):

		print("initializing contact detection...")		

		# get in safe position
		self.pos[1:] = copy.copy(self.digPos2[1:]) # copying first elt of self.digPos is unsafe
		oldRate = self.updateRate
		self.updateRate = 3
		self.publishStates()
		time.sleep(self.updateRate)
		self.updateRate = oldRate
		self.housekeeping()
		time.sleep(0.5)

		print("looking...")

		# find ground
		while (abs(self.zForce) < 20):
			self.perturbZ(-1)
			self.publishStates()
			self.updateStates()
			time.sleep(0.01)
			# self.rLog.append(self.dist)
			# self.zLog.append(self.z)

		print("done")

	def detectContactForceObserver(self):
		
		print("intializing force observer contact detection...")

		# get in safe position
		self.pos[1:] = copy.copy(self.digPos2[1:]) # copying first elt of self.digPos2 is unsafe
		oldRate = self.updateRate
		self.updateRate = 3
		self.publishStates()
		time.sleep(self.updateRate)
		self.updateRate = oldRate
		self.housekeeping()
		time.sleep(0.5)

		print("looking...")
		
		# find something
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
				print("forceMetric: ",forceMetric)
				time.sleep(0.001)
			iterations += 1

		print("found something!")
		print("done.")

		

	def digWithForceControl(self):
		# run PID on z force setpoint
		ff_gain = 0.0001 # feed-forward gain to make u easier to read
		kp = 20 # proportional gain
		kd = 0 # derivative gain
		ePrev = 0 # previous error
		e = 0 # current error
		fd = 50 # desired Z force

		print("digging...")

		# get in digging position:
		self.pos[1:] = copy.copy(self.digPos2[1:]) # copying first elt of self.digPos is unsafe
		oldRate = self.updateRate
		self.updateRate = 3
		self.publishStates()
		time.sleep(self.updateRate)
		self.updateRate = oldRate
		self.housekeeping()

		time.sleep(0.5)

		# find ground
		while (abs(self.zForce) < 20):
			self.perturbZ(-1)
			self.publishStates()
			self.updateStates()
			time.sleep(0.01)
			self.rLog.append(self.dist)
			self.zLog.append(self.z)

		time.sleep(0.5)
		oldZPert = self.zPert
		oldRate = self.updateRate
		self.updateRate = 0.001
		sumForce = 0
		counter = 0

		# dig radially inward and use PID to control height
		while (self.dist > 0.3):
			e = self.zForce - fd
			u = ff_gain*(kp*e + kd*(e - ePrev))
			if u > 0.0075:
				u = 0.2
			elif u < -0.0075:
				u = -0.2
			ePrev = e
			print("e: ", e)
			print("u: ", u)
			print("self.zForce: ", self.zForce)
			print("-------------------------------")
			sumForce += self.zForce
			counter += 1
			self.zPert = u

			# move in z
			self.perturbZ(1)
			self.perturbR(-1)
			self.publishStates()
			self.updateStates()
			#time.sleep(self.updateRate)

			"""# move in r separately
			self.perturbR(-1)
			self.publishStates()
			self.updateStates()
			time.sleep(self.updateRate)"""

			time.sleep(0.01)
			print(self.dist)
			self.rLog.append(self.dist)
			self.zLog.append(self.z)

		time.sleep(0.5)
		self.zPert = oldZPert
		self.updateRate = oldRate
		avgForce = sumForce/counter
		print("Average zForce: ", avgForce)

		# tilt bucket 
		while (self.pos[3] > 3.14):
			self.perturbPhi(1)
			self.publishStates()
			self.updateStates()
			time.sleep(0.01)
			self.rLog.append(self.dist)
			self.zLog.append(self.z)

		time.sleep(0.5)

		# go up
		while (self.z < 0):
			self.perturbZ(1)
			self.publishStates()
			self.updateStates()
			time.sleep(0.01)
			self.rLog.append(self.dist)
			self.zLog.append(self.z)

		print("done")

if __name__ == '__main__':

	# instantiate publisher node
	node = CommandPublisher()
