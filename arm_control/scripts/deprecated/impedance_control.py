import time
import hebi
import math
import numpy as np
from robotDimensions2D import *

class ImpedanceController:

	def __init__(self):

		# on real robot, use direct fbk from sensors
		lookup = hebi.Lookup()
		family = ['Arm_RML']
		names = 0 # define later

		rd = robotDimensions2D()
		self.l = rd[0]
		self.le = rd[1]
		self.dy = rd[2]
		self.dl = rd[3]
		self.d = rd[4]
		self.de = rd[5]
		self.dz = rd[6]		

		self.armGroup = lookup.get_group_from_names(family, names)
		self.freq = 200.0
		self.armGroup.feedback_frequency = self.freq # Hz; ROS loop runs at ~ 200 Hz in sumulation
		self.fbk = self.armGroup.get_next_feedback(timeout_ms=10000)

	def digWithImpedanceControl(self):

		# PHASE 1 --- CONTACT GROUND

		# PHASE 2 --- MAINTAIN CONTACT, CONTROL WITH POSITION AND FORCE

		# generate a cartesian position trajectory
		numPoints = 100
		radius = self.le
		circleAngle = np.linspace(0, -math.pi, numPoints)
		rPosTraj = [0]*numPoints
		zPosTraj = [0]*numPoints
		for i in range(numPoints):
			rPosTraj[i] = radius*math.cos(circleAngle[i])
			zPosTraj[i] = radius*math.sin(circleAngle[i])

		# generate a joint position trajectory
		xTraj = np.zeros(shape=([4,numPoints]))
		xTraj[0] = [self.fbk.position[0]]*numPoints
		xtraj[1] = [self.fbk.position[1]]*numPoints
		xTraj[2] = [self.fbk.position[2]]*numPoints
		xtraj[3] = np.linspace(0, -math.pi, numPoints) + self.fbk.position[4]
			
		# generate a force trajectory
		rForceTraj = [0]*numPoints
		zForceTraj = [0]*numPoints
		ffForceGain = 1 # this scales up difference in position 
		for i in range(1,numPoints-1):
			rForceTraj = (rPosTraj[i-1] - rPosTraj[i+1])*ffForceGain
			zForceTraj = (zPosTraj[i-1] - rPosTraj[i+1])*ffForceGain


		n = len(rPosTraj)
		for i in range(n):

			# get sensor readings
			self.fbk = self.armGroup.get_next_feedback()
			pos = self.fbk.position
			dx = self.fbk.velocity
			ddx = self.fbk.acceleration
			eff = self.fbk.effort

			# forward kinematics for position estimate
			fk = self.fk2d(pos)
			r,z = fk[0], fk[1]

			# must have len(rPosTraj) == len(rForceTraj)
			xd = posTraj[i]
			Fd = effTraj[i]

			# change these later
			Kf = np.zeros((3,3)) # force gain matrix
			Kd = np.zeros((3,3)) # desired stiffness matrix
			Bd = np.zeros((3,3)) # desired damping matrix
			Md = np.zeros((3,3)) # desired mass matrix

			JT = np.zeros((3,4)) # jacobain transpose

			ddx = (xd-x)*Kd + (Fd-Fe)*Kf + (dxd-dx)*Bd
			F = np.matmul(np.linalg.inv(Md),ddx)
			cmdEffort = JT*F 

	def fk2d(self, pos):
		# computes planar forward kinematics

		q1 = pos[0]
		q2 = pos[1]
		q3 = pos[3]

		r = self.l*math.cos(q1) + self.l*math.cos(q2) + self.le*math.cos(q3)
		z = self.l*math.sin(q1) + self.l*math.sin(q2) + self.le*math.sin(q3)
		
		return (r,z)
