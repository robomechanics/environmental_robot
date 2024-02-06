#!/usr/bin/env python3

import rospy
import hebi
import math
import time
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from anakin_control.msg import ForceMsg
from pathlib import Path
# from scipy.linalg import expm
from robotDimensions2D import *

class ForceTransformer:

	def __init__(self):

		# initialize node
		rospy.init_node('ForceTransformer', anonymous=True)

		# initialize publisher
		self.forcePub = rospy.Publisher('rz_forces', ForceMsg, queue_size=10)

		# set kinematics model
		# path = Path('/home/nick/Desktop/cmu/research/robomechanics/chevron/'\
			# 'hebi_ws/src/anakin_control/config/hrdf/anakin.hrdf') # ---- simulation
		path = Path('/home/rover/catkin_ws/src/anakin_control/config/hrdf/anakin.hrdf') # ---- real robot
		self.arm = hebi.robot_model.import_from_hrdf(path)
		self.forces = np.array([0.0, 0.0])
		self.window = 50
		self.recentR = list([0]*self.window)
		self.recentZ = list([0]*self.window)
		self.counter = 0
		self.time = 0.0

		"""# define robot dimensions
		rd = robotDimensions2D()
		self.l = rd[0]
		self.le = rd[1]
		self.dy = rd[2]
		self.dl = rd[3]
		self.d = rd[4]
		self.de = rd[5]
		self.dz = rd[6]		

		# define reference configuration (spatial to tool frame)
		DX = 2*self.l + self.le 
		DY = self.dy - self.dl - self.d - self.de 
		DZ = self.dz
		self.gst0 = np.array([[1, 0, 0, DX], [0, 0, -1, DY], [0, 1, 0, DZ], [0, 0, 0, 1]])

		# define body Jacobain components (xi_i)
		w1 = np.array([0, 0, 1])
		w2 = np.array([0, 1, 0])
		w3 = np.array([0, -1, 0])
		w4 = np.array([0, -1, 0])
		
		p1 = np.array([0, 0, 0])
		p2 = np.array([0, 0, self.dz])
		p3 = np.array([l, 0, self.dz])
		p4 = np.array([2*self.l, 0, self.dz])

		v1 = np.cross(-1*w1, p1)
		v2 = np.cross(-1*w2, p2)
		v3 = np.cross(-1*w3, p3)
		v4 = np.cross(-1*w4, p4)

		self.xi1 = self.getXiHat(w1, v1)
		self.xi2 = self.getXiHat(w2, v2)
		self.xi3 = self.getXiHat(w3, v3)
		self.xi4 = self.getXiHat(w4, v4)"""

		# initialize subscriber
		rospy.Subscriber('hebi_joint_states', JointState, self.forces2D, queue_size=1) # use 3D when end-effector in hrdf works
		# loop de doop
		rospy.spin()


	def timingTest(self, data):
		print("time since last call: ", time.time() - self.time)
		self.time = time.time()
		return

	def printJointStates(self, data):
		start = time.time()
		efforts = data.effort
		positions = data.position
		#print("positions: ", positions)
		#print("efforts: ", efforts)
		print("block 0: ", time.time() - start)

	def forces2D(self, data):
		# model as a 3 dof arm in r-z plane

		# start = time.time()
		efforts = data.effort
		positions = data.position
		# print("efforts: ", efforts)
		# print("positions: ", positions)
		# print("block 1: ", time.time() - start)

		# start = time.time()
		L1 = 0.325 # [m]
		L2 = 0.325 # [m]
		L3 = 0.116 # [m]
		T1 = math.pi - positions[1] # J2 irl
		T2 = positions[2] # J3 irl
		T3 = positions[3] # J4 irl
		E1 = -efforts[1] # J2 irl
		E2 = efforts[2] # J3 irl
		E3 = efforts[3] # J4 irl
		# print("block 2: ", time.time() - start)

		# print("effort data: ", np.array([[E1],[E2],[E3]]))

		# start = time.time()		
		# define trig shorthand
		s1, s12, s123 = math.sin(T1), math.sin(T1+T2), math.sin(T1+T2+T3)
		c1, c12, c123 = math.cos(T1), math.cos(T1+T2), math.cos(T1+T2+T3)
		# print("block 3: ", time.time() - start)

		# start = time.time()
		# link/joint masses
		linkMass = 0.402 # [kg]
		jointMass = 0.490 # [kg]
		scoopMass = 0.0675 # [kg]

		# simulatied link/joint/scoop masses
		linkMass = 0.404 # [kg]
		jointMass = 0.500 # [kg]
		scoopMass = 0.010 # [kg]
		# print("block 4: ", time.time() - start)

		# start = time.time()
		# link/joint weights
		g = -9.81 # [m/s^2]
		linkWeight = np.array([[0], [g*linkMass], [0]])
		jointWeight = np.array([[0], [g*jointMass], [0]])
		scoopWeight = np.array([[0], [g*scoopMass], [0]])
		# print("block 5: ", time.time() - start)

		# start = time.time()	
		# Jacobian to L1 COM
		JL1 = np.array([[-(L1/2)*s1,0,0], [(L1/2)*c1,0,0], [1,0,0]])

		# Jacobain to J2
		JJ2 = np.array([[-L1*s1,0,0], [L1*c1,0,0], [1,0,0]])

		# Jacobian to L2 COM
		JL2 = np.array([[-L1*s1 - (L2/2)*s12, -(L2/2)*s12, 0],
						[L1*c1 + (L2/2)*c12, (L2/2)*c12, 0],
						[1, 1, 0]])

		# Jacobian to J3
		JJ3 = np.array([[-L1*s1 - L2*s12, -L2*s12, 0],
						[L1*c1 + L2*c12, L2*c12, 0],
						[1, 1, 0]])

		# Jacobian to L3 COM
		JL3 = np.array([[-L1*s1 - L2*s12 - (L3/2)*s123, -L2*s12 - (L3/2)*s123, -(L3/2)*s123],
						[L1*c1 + L2*c12 + (L3/2)*c123, L2*c12 + (L3/2)*c123, (L3/2)*c123],
						[1, 1, 1]])

		# Jacobian to EE
		JEE = np.array([[-L1*s1 - L2*s12 - L3*s123, -L2*s12 - L3*s123, -L3*s123],
						[L1*c1 + L2*c12 + L3*c123, L2*c12 + L3*c123, L3*c123],
						[1, 1, 1]])
		# print("block 6: ", time.time() - start)

		# start = time.time()
		# torques to compensate gravity for L1, J2, L2, J3, L3/scoop)
		TL1 = np.matmul(JL1.transpose(),linkWeight)
		TJ2 = np.matmul(JJ2.transpose(),jointWeight)
		TL2 = np.matmul(JL2.transpose(),linkWeight)
		TJ3 = np.matmul(JJ3.transpose(),jointWeight)
		TL3 = np.matmul(JL3.transpose(),scoopWeight)
		# print("block 7: ", time.time() - start)

		"""print("TL1: ", TL1)
		print("TJ2: ", TJ2)
		print("TL2: ", TL2)
		print("TJ3: ", TJ3)
		print("TL3: ", TL3)"""

		# start = time.time()
		# effective torques
		gravCompTorques = TL1 + TJ2 + TL2 + TJ3 + TL3 
		# effectiveTorques = np.array([[E1],[E2],[E3]]) - TL1 - TJ2 - TL2 - TJ3 - TL3
		effectiveTorques = np.array([[E1],[E2],[E3]]) + gravCompTorques
		# print("block 8: ", time.time() - start)
		
		# print("effectiveTorques: ", effectiveTorques)
		
		# start = time.time()
		self.forces = np.matmul(np.linalg.inv(JEE),effectiveTorques)
		# print("time since last call: ", rospy.get_time() - self.time)
		self.time = rospy.get_time()
		# print("block 9: ", time.time() - start)

		# print("self.forces: ", self.forces)

		# keep track of moving avg instead of raw forces
		
		"""if self.counter < self.window:
			self.recentR[self.counter] = self.forces[0]
			self.recentZ[self.counter] = self.forces[1]
			self.counter += 1
		else:
			self.recentR.pop(0)
			self.recentZ.pop(0)
			self.recentR.append(self.forces[0])
			self.recentZ.append(self.forces[1])
		rAvg = np.mean(self.recentR)
		zAvg = np.mean(self.recentZ)"""

		# print("rAvg: ", rAvg)
		# print("zAvg: ", zAvg)

		# send to force topic
		# rz0 = ForceMsg(self.time, -rAvg, -zAvg)

		# start = time.time()
		rz0 = ForceMsg(self.time, self.forces[0], self.forces[1])
		rospy.logwarn("r: %f", self.forces[0])
		rospy.logwarn("z: %f", self.forces[1])
		"""pythonTime = time.time()
		print("elapsed: ", pythonTime - start)
		print("python time: ", pythonTime)
		print("ros time: ", self.time)
		print("time offset: ", self.time - pythonTime)
		print("-----------------------------------------------")"""
		self.forcePub.publish(rz0)
		# print("block 10: ", time.time() - start)


	def forces3D(self, data):
		# full kinematic model of the arm including scoop
		efforts = data.effort
		positions = data.position
		q1 = positions[0]
		q2 = positions[1]
		q3 = positions[2]
		q4 = positions[3]
		c1, c2, c3, c4 = math.cos(q1), math.cos(q2), math.cos(q3), math.cos(q4)
		s1, s2, s3, s4 = math.sin(q1), math.sin(q2), math.sin(q3), math.sin(q4)

		# define gst_inv in terms of q_i
		sig1 = math.sin(q3-q2+q4)
		sig2 = math.cos(q3-q2+q4)
		g03 = -(27/500)*sig1 - (13/40)*math.cos(q3+q4) - (13/40)*c4 - (29/250)
		g13 = (13/40)*math.sin(q3+q4) - (27/500)*sig2 + (13/40)*s4
		gst_inv = np.array([[sig2*c1, sig2*s1, sig1, g03],
							[-sig1*c1, -sig1*s1, sig2, g13],
							[s1, -c1, 0, -187/2000],
							[0, 0, 0, 1]])

		# define dg1 in terms of q_i
		a, b, c = 187/2000, 13/40, 29/250
		dg1_03 = a*c1 - b*c2*s1 - b*s1*s2*s3 \
				 - b*c2*c3*s1 - c*c2*c3*c4*s1 \
				 + c*c2*s1*s3*s4 \
				 - c*c3*s1*s2*s4 \
				 - c*c4*s1*s2*s3
		dg1_13 = a*s1 + b*c1*c2 + b*c1*c2*c3 \
				 + b*s2*s3*c1 + c*c2*c3*c4*c1 \
				 - c*c1*c2*s3*s4 \
				 + c*c1*s2*c3*s4 \
				 + c*c1*s2*s3*c4
		dg1 = np.array([[-sig2*s1, sig1*s1, c1, dg1_03],
						[sig2*c1, -sig1*math.cos(1), s1, dg1_13], 
						[0, 0, 0, 0],
						[0, 0, 0, 0]])

		# define dg2 in terms of q_i
		sig1 = math.cos(q3-q2+q4)
		sig3 = math.sin(q3-q2+q4)
		sig2 = 325*math.sin(q2-q3) - 116*sig3 + 325*s2
		dg2 = np.array([[sig3*c1, sig1*c1, 0, -(1/1000)*sig2*c1],
					    [sig3*s1, sig1*s1, 0, -(1/1000)*sig2*s1],
					    [-sig1, sig3, 0, -(13/40)*math.cos(q2-q3) - (29/250)*sig1 - (13/40)*c2],
					    [0, 0, 0, 0]])

		# define dg3 in terms of q_i
		sig1 = math.cos(q3-q2+q4)
		sig2 = math.sin(q3-q2+q4)
		a, b = 29/500, 13/80
		dg3_03 = a*math.sin(q1+q2-q3-q4) + b*math.sin(q1+q2-q3) - b*math.sin(q1-q2+q3) - a*math.sin(q1-q2+q3+q4)
		dg3_13 = b*math.cos(q1-q2+q3) - b*math.cos(q1+q2-q3) - a*math.cos(q1+q2-q3-q4) + a*math.cos(q1-q2+q3+q4)
		dg3 = np.array([[-sig2*c1, -sig1*c1, 0, dg3_03],
						[-sig2*s1, -sig1*s1, 0, dg3_13],
						[sig1, -sig2, 0, (13/40)*math.cos(q2-q3) + (29/250)*sig1],
						[0, 0, 0, 0]])

		# define dg4 in terms of q_i
		sig1 = math.cos(q3-q2+q4)
		sig2 = math.sin(q3-q2+q4)
		a = 29/500
		dg4_03 = a*math.sin(q1+q2-q3-q4) - a*math.sin(q1-q2+q3+q4)
		dg4_13 = a*math.cos(q1-q2+q3+q4) - a*math.cos(q1+q2-q3-q4)
		dg4 = np.array([[-sig2*c1, -sig1*c1, 0, dg4_03],
						[-sig2*s1, -sig1*s1, 0, dg4_13],
						[sig1, -sig2, 0, (29/250)*sig1],
						[0, 0, 0, 0]])

		# assemble body Jacobian with twists
		col0 = self.rbvel2twist(gst_inv*dg1)
		col1 = self.rbvel2twist(gst_inv*dg2)
		col2 = self.rbvel2twist(gst_inv*dg3)
		col3 = self.rbvel2twist(gst_inv*dg4)

		# T = J_b^T * F
		J = np.array([col0, col1, col2, col3])
		JT = np.linalg.pinv(J)
		

		
		# look into momentum observer


	def angvel2skew(self, w):
		# w must be a 3-vector
		return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])

	def skew2angvel(self, wHat):
		# wHat must be a 3x3 skew symmetric matrix
		return np.array([[wHat[2,1]], [wHat[0,2]], [wHat[1,0]]])

	def rbvel2twist(self, xiHat):
		# hiHat must be a 4x4 matrix
		w = self.skew2angvel(xiHat[0:3,0:3])
		# v = xiHat([0:3,3])
		return np.array([[v[0]], [v[1]], [v[2]], [w[0]], [w[1]], [w[2]]])


	def getXiHat(self, w, v):
		# [v,w]^T must be a 6x1 twist
		wHat = self.angvel2skew(w)
		return np.array([[wHat[0,0], wHat[0,1], wHat[0,2], v[0]],
						  [wHat[1,0], wHat[1,1], wHat[1,2], v[1]],
						  [wHat[2,0], wHat[2,1], wHat[2,2], v[2]],
						  [0, 0, 0, 0]])




if __name__ == '__main__':
	node  = ForceTransformer()
