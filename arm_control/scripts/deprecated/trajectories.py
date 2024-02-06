#!/usr/bin/env python3

import hebi
import math
import numpy as np


def workspaceTrajectory(startPos, arm, trajectoryShape, depth):
	# packages trajectory of specified shape into a trajectory object in Hebi API
	# the trajectory object is not necessary for use with the ROS command_node
	# using the trajectory object allows this function to run without ROS (only the Python API) if necessary

	# let's start with some constants
	# depth = 0.150 # [m]
	length = 0.250 # [m]
	numPoints = 100
	duration = 5 # [s]
	L = 0.325 # [m] - link length between J3 and J4
	off = 0.0535 # [m] - offset of ee from origin
	numDOF = 4
	zeroPhi = -3.0

	# start points
	transforms = arm.get_forward_kinematics('output', startPos)
	startXYZ = transforms[6]
	x1, y1, z1 = startXYZ[0][3], startXYZ[1][3], startXYZ[2][3]

	# find unit vector
	offsetOriginX = off*math.cos(startPos[0])
	offsetOriginY = off*math.sin(startPos[0])
	dist = math.sqrt((x1 - offsetOriginX)**2 + (y1 - offsetOriginY)**2)
	r = np.array([x1 - offsetOriginX, y1 - offsetOriginY])
	unitR = -1*r/dist

	# unitR is a unit vector
	dx, dy = unitR[0]*length, unitR[1]*length

	# end points
	x2, y2, z2 = x1 + dx, y1 + dy, z1

	# vector along horizontal length of trajectory
	coords = np.linspace(-1*length/2, length/2, numPoints)

	# initialize xyz trajectory
	xyzTrajectory = np.zeros(shape=[3, numPoints])

	# z offset for different curves, create xyz trajectory
	if trajectoryShape == "rectangular":
		# subdivision size
		c = numPoints//5
		# first segment
		xyzTrajectory[0,:c] = np.linspace(x1, x1, c)
		xyzTrajectory[1,:c] = np.linspace(y1, y1, c)
		xyzTrajectory[2,:c] = np.linspace(z1, z1 - depth, c)
		# second segment
		xyzTrajectory[0,c:4*c] = np.linspace(x1, x2, 3*c)
		xyzTrajectory[1,c:4*c] = np.linspace(y1, y2, 3*c)
		xyzTrajectory[2,c:4*c] = np.linspace(z1 - depth, z1 - depth, 3*c)
		# third segment
		xyzTrajectory[0,4*c:] = np.linspace(x2, x2, c)
		xyzTrajectory[1,4*c:] = np.linspace(y2, y2, c)
		xyzTrajectory[2,4*c:] = np.linspace(z1 - depth, z1, c)
	elif trajectoryShape == "elliptical":
		# heights = -1*depth*(1 - (coords**2)/(length/2)**2)**0.25 # wacky ellipse
		heights = -1*depth*(1 - (coords**2)/(length/2)**2)**0.5 # actual ellipse
		xyzTrajectory[0,:] = np.linspace(x1, x2, numPoints)
		xyzTrajectory[1,:] = np.linspace(y1, y2, numPoints)
		xyzTrajectory[2,:] = heights + z1
	elif trajectoryShape == "parabolic":
		heights = (coords + length/2) * (coords - length/2) * (depth/(length/2)**2)
		xyzTrajectory[0,:] = np.linspace(x1, x2, numPoints)
		xyzTrajectory[1,:] = np.linspace(y1, y2, numPoints)
		xyzTrajectory[2,:] = heights + z1
	else:
		raise Exception("Trjaectory shape invalid; please try again.")

	# convert xyz trajectory to position trajectory through ik
	positions = np.zeros(shape=[numDOF, numPoints])
	initial_joint_angles = np.array(startPos)
	newPos = [0, 0, 0, 0] # initialize once
	for i in range(numPoints):
		target_xyz = xyzTrajectory[:,i]
		end_effector_position_objective = hebi.robot_model.endeffector_position_objective(target_xyz)
		newPos = arm.solve_inverse_kinematics(initial_joint_angles, end_effector_position_objective)
		positions[:,i] = newPos
		initial_joint_angles = newPos

	# adjust phi angle
	for j in range(1, numPoints - 1):
		pos = list(positions[:,j])
		transforms = arm.get_forward_kinematics('output', pos)
		z3 = transforms[4][2][3]
		z4 = transforms[6][2][3]
		deltaZ = z3 - z4
		phi = math.asin(deltaZ/L)
		if trajectoryShape == "rectangular":

			phiOffset = 0.5
		else:
			dh = heights[j + 1] - heights[j - 1]
			dc = coords[j + 1] - coords[j - 1]
			phiOffset = math.atan2(dh, dc)
		positions[3,j] = phi - phiOffset - zeroPhi
		# positions[3, j] = -1*phi + zeroPhi

	# pad the ends
	positions[3,0] = positions[3,1]
	positions[3,-1] = positions[3,-2]

	# set times
	times = np.linspace(0.0, duration, numPoints)

	# set velocities, accelerations to make hebi API happy
	vel = np.empty(shape=[numDOF, numPoints])
	acc = np.empty(shape=[numDOF, numPoints])
	vel[:,0] = acc[:,0] = 0.0
	vel[:,-1] = acc[:,-1] = 0.0
	vel[:,1:-1] = acc[:,1:-1] = np.nan

	trajectory = hebi.trajectory.create_trajectory(times, positions, vel, acc)

	return trajectory