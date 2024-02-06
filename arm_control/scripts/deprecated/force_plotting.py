#!/usr/bin/env python3

import rospy
import math
import time
import random
import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from anakin_control.msg import ForceMsg
from threading import Thread


def init():

	global times, rForce, zForce, rMovingAvg, zMovingAvg, fig1, fig2, ax1, ax2
	times, rForce, zForce, zMovingAvg, rMovingAvg = [0.0], [0.0], [0.0], [0.0], [0.0]
	fig1, ax1 = plt.subplots()
	fig2, ax2 = plt.subplots()
	# ax.plot(times, rForce)
	# ax.plot(times, zForce)
	ax1.plot(times, rMovingAvg)
	ax1.plot(times, zMovingAvg)
	ax2.plot(times, rForce)
	ax2.plot(times, zForce)
	ani1 = animation.FuncAnimation(fig1, animate1, interval=100)
	ani2 = animation.FuncAnimation(fig2, animate2, interval=100)
	plt.show()

def getData():

	rospy.init_node('ForceDisplay', anonymous=True)
	rospy.Subscriber('rz_forces', ForceMsg, addData)
	# rospy.Subscriber('joint_states', JointState, addJointStates)

	while not rospy.is_shutdown():
		global times, rForce, zForce, rMovingAvg, zMovingAvg, fig1, fig2, ax1, ax2
		times, rForce, zForce, zMovingAvg, rMovingAvg = [0.0], [0.0], [0.0], [0.0], [0.0]
		fig1, ax1 = plt.subplots()
		fig2, ax2 = plt.subplots()
		# ax.plot(times, rForce)
		# ax.plot(times, zForce)
		ax1.plot(times, rMovingAvg)
		ax1.plot(times, zMovingAvg)
		ax2.plot(times, rForce)
		ax2.plot(times, zForce)
		ani1 = animation.FuncAnimation(fig1, animate1, interval=100)
		ani2 = animation.FuncAnimation(fig2, animate2, interval=100)
		plt.show()

		rospy.spin()

def addData(data):

	global times, rForce, zForce
	times.append(data.timestamp)
	rForce.append(data.r)
	zForce.append(data.z)
	movingAverage()

def addJointStates(data):

	global times, rForce, zForce
	times.append(rospy.get_time())
	rForce.append(data.effort[1])
	zForce.append(data.effort[2])
	movingAverage()

def animate1(i):

	global ax, times, rForce, zForce
	ax1.clear()
	# ax.plot(times, rForce)
	# ax.plot(times, zForce)
	ax1.plot(times, rMovingAvg)
	ax1.plot(times, zMovingAvg)
	ax1.legend(['r', 'z'])

def animate2(i):

	global ax, times, rForce, zForce
	window = 1000
	ax2.clear()
	if len(times) > window:
		ax2.plot(times[-1*window:], rForce[-1*window:])
		ax2.plot(times[-1*window:], zForce[-1*window:])
	else:
		ax2.plot(times, rForce)
		ax2.plot(times, zForce)
	ax2.legend(['r', 'z'])

def movingAverage():

	global times, rForce, zForce, rMovingAvg, zMovingAvg
	window = 50
	th = 1000

	if len(times) <= window:
		rMovingAvg.append(0.0)
		zMovingAvg.append(0.0)
	else:
		rAvg = np.mean(rForce[-1*window:])
		zAvg = np.mean(zForce[-1*window:])
		if rAvg < -1*th: rAvg = -1*th
		if rAvg > th: rAvg = th
		if zAvg < -1*th: zAvg = -1*th
		if zAvg > th: zAvg = th
		rMovingAvg.append(rAvg)
		zMovingAvg.append(zAvg)


if __name__ == '__main__':

	#Thread(target=init).start()
	getData()