#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Header
from sensor_msgs.msg import Joy, JointState

import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
print(tf.__version__)

def initialize():
	rospy.init_node('showStates', anonymous=True)
	rospy.Subscriber('joint_states', JointState, printStates)
	rospy.spin()

def printStates(data):

	print(data.position[0])
	print(data.position[1])
	print(data.position[2])
	print(data.position[3])
	print("---------------------------------------------------")

if __name__ == '__main__':
	effort0 = keras.models.load_model('effort0')
	effort1 = keras.models.load_model('effort1')
	effort2 = keras.models.load_model('effort2')
	effort3 = keras.models.load_model('effort3')
	initialize()
