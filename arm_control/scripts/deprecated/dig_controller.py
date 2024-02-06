#!/usr/bin/env python3

import rospy
import time
import math
import hebi
import copy
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point
from anakin_control.msg import ForceMsg
from pathlib import Path
from trajectories import workspaceTrajectory

class DigNode:

	def __init__(self):

		rospy.init_node('DigNode', anonymous=True)

		rospy.Subscriber('joy', Joy, self.joystickUpdater)
		rospy.Subscriber('joint_states', JointState, self.joystickHandler)