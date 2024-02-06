#!/usr/bin/env python3

import time
import rospy

from std_msgs.msg import Bool

class Talker:

	def __init__(self):
			
		rospy.init_node('Talker', anonymous=True)
		pub = rospy.Publisher('talker', Bool, queue_size=10)

		updateRate = 0.01
		while not rospy.is_shutdown():
			pub.publish(Bool(True))
			time.sleep(updateRate)

if __name__ == '__main__':
	node = Talker()
