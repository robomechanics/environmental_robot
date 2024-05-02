#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult

class DummyMoveBaseActionServer:
    def __init__(self):
        # Initialize the node
        rospy.init_node('dummy_move_base_action_server')
        # Create the action server
        self.server = actionlib.SimpleActionServer('move_base', MoveBaseAction, self.execute, False)
        # Start the action server
        self.server.start()
        rospy.loginfo("Dummy Move Base Action Server has been started.")

    def execute(self, goal):
        rospy.loginfo("A goal has been received.")
        rospy.loginfo("Pretending to move the base to the goal...")
        
        # Wait for a bit to simulate action (not necessary in a real scenario)
        rospy.sleep(2)
        
        # Set the result of the action to success
        result = MoveBaseResult()
        rospy.loginfo("Reached the goal!")
        self.server.set_succeeded(result)

if __name__ == '__main__':
    try:
        # Instantiate the action server
        DummyMoveBaseActionServer()
        # Keep the node alive
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
