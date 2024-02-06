#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np
from time import sleep
import hebi
import rospkg
import math

class RobotController:
    def __init__(self, family_name, module_names, hrdf_path):
        # HEBI setup
        self.lookup = hebi.Lookup()
        sleep(2.0)

        self.group = self.lookup.get_group_from_names([family_name], module_names)   #from HEBI API

        if self.group is None:
            rospy.logerr('Group not found')
            exit(1)

        self.model = hebi.robot_model.import_from_hrdf(hrdf_path)

        # Set the gains
        gainsCmd = hebi.GroupCommand(self.group.size)
        rospack = rospkg.RosPack()
        ak = rospack.get_path('anakin_control')
        path = ak + '/config/gains/anakin_gains.xml'
		# path = "/home/rover/catkin_ws/src/anakin_control/config/gains/anakin_gains.xml"
        gainsCmd.read_gains(path)
        self.group.send_command_with_acknowledgement(gainsCmd)

    def get_initial_joint_angles(self):
        # Get position feedback from the robot to use as initial conditions 
        group_fbk = hebi.GroupFeedback(self.group.size)
        if self.group.get_next_feedback(reuse_fbk=group_fbk) is None:
            rospy.logerr("Couldn't get feedback")
            exit(1)

        return group_fbk.position

    #def calculate_ik_solution(self, initial_joint_angles, target_xyz, joint_limit_constraint):
    def calculate_ik_solution(self, initial_joint_angles, target_xyz):
        ee_pos_objective = hebi.robot_model.endeffector_position_objective(target_xyz)
        #return self.model.solve_inverse_kinematics(initial_joint_angles, ee_pos_objective, joint_limit_constraint)
        return self.model.solve_inverse_kinematics(initial_joint_angles, ee_pos_objective)

    def get_fk_of_ik(self, ik_result_joint_angles):
        return self.model.get_end_effector(ik_result_joint_angles)[0:4, 3]


class EndEffectorTrajectory:
    def __init__(self, robot_controller):
        self.robot_controller = robot_controller
        self.target_positions = [
            [0.0, 0.0, 0.4]
            # only x,y,z coordinates
            # can write in series till we get where we want
        ]

    def execute_trajectory(self):
        # Loop through the target positions
        positions = np.zeros((4,2))

        #min_positions = [-1.0, -1.0, -1.0, 0]  # Joint limits for joints
        #max_positions = [1.0, 1.0, 1.0, 0]

        #initial_joint_angles = []
        positions[:,0] = self.robot_controller.get_initial_joint_angles()

        for i, target_xyz in enumerate(self.target_positions):
            initial_joint_angles = self.robot_controller.get_initial_joint_angles()

            #joint_limit_constraint = hebi.robot_model.joint_limit_constraint(min_positions[i], max_positions[i])

            # Calculate IK solution for the current target position
            #ik_result_joint_angles = self.robot_controller.calculate_ik_solution(initial_joint_angles, target_xyz, joint_limit_constraint)
            ik_result_joint_angles = self.robot_controller.calculate_ik_solution(initial_joint_angles, target_xyz)
        
            positions[:,i+1]=np.array(ik_result_joint_angles)

            rospy.loginfo('Target position: {0}'.format(target_xyz))
            rospy.loginfo('IK joint angles: {0}'.format(ik_result_joint_angles))
            #rospy.loginfo('FK of IK: {0}'.format(self.robot_controller.get_fk_of_ik(ik_result_joint_angles)[0:3, 3]))

        num_joints = len(initial_joint_angles)

        # Position, velocity, and acceleration waypoints
        pos = np.empty((num_joints, len(positions)))
        vel = np.empty((num_joints, len(positions)))
        acc = np.empty((num_joints, len(positions)))

        # Set first and last waypoint values to 0.0
        vel[:, 0] = acc[:, 0] = 0.0
        vel[:, -1] = acc[:, -1] = 0.0

        # set time
        time = [0, 10]

        # Create the trajectory
        # trajectory = hebi.trajectory.create_trajectory(time, pos, vel, acc)
        trajectory = hebi.trajectory.create_trajectory(time, positions)

        hz = float(20)
        rate = rospy.Rate(hz)

        # Follow the overall trajectory
        cmd = hebi.GroupCommand(num_joints)
        period =   1.0/hz

        t = 0.0
        while t < time[-1]:
            pos, vel, acc = trajectory.get_state(t)
            cmd.position = pos
            cmd.velocity = vel
            self.robot_controller.group.send_command(cmd)

            t += period
            rate.sleep()

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('robot_control_node')

    # ROS Publisher for joint commands
    joint_command_publisher = rospy.Publisher('/robot/joint_commands', JointState, queue_size=10)

    # Wait for 2 seconds to allow time for the ROS system to initialize
    sleep(2.0)

    # Create instances of the classes
    robot_controller = RobotController("Arm_RML", ["J1_base", "J2_shoulder", "J3_elbow", 'J4_wrist'], "/home/patrick/catkin_ws/src/anakin_control/config/hrdf/anakin.hrdf")
    end_effector_trajectory = EndEffectorTrajectory(robot_controller)

    # Execute the end-effector trajectory
    end_effector_trajectory.execute_trajectory()

    rospy.loginfo("Script completed. Shutting down the ROS node.")