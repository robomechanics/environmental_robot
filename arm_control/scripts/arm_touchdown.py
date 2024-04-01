#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np
from time import sleep
import hebi
import rospkg
import math
import matplotlib.pyplot as plt
from std_srvs.srv import Trigger, TriggerResponse

class RobotController():
    def __init__(self, family_name, module_names, hrdf_path):
        # HEBI setup
        self.lookup = hebi.Lookup()
        # sleep(2.0)

        self.group = self.lookup.get_group_from_names([family_name], module_names)   #from HEBI API

        if self.group is None:
            rospy.logerr('Group not found')
            raise "Error: Group not found"

        self.model = hebi.robot_model.import_from_hrdf(hrdf_path)

        # Set the gains
        gainsCmd = hebi.GroupCommand(self.group.size)
        rospack = rospkg.RosPack()
        ak = rospack.get_path('arm_control')
        path = ak + '/config/gains/anakin_gains.xml'
        print("Gains path: ", path)
		# path = "/home/rover/catkin_ws/src/anakin_control/config/gains/anakin_gains.xml"
        gainsCmd.read_gains(path)
        self.group.send_command_with_acknowledgement(gainsCmd)

        self.num_joints = self.group.size
        self.group_fbk = hebi.GroupFeedback(self.num_joints)
        #gainsCmd.mstop_strategy = 2
        
        # Register feedback callback 
        # self.group.feedback_frequency = 100.0
        # self.group.add_feedback_handler(self.feedback_func_callback)
    
    #def feedback_func_callback(self, feedback):
        # feedback is guaranteed to be a "hebi.GroupFeedback" instance
    #    self.feedback = feedback

        
    def get_joint_angles(self):
        # Get position feedback from the robot to use as initial conditions 
        group_fbk = hebi.GroupFeedback(self.group.size)
        if self.group.get_next_feedback(reuse_fbk=group_fbk) is None:
            rospy.logerr("Couldn't get feedback")
            exit(1)

        current_joint_angles = group_fbk.position
        return current_joint_angles

    def calculate_ik_solution(self, initial_joint_angles, target_xyz, joint_limit_constraint):
    #def calculate_ik_solution(self, initial_joint_angles, target_xyz):
        ee_pos_objective = hebi.robot_model.endeffector_position_objective(target_xyz)
        return self.model.solve_inverse_kinematics(initial_joint_angles, ee_pos_objective, joint_limit_constraint)
        #return self.model.solve_inverse_kinematics(initial_joint_angles, ee_pos_objective)

    def get_fk_of_ik(self, ik_result_joint_angles):
        return self.model.get_end_effector(ik_result_joint_angles)[0:4, 3]
    
    def get_effort(self):
        self.group_fbk = self.group.get_next_feedback(reuse_fbk = self.group_fbk)
        effort_fbk = self.group_fbk.effort
        # set mstop strategy
        # rospy.loginfo('Effort: {0}'.format(effort_fbk))
      
        return effort_fbk

class EndEffectorTrajectory():
    
    def __init__(self, robot_controller):
        self.home_open = [0.0, 0.0, 0.4]
        # self.homeClosed = [4.75, 0.11, -2.90, -3.38]
        self.robot_controller = robot_controller
        self.target_positions = [
            self.home_open, # go up from home 
            [0.0, 0.2, 0.4], # extend
            [0.0, 0.4, 0.0] # extend and move down
            # [0.0, 0.4, -0.1] 
            # only x,y,z coordinates
        ]
        
        self.min_positions = [4, -2*np.pi, -2*np.pi, -np.pi/2]  # Joint limits for joints
        self.max_positions = [5, 2*np.pi, -1, np.pi/2]
        
        self.group_fbk = hebi.GroupFeedback(robot_controller.group.size)
        
    #     self.raise_arm = rospy.Service('arm/raise', Trigger, self.raise_arm_service_cb)
    #     self.lower_arm = rospy.Service('arm/lower', Trigger, self.lower_arm_service_cb)

    # def raise_arm_service_cb(req):
    #     return TriggerResponse(True, "Success")

    # def lower_arm_service_cb(req):
    #     return TriggerResponse(True, "Success")
    
    def get_trajectory(self, total_duration = 40):

        initial_joint_angles = self.robot_controller.get_joint_angles()
        print(initial_joint_angles)
        ##self.positions = np.zeros((4, len(self.target_positions) + 1 ))

        #initial_joint_angles = []

        ##self.positions[:,0] = initial_joint_angles
        # Loop through the target positions
        
        ##for i, target_xyz in enumerate(self.target_positions):

            #current_joint_angles = self.robot_controller.get_joint_angles()
            #positions[:, 0] = current_joint_angles

            ##joint_limit_constraint = hebi.robot_model.joint_limit_constraint(self.min_positions, 
            ##                                                                 self.max_positions)

            # Calculate IK solution for the current target position
            ##ik_result_joint_angles = self.robot_controller.calculate_ik_solution(self.positions[:, i], 
            ##                                                                     target_xyz, 
            ##                                                                     joint_limit_constraint)

            ##self.positions[:, i + 1] = np.array(ik_result_joint_angles)

            #angle_sum = 0
            #for j in self.positions[1:-1, i+1]:
            #    angle_sum += j 

            #k = - 1.25*np.pi

            #self.positions[3, i+1] = k + self.positions[1, i+1] - self.positions[2, i+1]
            # print(positions[3, i+1])
        

            ##rospy.loginfo('Target position: {0}'.format(target_xyz))
            ##rospy.loginfo('IK joint angles: {0}'.format(ik_result_joint_angles))
            ##rospy.loginfo('FK of IK: {0}'.format(self.robot_controller.get_fk_of_ik(ik_result_joint_angles)))
            ##rospy.loginfo('Positions: {0}'.format(self.positions))
            
        # self.positions = np.array([
        #      initial_joint_angles,
        #     [ 4.73091793,  4.89363516,  4.77573206,  4.74393144],
        #     [-0.02057076,  0.46017654,  1.16832948, -3.78152561],
        #     [-3.02600002, -2.18277832, -1.97059152, -1.77164562],
        #     [-2.28942299, -1.28403595, -0.78806982, -5.93687081]
        # ])
        
        positions = self.positions.T

        self.num_joints = len(initial_joint_angles)

        # Position, velocity, and acceleration waypoints
        self.pos = np.full(positions.shape, np.nan)
        self.vel = np.full(positions.shape, np.nan)
        self.acc = np.full(positions.shape, np.nan)

        # Set first and last waypoint values to 0.0
        self.vel[:, 0] = self.acc[:, 0] = 0.0
        self.vel[:, -1] = self.acc[:, -1] = 0.0

        # set time
        self.times = np.linspace(0, total_duration, positions.shape[1])

        # Create the trajectory
        # trajectory = hebi.trajectory.create_trajectory(time, pos, vel, acc)
        self.trajectory = hebi.trajectory.create_trajectory(self.times, np.copy(positions), self.vel, self.acc)
        
        #print("----------------\n\n")

    def execute_trajectory(self):

        #initial_joint_angles = self.robot_controller.get_joint_angles()
        #self.num_joints = len(initial_joint_angles)
        
        hz = float(20)
        rate = rospy.Rate(hz)

        # Follow the overall trajectory
        cmd = hebi.GroupCommand(self.num_joints)
        
        period = 0.01
        print("Period:", period)

        t = 0.0
        duration = self.trajectory.duration
        while (t < duration):
            # self.robot_controller.group.get_next_feedback(reuse_fbk=self.group_fbk)
            self.pos_cmd, self.vel_cmd, self.acc_cmd = self.trajectory.get_state(t)

            cmd.position = self.pos_cmd
            cmd.velocity = self.vel_cmd
            self.robot_controller.group.send_command(cmd)

            #effort_array = np.zeros(num_joints, dtype=np.float32)
            #self.robot_controller.get_effort(effort_array)

            #rospy.loginfo('Effort: {0}'.format(effort_array))
            
            # print("Sending command at time = ", t)
            # print("Current Trajectory State:", self.trajectory.get_state(t))
            # print("Joint angles: ", self.robot_controller.get_joint_angles())
            # print("---")
                

            t = t + period
            
            sleep(period)
            
        print("-----------")
        print("Sending command at time = ", t)
        print("Current Trajectory State:", self.trajectory.get_state(t))
        print("Joint angles: ", self.robot_controller.get_joint_angles())

        repeat_delay = 5
        rospy.loginfo('Staying in the last position for {0} seconds...'.format(repeat_delay))
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < repeat_delay:
            self.robot_controller.group.send_command(cmd)
            rate.sleep()
           


if __name__ == '__main__':

    #reverse_target_positions = target_positions[::-1][2:]
    
    # Initialize the ROS node
    rospy.init_node('robot_control_node')

    # ROS Publisher for joint commands
    # joint_command_publisher = rospy.Publisher('/robot/joint_commands', JointState, queue_size=10)

    # Wait for 2 seconds to allow time for the ROS system to initialize
    # sleep(2.0)

    # Create instances of the classes
    robot_controller = RobotController("Arm_RML", ["J1_base", "J2_shoulder", "J3_elbow", 'J4_wrist'], "/home/patrick/catkin_ws/src/anakin_control/config/hrdf/anakin.hrdf")
    end_effector_trajectory = EndEffectorTrajectory(robot_controller)

    end_effector_trajectory.get_trajectory()
    
    end_effector_trajectory.execute_trajectory()
  

    rospy.loginfo("Script completed. Shutting down the ROS node.")