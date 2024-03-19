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
        #rospy.loginfo('Effort: {0}'.format(effort_fbk))  
        return effort_fbk
    
    def set_arm_to_hold(self):
        self.group.command_lifetime = 0
        self.send() 
        
    def release_arm_hold(self):
        self.group.command_lifetime = 100 
        self.update() 

class EndEffectorTrajectory():

    def __init__(self, robot_controller):
        self.robot_controller = robot_controller
      
    def get_trajectory(self, total_duration = 40):

        initial_joint_angles = robot_controller.get_joint_angles()
        
        self.positions = np.array([
             robot_controller.get_joint_angles(), 
            [ 4.7661829,   2.58495361, -1.95124054, -0.92874146],
            [ 4.83683014,  1.60074646, -2.15341663, -0.23106194],
            [ 4.89430189,  0.72055084, -2.27849388, -0.22733116],
            [ 4.79459,     0.39897567, -2.67670345, -0.23143387],
         ])
        
        positions = self.positions.T
        
        print("\n---------\n Transposed Positions: \n", positions, "\n---------\n")

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
        self.trajectory = hebi.trajectory.create_trajectory(self.times, np.copy(positions), self.vel, self.acc)
        
        

    def execute_trajectory(self):
        
        hz = float(20)
        rate = rospy.Rate(hz)

        # Follow the overall trajectory
        cmd = hebi.GroupCommand(self.num_joints)
        
        period = 0.01
        t = 0.0
        duration = self.trajectory.duration

        while (t < duration):
            # self.robot_controller.group.get_next_feedback(reuse_fbk=self.group_fbk)
            self.pos_cmd, self.vel_cmd, self.acc_cmd = self.trajectory.get_state(t)

            cmd.position = self.pos_cmd
            cmd.velocity = self.vel_cmd

            self.robot_controller.group.send_command(cmd)
                
            t = t + period           
            sleep(period)
            

def execute_trajectory_server(req):
    EndEffectorTrajectory.execute_trajectory()
    return TriggerResponse(success=True, message="Trajectory executed successfully")
           

if __name__ == '__main__':
    
    # Initialize the ROS node
    rospy.init_node('robot_control_node')

    # ROS Publisher for joint commands
    # joint_command_publisher = rospy.Publisher('/robot/joint_commands', JointState, queue_size=10)

    # Wait for 2 seconds to allow time for the ROS system to initialize
    # sleep(2.0)

    # Create instances of the classes
    robot_controller = RobotController("Arm_RML", ["J1_base", "J2_shoulder", "J3_elbow", 'J4_wrist'], "/home/patrick/catkin_ws/src/enviromental_robot/arm_control/config/hrdf/anakin.hrdf")
    end_effector_trajectory = EndEffectorTrajectory(robot_controller)

    end_effector_trajectory.get_trajectory()
    
    execute_service = rospy.Service('execute_trajectory', Trigger, execute_trajectory_server)
    #end_effector_trajectory.execute_trajectory()
    rospy.spin()

    rospy.loginfo("Script completed. Shutting down the ROS node.")