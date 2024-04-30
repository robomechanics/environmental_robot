#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import numpy as np
from time import sleep
import hebi
import rospkg
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from os.path import join as os_path_join


class RobotController:

    def __init__(self, family_name, module_names, hrdf_relative_path):
        # HEBI setup

        self.lookup = hebi.Lookup()
        # sleep(2.0)

        self.group = self.lookup.get_group_from_names(
            [family_name], module_names)  # from HEBI API

        if self.group is None:
            rospy.logerr("Group not found")
            raise "Error: Group not found"

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("arm_control")
        hrdf_path = os_path_join(pkg_path, hrdf_relative_path)

        # Load Model
        self.model = hebi.robot_model.import_from_hrdf(hrdf_path)

        # Set the gains
        gainsCmd = hebi.GroupCommand(self.group.size)
        path = os_path_join(pkg_path, "config/gains/anakin_gains.xml")
        # rospy.loginfo("Gains path: ", path)
        # path = "/home/rover/catkin_ws/src/anakin_control/config/gains/anakin_gains.xml"
        gainsCmd.read_gains(path)
        self.group.send_command_with_acknowledgement(gainsCmd)

        self.num_joints = self.group.size
        self.group_fbk = hebi.GroupFeedback(self.num_joints)
        # gainsCmd.mstop_strategy = 2

    def get_joint_angles(self):
        # Get position feedback from the robot to use as initial conditions
        group_fbk = hebi.GroupFeedback(self.group.size)
        if self.group.get_next_feedback(reuse_fbk=group_fbk) is None:
            rospy.logerr("Couldn't get feedback")
            exit(1)

        current_joint_angles = group_fbk.position
        return current_joint_angles

    def calculate_ik_solution(self, initial_joint_angles, target_xyz,
                              joint_limit_constraint):
        # def calculate_ik_solution(self, initial_joint_angles, target_xyz):
        ee_pos_objective = hebi.robot_model.endeffector_position_objective(
            target_xyz)
        return self.model.solve_inverse_kinematics(initial_joint_angles,
                                                   ee_pos_objective,
                                                   joint_limit_constraint)
        # return self.model.solve_inverse_kinematics(initial_joint_angles, ee_pos_objective)

    def get_fk_of_ik(self, ik_result_joint_angles):
        return self.model.get_end_effector(ik_result_joint_angles)[0:4, 3]

    def get_effort(self):
        self.group_fbk = self.group.get_next_feedback(reuse_fbk=self.group_fbk)
        effort_fbk = self.group_fbk.effort
        # rospy.loginfo('Effort: {0}'.format(effort_fbk))
        return effort_fbk

    def set_arm_to_hold(self):
        self.group.command_lifetime = 0
        self.send()

    def release_arm_hold(self):
        self.group.command_lifetime = 100
        self.update()


class EndEffectorTrajectory:

    def __init__(self, robot_controller):
        self.robot_controller = robot_controller

        self.load_ros_params()
        
        self.motor4()

        self._lower_arm_service = rospy.Service(self._lower_arm_service_name,
                                                SetBool,
                                                self.lower_arm_callback)

        self.num_joints = len(robot_controller.get_joint_angles())

    def load_ros_params(self):
        # Load service names into params
        self._lower_arm_service_name = rospy.get_param(
            "lower_arm_service_name")

    def get_touchdown_waypoints(self):
        rows = 4  # num of joint angles
        cols = 6  # num of target waypoints
        pos = np.zeros((rows, cols))
        initial_joint_angles = self.robot_controller.get_joint_angles().tolist()
        pos[:, 0] = initial_joint_angles
        pos[:, 1] = [4.79459, 0.39897567, -2.67670345, -0.23143387]
        pos[:, 2] = [4.89430189, 0.72055084, -2.27849388, -0.22733116]
        pos[:, 3] = [4.83683014, 1.60074646, -2.15341663, -0.13106194]
        pos[:, 4] = [4.7667532, 2.01732284, -1.83409595, -0.66559677]
        pos[:, 5] = [4.70845413, 2.87031967, -1.8154192, 0.439913826]

        # initial_joint_angles = self.robot_controller.get_joint_angles().tolist(
        # )
        # positions = np.array([
        #     initial_joint_angles,
        #     [4.79459, 0.39897567, -2.67670345, -0.23143387],
        #     [4.89430189, 0.72055084, -2.27849388, -0.22733116],
        #     [4.83683014, 1.60074646, -2.15341663, -0.23106194],
        #     [4.7667532, 2.01732284, -1.83409595, -0.86559677],
        #     [4.70845413, 2.87031967, -1.8154192, 0.09913826],
        # ])

        return pos

    def get_return_waypoints(self):
        rows = 4  # num of joint angles
        cols = 5  # num of target waypoints
        pos = np.zeros((rows, cols))
        initial_angles = self.robot_controller.get_joint_angles()
        pos[:, 0] = initial_angles
        pos[:, 1] = [4.7661829, 2.58495361, -1.95124054, 0.439913826]
        pos[:, 2] = [4.83683014, 1.60074646, -2.15341663, 0.439913826]
        pos[:, 3] = [4.89430189, 0.72055084, -2.27849388, -0.12733116]
        pos[:, 4] = [4.79459, 0.39897567, -2.67670345, -0.23143387]
        return pos

        #positions = np.array([
        #    self.robot_controller.get_joint_angles().tolist(),
        #    [4.7661829, 2.58495361, -1.95124054, -0.92874146],
        #    [4.83683014, 1.60074646, -2.15341663, -0.23106194],
        #    [4.89430189, 0.72055084, -2.27849388, -0.22733116],
        #    [4.79459, 0.39897567, -2.67670345, -0.23143387],
        #])
        #return positions

    def get_hebi_trajectory(self, waypoints, total_duration=10):
        # positions = self.positions.T
        # positions = self.positions
        # print('Transpose shape:', self.positions.T.shape)
        # print('Normal shape: ',self.positions.shape)
        # print('Initial Joint Angles Shape', initial_joint_angles.shape)

        # Position, velocity, and acceleration waypoints
        self.pos = np.full(waypoints.shape, np.nan)
        self.vel = np.full(waypoints.shape, np.nan)
        self.acc = np.full(waypoints.shape, np.nan)

        # Set first and last waypoint values to 0.0
        self.vel[:, 0] = self.acc[:, 0] = 0.0
        self.vel[:, -1] = self.acc[:, -1] = 0.0

        # set time
        self.times = np.linspace(0, total_duration, waypoints.shape[1])
        # print(self.times)

        # Create the trajectory
        self.trajectory = hebi.trajectory.create_trajectory(
            self.times, np.copy(waypoints), self.vel, self.acc)

    def execute_trajectory_with_efforts(self, motor_id=1, max_effort=8):
        hz = float(20)
        rate = rospy.Rate(hz)

        # Follow the overall trajectory
        cmd = hebi.GroupCommand(self.num_joints)

        period = 0.01
        t = 0.0
        duration = self.trajectory.duration

        effort = np.zeros((int(duration / period) + 1, 4))
        i = 0

        while t < duration:
            # self.robot_controller.group.get_next_feedback(reuse_fbk=self.group_fbk)
            self.pos_cmd, self.vel_cmd, self.acc_cmd = self.trajectory.get_state(
                t)
            angles = robot_controller.get_joint_angles()
            # print(angles)

            cmd.position = self.pos_cmd
            cmd.velocity = self.vel_cmd

            self.robot_controller.group.send_command(cmd)

            effort[i, :] = self.robot_controller.get_effort()
            # checking change in effort within 0.5s
            delta_effort = effort[i, motor_id] - effort[i - 50, motor_id]

            # rospy.logdebug_throttle(0.2, f" | Effort: {effort[i]} | Step: {i} | Delta Effort: {delta_effort}")

            effort_check = delta_effort > max_effort
            # rospy.loginfo("effort_check: {effort_check}")

            if effort_check:
                rospy.loginfo(" | Torque out of bounds at t = {t}")
                self.robot_controller.group.command_lifetime = 0
                self.pos_cmd, self.vel_cmd, self.acc_cmd = self.trajectory.get_state(
                    t)
                cmd.position = self.pos_cmd
                cmd.velocity = 0
                self.robot_controller.group.send_command(cmd)
                rospy.loginfo(" | Motors stopped")
                break

            i += 1
            t = t + period
            sleep(period)

        # repeat_delay = 30
        # rospy.loginfo(
        #     "Staying in the last position for {0} seconds...".format(repeat_delay)
        # )
        # start_time = rospy.get_time()
        # while rospy.get_time() - start_time < repeat_delay:
        #     self.robot_controller.group.send_command(cmd)
        #     rate.sleep()

    def execute_trajectory(self):
        hz = float(20)
        rate = rospy.Rate(hz)

        # Follow the overall trajectory
        cmd = hebi.GroupCommand(self.num_joints)

        period = 0.01
        t = 0.0
        duration = self.trajectory.duration

        rospy.loginfo(f"Number of joints: {self.num_joints}")

        while t < duration:
            # self.robot_controller.group.get_next_feedback(reuse_fbk=self.group_fbk)
            self.pos_cmd, self.vel_cmd, self.acc_cmd = self.trajectory.get_state(
                t)

            cmd.position = self.pos_cmd
            cmd.velocity = self.vel_cmd

            # rospy.loginfo('commanded: ', cmd.position)
            self.robot_controller.group.send_command(cmd)

            angles = self.robot_controller.get_joint_angles()
            # rospy.loginfo('feedback: ',angles)

            t = t + period
            sleep(period)
        
        

    def lower_arm_callback(self, req):
        if (req.data):
            waypoints = self.get_touchdown_waypoints()
            self.get_hebi_trajectory(waypoints)
            self.execute_trajectory_with_efforts()
            # self.execute_trajectory()
            return SetBoolResponse(
                success=True,
                message="Touchdown Trajectory executed successfully")
        else:
            waypoints = self.get_return_waypoints()
            self.get_hebi_trajectory(waypoints)
            self.execute_trajectory()
            
            rospy.loginfo("Arm Drooping to Home...")
            rospy.sleep(5)
            rospy.loginfo("Arm Back to Home")
            
            self.motor4()
                
            return TriggerResponse(
                success=True,
                message="Return Trajectory executed successfully")
            
    def motor4(self):
        print("motor4")
        self.robot_controller.group.command_lifetime = 0
        cmd = hebi.GroupCommand(self.robot_controller.group.size)
        
        # initial_joint_angles = self.robot_controller.get_joint_angles().tolist()
        # print(initial_joint_angles)
        # initial_joint_angles[3] = -0.23143387
        # print(initial_joint_angles)
        
        
        cmd.position = [4.7336320877075195, -0.019812583923339844, -3.154970169067383, -0.23143387]
        
        self.robot_controller.group.send_command_with_acknowledgement(cmd)


if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("arm_control_node")
    
    rospy.loginfo("Started Arm Control Node...")
    

    # ROS Publisher for joint commands
    # joint_command_publisher = rospy.Publisher('/robot/joint_commands', JointState, queue_size=10)

    # Wait for 2 seconds to allow time for the ROS system to initialize
    # sleep(2.0)

    # Create instances of the classes
    robot_controller = RobotController(
        "Arm_RML",
        ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist"],
        "config/hrdf/anakin.hrdf",
    )
    end_effector_trajectory = EndEffectorTrajectory(robot_controller)

    rospy.spin()

    rospy.loginfo("Shutting down the ROS node.")
