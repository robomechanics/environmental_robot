#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import numpy as np
from time import sleep
import hebi
import rospkg
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from os.path import join as os_path_join
from std_msgs.msg import String


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
        self.batt_voltage = 0.0
        # gainsCmd.mstop_strategy = 2
    
    def get_batt_voltage(self):
        # Get position feedback from the robot to use as initial conditions
        group_fbk = hebi.GroupFeedback(self.group.size)
        
        if self.group.get_next_feedback(reuse_fbk=group_fbk) is None:
            rospy.logerr("Couldn't get feedback")
            return False
        else:
            self.batt_voltage = min(group_fbk.voltage)
            return True

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

    def __init__(self, robot_controller: RobotController):
        self.robot_controller = robot_controller

        self.load_ros_params()
        rospy.on_shutdown(self.shutdown_handler)
        
        
       
        self.home_pose = [4.7336320877075195, -0.019812583923339844, -3.154970169067383, -0.23143387]
        self.num_joints = len(robot_controller.get_joint_angles())
        
        # Move arm to home pose
        if not self.check_if_robot_in_home_pose():
            self.arm_return()
        
        self._lower_arm_service = rospy.Service(self._lower_arm_service_name,
                                                SetBool,
                                                self.lower_arm_callback)

        self.lipo_battery_voltage_pub = rospy.Publisher(self._lipo_battery_voltage_topic,
                                                        String, 
                                                        queue_size=1)
        
        self._batt_voltage_timer = rospy.Timer(rospy.Duration(30), self.publish_lipo_battery_voltage)

        
    def load_ros_params(self):
        # Load service names into params
        self._lower_arm_service_name = rospy.get_param(
            "lower_arm_service_name")
        self._is_arm_in_home_pose_param_name = rospy.get_param(
            "is_arm_in_home_pose_param_name")
        self._lipo_battery_voltage_topic = rospy.get_param(
            "lipo_battery_voltage_topic")
    
    def publish_lipo_battery_voltage(self, data):
        self.robot_controller.get_batt_voltage()
        self.lipo_battery_voltage_pub.publish(f"{self.robot_controller.batt_voltage:.1f}")

    def get_forward_waypoints(self, traj_type = 0):
        if traj_type == 0:
            rows = 4  # num of joint angles
            cols = 5  # num of target waypoints
            pos = np.zeros((rows, cols))
            initial_joint_angles = self.robot_controller.get_joint_angles().tolist()
            pos[:, 0] = initial_joint_angles
            pos[:, 1] = [4.79459, 0.39897567, -2.67670345, -0.23143387]
            pos[:, 2] = [4.79459, 0.72055084, -2.1341663, -0.22733116]
            pos[:, 3] = [4.79459, 1.60074646, -2.1341663, 0.439913826]
            pos[:, 4] = [4.79459, 2.3232284, -1.83409595, 0.439913826]
            return pos
        
        elif traj_type == 1:
            rows = 4  # num of joint angles
            cols = 2  # num of target waypoints
            pos = np.zeros((rows, cols))
            # initial_joint_angles = self.robot_controller.get_joint_angles().tolist()
            pos[:, 0] = [4.79459, 2.3232284, -1.83409595, 0.439913826]
            pos[:, 1] = [4.79459, 2.87031967, -1.8154192, 0.439913826]
            return pos

    def get_return_waypoints(self, traj_type = 0):
        if traj_type == 0:
            rows = 4  # num of joint angles
            cols = 2  # num of target waypoints
            pos = np.zeros((rows, cols))
            initial_angles = self.robot_controller.get_joint_angles()
            pos[:, 0] = initial_angles
            pos[:, 1] = [4.7661829, 2.00495361, -2.06000, 0.439913826]
            return pos
        elif traj_type == 1:
            rows = 4  # num of joint angles
            cols = 4  # num of target waypoints
            pos = np.zeros((rows, cols))
            pos[:, 0] = [4.7661829, 2.00495361, -2.06000, 0.329913826]
            pos[:, 1] = [4.83683014, 1.60074646, -2.15341663, 0.329913826]
            pos[:, 2] = [4.79459, 0.39897567, -2.67670345, -0.23143387]
            pos[:, 3] = list(self.home_pose)
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

        # rospy.loginfo(f"Number of joints: {self.num_joints}")

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
            if not self.check_if_robot_in_home_pose():
                return SetBoolResponse(True, "Arm is already in touchdown position")
            self.arm_touchdown()
        else:
            if self.check_if_robot_in_home_pose():
                return SetBoolResponse(True, "Arm is already in home position")
            self.arm_return()
        return SetBoolResponse(True, "SUCCESS")
           
    def arm_touchdown(self):
        # Set a param flag that persists across restarts
        rospy.set_param(self._is_arm_in_home_pose_param_name, False)
        
        waypoints_forward = self.get_forward_waypoints(traj_type=0)
        self.get_hebi_trajectory(waypoints_forward, 3)
        self.execute_trajectory_with_efforts()
        
        waypoints_touchdown = self.get_forward_waypoints(traj_type=1)
        self.get_hebi_trajectory(waypoints_touchdown, 4)
        self.execute_trajectory_with_efforts()
        
        # self.execute_trajectory()
        return SetBoolResponse(
            success=True,
            message="Touchdown Trajectory executed successfully")
    
    def arm_return(self):
        waypoints = self.get_return_waypoints(traj_type=0)
        self.get_hebi_trajectory(waypoints, 4)
        self.execute_trajectory()
        
        waypoints = self.get_return_waypoints(traj_type=1)
        self.get_hebi_trajectory(waypoints, 5)
        self.execute_trajectory()
        
        self.goto_and_hold_home_pose()
        
        # Set a param flag that persists across restarts
        rospy.set_param(self._is_arm_in_home_pose_param_name, True)
            
        return TriggerResponse(
            success=True,
            message="Return Trajectory executed successfully")
    
    def goto_and_hold_home_pose(self):
        self.robot_controller.group.command_lifetime = 0
        cmd = hebi.GroupCommand(self.robot_controller.group.size)
        
        cmd.position = self.home_pose
        
        self.robot_controller.group.send_command_with_acknowledgement(cmd)

    def check_if_robot_in_home_pose(self):
        current_joint_angles = self.robot_controller.get_joint_angles()
        pose_diff = abs(np.max(current_joint_angles-np.array(self.home_pose)))
        rospy.loginfo(f"Home Pose Diff: {pose_diff}")
        if pose_diff > 0.2:
            rospy.loginfo("Arm is NOT in home pose")
            return False
        else:
            rospy.set_param(self._is_arm_in_home_pose_param_name, True)
            rospy.loginfo("Arm is in home pose")
            return True
    
    def shutdown_handler(self):
        # Move arm to home pose
        if not self.check_if_robot_in_home_pose():
            self.arm_return()
        rospy.loginfo("Arm should be in home position, Exiting...")

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("arm_control")
    
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
