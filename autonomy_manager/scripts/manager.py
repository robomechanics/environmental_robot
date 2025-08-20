#!/usr/bin/env python3
import math
from copy import deepcopy
from colorama import Fore, Back, Style
from numpy import where

import rosnode
import rospy
import actionlib

from std_srvs.srv import Empty
from std_srvs.srv import SetBool
from sensor_msgs.msg import NavSatFix

from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from std_msgs.msg import String
from autonomy_manager.srv import (
    SetSearchBoundary,
    NavigateGPS,
    Complete,
    Waypoints,
    SetString, SetStringResponse
)

from pxrf.msg import PxrfReading
from sklearn.gaussian_process.kernels import RBF
from pyproj import Transformer
from autonomy_manager import adaptiveROS, gridROS, Conversion
from autonomy_manager.algo_constants import *


class Manager(object):
    def __init__(self, 
                 skip_checks = True, 
                 debug_flag = True, 
                 fake_hardware_flags=[FAKE_MOVE_BASE],
                 fake_pxrf_values=None):
        
        rospy.init_node("manager", anonymous=False)
        rospy.sleep(0.1)
        
        self.debug_flag = debug_flag
        self.fake_hardware_flags = fake_hardware_flags
        self.take_first_sample = True
        
        self.load_ros_params()
        
        if FAKE_PXRF in self.fake_hardware_flags:
            if fake_pxrf_values:
                self.fake_pxrf_values = deepcopy(fake_pxrf_values)
                self.original_fake_pxrf_values = deepcopy(self.fake_pxrf_values)
            else:
                raise('Need Fake PXRF Values!')
        else:
            self._scan_completed_sub = rospy.Subscriber(self._scan_recorded_to_disk_topic, 
                                                        PxrfReading, 
                                                        self.pxrf_scan_completed_callback)
            self.pxrf_scan_service = rospy.ServiceProxy(self._start_scan_service_name, SetBool)
            self.pxrf_element_to_focus = "Pb"
            rospy.logwarn("Started PXRF")
        
        if not FAKE_ARM:
            self.sensor_arm_state_sub = rospy.Subscriber(self._sensor_arm_state_topic, String, self.sensor_arm_state_callback)

        self.statusPub = rospy.Publisher(
            self._status_topic, String, queue_size=10, latch=True
        )
        self.update_status(INIT)
        
        # Flags
        self.pxrf_complete = False
        self.pxrf_mean_value = None
        self.nav_goal_gps = None
        self.lat = None
        self.lon = None
        self.run_type = None
        self.run_once_flag = False
        self.is_arm_in_home_pose = True
        self._gps_sub = rospy.Subscriber(self._gq7_ekf_llh_topic, NavSatFix, self.gps_callback)

        # intialize adaptive sampling class and conversion class
        self.adaptiveROS = None     # Initialized when boundary is set
        self.gridROS = None         # Initialized when boundary is set
        self.conversion = Conversion(cells_per_meter=self._cells_per_meter)
        self.searchBoundary = []
        self.waypoints = []

        self.transformer = Transformer.from_crs(self._crs_GPS, self._crs_UTM)
        
        self._set_search_boundary_service = rospy.Service(
            self._set_search_boundary_name, SetSearchBoundary, self.set_search_boundary_callback
        )
        self._waypoints_service = rospy.Service(
            self._waypoints_service_name, Waypoints, self.set_waypoints
        )
        self._run_loop_service = rospy.Service(
            self._run_loop_service_name, SetString, self.run_loop_callback
        )
        self._reset_service = rospy.Service(self._clear_service_name, Complete, self.reset_callback)

        if not skip_checks:
            # Wait until GPS Full Nav is achieved
            msg = None
            while msg is None and not rospy.is_shutdown():
                self.update_status(WAITING_FOR_GPS_INIT)
                try:
                    msg = rospy.wait_for_message(self._gps_odom_topic, Odometry, timeout=3.0)
                except rospy.ROSException:
                    rospy.loginfo("Waiting for GPS Initialization...")
                    rospy.sleep(1)
            
            rospy.loginfo("GPS Full Navigation Achieved!") 
        
        if len(self.fake_hardware_flags) > 1:
            rospy.logwarn('>>> USING FAKE HARDWARE <<<<')
            rospy.logwarn(f'Fake Hardware Flags: {self.fake_hardware_flags}')
        if FAKE_MOVE_BASE not in self.fake_hardware_flags:
            self.mb_client = actionlib.SimpleActionClient(self._move_base_action_server_name, MoveBaseAction)
            if not skip_checks:
                self.update_status(WAITING_FOR_MOVE_BASE)
                rospy.loginfo_throttle(3,"Waiting for move base...")
                self.mb_client.wait_for_server() 
        
        self.update_status(READY)
        
        self.algorithm_type = ALGO_NONE
        
        rospy.loginfo(f"{Fore.GREEN}{Back.BLACK} ----------- READY ----------- {Style.RESET_ALL}")

    def run_once(self):
        """
        Sub-main loop called based on run type
        """
        self.run_once_flag = False
        
        self.algorithm_type = rospy.get_param(self._algorithm_type_param_name)
        
        rospy.loginfo(f"{Back.YELLOW}{Fore.BLACK}----------- Manager Loop: {self.algorithm_type} -----------{Style.RESET_ALL}")
        if self.status == RECEIVED_SEARCH_AREA or self.status == ARM_RETURNED or self.status == READY:
            if self.algorithm_type == ALGO_ADAPTIVE:
                self.run_adaptive_search_algo()
            elif self.algorithm_type == ALGO_WAYPOINT:
                self.run_waypoint_algo()
            elif self.algorithm_type == ALGO_GRID:
                self.run_grid_algo()
        elif self.status == RECEIVED_NEXT_SCAN_LOC:
            self.navigate_to_scan_loc()
        elif self.status == ARRIVED_AT_SCAN_LOC:
            if FAKE_ARM not in self.fake_hardware_flags:
                self.arm_touchdown()
            self.update_status(ARM_LOWERED)
        elif self.status == ARM_LOWERED:
            if FAKE_PXRF in self.fake_hardware_flags:
                self.fake_pxrf()
                self.update_status(FINISHED_SCAN)
            else:
                self.update_status(SCANNING)
                self.pxrf_scan_service(True)
        elif self.status == FINISHED_SCAN:
            if FAKE_ARM not in self.fake_hardware_flags:
                self.arm_return()
            self.update_status(ARM_RETURNED)
        elif self.status == ERROR:
            self.update_status(ERROR)            
        
        rospy.loginfo("----------- Manager Loop END -----------")
    
    def run(self):
        """
        Main loop
        """
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.status == SCANNING:
                # Wait for scanned data to be received
                pass
            elif self.run_type == "State Step" and self.run_once_flag == True:
                self.run_once()
            elif self.run_type == "Sample Step" and self.status != ARM_RETURNED and self.status != ERROR:
                self.run_once()
                self.run_once_flag = True
            elif self.run_type == "Continuous" and self.status != ERROR:
                self.run_once()
                self.run_once_flag = True
            
            rate.sleep()
    
    def run_loop_callback(self, data):
        """
        Triggered when manager step button is pressed in GUI
        Changes run_type
        """
        self.run_type = data.text
        
        if self.status != SCANNING:
            self.run_once_flag = True
        
        if self.run_type == "Sample Step" and self.status == ARM_RETURNED:
            self.update_status(READY)
            
        rospy.loginfo(f"self.run_type: {self.run_type} | self.run_once_flag: {self.run_once_flag} | self.status: {self.status}")
        return SetStringResponse(True, "SUCCESS")

    def load_ros_params(self):
        # Load topic names into params
        self._status_topic = rospy.get_param("status_topic")
        self._tf_utm_odom_frame = rospy.get_param("tf_utm_odom_frame")
        self._gq7_ekf_llh_topic = rospy.get_param("gq7_ekf_llh_topic")
        self._move_base_action_server_name = rospy.get_param('move_base_action_server_name')
        self._crs_GPS = rospy.get_param("crs_GPS")
        self._crs_UTM = rospy.get_param("crs_UTM")
        self._gps_odom_topic = rospy.get_param("gps_odom_topic")
        self._scan_recorded_to_disk_topic = rospy.get_param("scan_recorded_to_disk_topic")
        self._sensor_arm_state_topic = rospy.get_param("sensor_arm_state_topic")
        self._algorithm_type_param_name = rospy.get_param("algorithm_type_param_name")
        
        self.algorithm_type = rospy.get_param(self._algorithm_type_param_name)
        self.algorithm_total_samples = rospy.get_param("algorithm_total_samples")
        
        # Load service names into params
        self._set_search_boundary_name = rospy.get_param("set_search_boundary_name")
        self._clear_service_name = rospy.get_param("clear_service_name")
        self._waypoints_service_name = rospy.get_param("waypoints_service_name")
        self._grid_points_service_name = rospy.get_param("grid_points_service_name")
        self._next_goal_to_GUI_service_name = rospy.get_param("next_goal_to_GUI_service_name")
        self._lower_arm_service_name = rospy.get_param("lower_arm_service_name")
        self._start_scan_service_name = rospy.get_param("start_scan_service_name")
        self._run_loop_service_name = rospy.get_param("manager_run_loop_service_name")
        
        self._start_utm_x_param = rospy.get_param("start_utm_x_param")
        self._start_utm_y_param = rospy.get_param("start_utm_y_param")
        self._start_utm_lat_param = rospy.get_param("start_utm_lat_param")
        self._start_utm_lon_param = rospy.get_param("start_utm_lon_param")
        self._cells_per_meter = rospy.get_param("cells_per_meter")
        self._constant_velocity_commander_service_name = rospy.get_param("constant_velocity_commander_service_name")
        self._sim_mode = rospy.get_param("sim_mode")

    def pxrf_scan_completed_callback(self, data: PxrfReading):
        """
        Callback for scan completion
        """
        self.pxrf_complete = True
        element_index = where(data.elements == self.pxrf_element_to_focus)[0]
        self.pxrf_mean_value = data.concentrations[element_index]
        rospy.loginfo(f'PXRF Mean Value: {self.pxrf_mean_value}')
        
        self.update_status(FINISHED_SCAN)
        return True

    def sensor_arm_state_callback(self, data: String):
        self.is_arm_in_home_pose = bool(data.data == "IDLE")

    def send_autonomy_params(self, boundary_lat, boundary_lon, width, height):
        start_utm_x = rospy.get_param(self._start_utm_x_param)
        start_utm_y = rospy.get_param(self._start_utm_y_param)
        start_utm_lat = rospy.get_param(self._start_utm_lat_param)
        start_utm_lon = rospy.get_param(self._start_utm_lon_param)
        
        print(boundary_lat,
                boundary_lon,
                start_utm_x,
                start_utm_y,
                start_utm_lat,
                start_utm_lon,
                width,
                height,
                self.algorithm_total_samples)
            
    
    def set_search_boundary_callback(self, data):
        rospy.loginfo(f"----------------\n Boundary Points:\n {list(zip(data.boundary_x, data.boundary_y))}\n----------------")
        is_gps_type = data.boundary_type != 1 # Anything but 1 (map) will default to gps
        rospy.loginfo(f"Boundary type: {data.boundary_type} ({'gps' if is_gps_type else 'map'})")
        
        # data.boundary_x and data.boundary_y lists, put then in the format of [[lat1,lon1],[lat2,lon2],...]
        for i in range(len(data.boundary_x)):
            self.searchBoundary.append([data.boundary_x[i], data.boundary_y[i]])
        
        if is_gps_type:
            # Mode #1: [GPS] Convert gps coordinates into map coords
            # Initialize the zone, define boundary in utm coordinates
            self.conversion.get_zone(self.lat, self.lon)
            
            boundary_utm_offset = self.conversion.boundary_conversion(self.searchBoundary)
            startx, starty = self.conversion.gps2map(self.lat, self.lon)
        
            # rospy.loginfo(f'Data: \n {data}')
            # rospy.loginfo(f'boundary_utm_offset: \n {boundary_utm_offset}')
            # rospy.loginfo(f'conversion width and height: {self.conversion.width} {self.conversion.height}')

            # When you receive the search area, define the robot position as the starting point, make sure to drive the
            # the robot into the boundary first
            
            width_in_grid = self.conversion.width * self.conversion.cells_per_meter
            height_in_grid = self.conversion.height * self.conversion.cells_per_meter
            boundary_in_grid = [self.conversion.map2grid(p[0], p[1]) for p in boundary_utm_offset]
            startx_in_grid, starty_in_grid = self.conversion.map2grid(startx, starty)
            
            
            rospy.loginfo(f'Width: {self.conversion.width} m | {width_in_grid} cells')
            rospy.loginfo(f'Height: {self.conversion.height} m | : {height_in_grid} cells')
            rospy.loginfo(f'Start: ({startx}, {starty}) m | ({startx_in_grid}, {starty_in_grid}) cells')
            rospy.loginfo(f'Boundary Offset (MAP): {boundary_utm_offset}')
            rospy.loginfo(f'Boundary Offset (GRID): {boundary_in_grid}')
            
            self.adaptiveROS = adaptiveROS(
                size_x=width_in_grid,
                size_y=height_in_grid,
                startpoint=[startx_in_grid, starty_in_grid],
                max_samples=self.algorithm_total_samples,
                boundary = [],
                kernel=RBF(length_scale=100, length_scale_bounds=(5, 1e06))
            )
            # TODO: Check width and height
            self.adaptiveROS.update_boundary(boundary_in_grid)
            self.gridROS = gridROS(
                self.conversion.width, self.conversion.height, [0, 0], self.algorithm_total_samples
            )
        else:
            # Mode #2: [Map] type, used for simulation without need to convert between gps and map
            min_x, max_x = min(data.boundary_x), max(data.boundary_x)
            min_y, max_y = min(data.boundary_y), max(data.boundary_y)
            width = math.ceil(max_x - min_x)
            height = math.ceil(max_y - min_y)
            rospy.loginfo(f'Width: {width}')
            rospy.loginfo(f'Height: {height}')

            self.adaptiveROS = adaptiveROS(
                size_x=width,
                size_y=height,
                startpoint=[0, 0],
                max_samples=self.algorithm_total_samples,
                boundary=self.searchBoundary
            )
            self.gridROS = gridROS(
                width, height, [0, 0], self.algorithm_total_samples
            )
            self.conversion.width = width
            self.conversion.height = height
        
        rospy.loginfo(f'{Back.BLUE}lengths of x1 | x2 | x1x2: {len(self.adaptiveROS.x1)} | {len(self.adaptiveROS.x2)} | {self.adaptiveROS.x1x2.shape} {Style.RESET_ALL}')
        
        
        # self.gridROS.updateBoundary(boundary_utm_offset)
        # Call ros service to pass all the grid points
        #TODO: Uncomment grid points
        # lat = []
        # lon = []
        # for i in range(len(self.gridROS.grid_points)):
        #     gps = self.conversion.map2gps(
        #         self.gridROS.grid_points[i][0], self.gridROS.grid_points[i][1]
        #     )
        #     lat.append(gps[0])
        #     lon.append(gps[1])
        # rospy.loginfo(f"-----------------\n Grid Points of length = {len(lat)}:\n Lat: {lat}\n Lon: {lon}\n -----------------\n")
        if FAKE_ARM not in self.fake_hardware_flags: 
            self.send_autonomy_params(data.boundary_x, 
                                    data.boundary_y,
                                    width_in_grid, 
                                    height_in_grid)

        # TODO: Display grid points in GUI
        # try:
        #     grid_points = rospy.ServiceProxy(self._grid_points_service_name, Waypoints)
        #     res = grid_points(lat, lon)
        # except rospy.ServiceException as e:
        #     rospy.logerr(e)
        #     rospy.logerr("Grid points display failed")
            
        

        # self.gridROS.updateBoundary(boundary_utm_offset)
        # rospy.loginfo(boundary_utm_offset)
        self.update_status(RECEIVED_SEARCH_AREA)

        self.init_pos_gps = (self.lat, self.lon)
        self.init_pos_map = self.conversion.gps2map(self.init_pos_gps[0], self.init_pos_gps[1])
        self.init_pos_grid = self.conversion.map2grid(self.init_pos_map[0], self.init_pos_map[1])
        
        rospy.loginfo(f'{Back.YELLOW}{Fore.BLACK} | Inital Robot Location (GPS|Map|Grid): {self.init_pos_gps} | {self.init_pos_map} | {self.init_pos_grid} {Style.RESET_ALL}')
        
        if self.take_first_sample:
            # self.backup_robot()
            self.update_status(ARRIVED_AT_SCAN_LOC)
        
        return True
    
    def set_waypoints(self, data):
        """
        Callback function for waypoints service
        """
        self.waypoints = []
        for i in range(len(data.waypoints_lat)):
            self.waypoints.append([data.waypoints_lat[i], data.waypoints_lon[i]])
        # fake search area to faciliate the state machine
        rospy.loginfo(f"-----------------\n Received Waypoints:\n {self.waypoints} \n-----------------")
        self.update_status(RECEIVED_SEARCH_AREA)
        return True

    def update_status(self, newStatus):
        """
        Publish manager status
        """
        self.status = newStatus
        msg = String()
        msg.data = self.status
        self.statusPub.publish(msg)
        if self.debug_flag:
            rospy.loginfo(f'{Back.BLUE}{Fore.WHITE} < Status: {self.status} > {Style.RESET_ALL}')
            # input("Press Enter to Continue")

    def gps_callback(self, data: NavSatFix):
        self.lat = data.latitude
        self.lon = data.longitude
    
    def backup_robot(self):
        """
        Calls the constant velocity publisher service to back up the robot for scanning
        """
        try:
            constant_vel_cmder_client = rospy.ServiceProxy(self._constant_velocity_commander_service_name, Empty)
            res = constant_vel_cmder_client()
        except rospy.ServiceException as e:
            rospy.logerr(e)
            rospy.logerr("Backup Service Failed!")

    def send_location_to_GUI(self, x, y):
        try:
            next_goal_to_GUI = rospy.ServiceProxy(self._next_goal_to_GUI_service_name, NavigateGPS)
            res = next_goal_to_GUI(x, y)
        except rospy.ServiceException as e:
            rospy.logwarn("Sending location to GUI failed")

    def navigate_to_scan_loc(self):
        """
        Publish next scan location to move base
        """
        self.update_status(NAVIGATION_TO_SCAN_LOC)
        if FAKE_ARM not in self.fake_hardware_flags and not self.is_arm_in_home_pose:
            rospy.logerr("Arm is not in home pose, will not publish move_base goal!")
            self.update_status(ERROR)
            return

        if FAKE_MOVE_BASE in self.fake_hardware_flags:
            return

        #TODO: Orientation for goal
        self.goal_x_UTM, self.goal_y_UTM  = self.transformer.transform(self.nav_goal_gps[0], self.nav_goal_gps[1])
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self._tf_utm_odom_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        
        self.x_UTM_start = rospy.get_param(self._start_utm_x_param)
        self.y_UTM_start = rospy.get_param(self._start_utm_y_param)
        
        goal.target_pose.pose.position.x = self.goal_x_UTM - self.x_UTM_start
        goal.target_pose.pose.position.y = self.goal_y_UTM - self.y_UTM_start
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0
        goal.target_pose.pose.orientation.w = 1 

        self.mb_client.send_goal(goal)
        rospy.loginfo(" | Goal Sent to movebase...")
        wait = self.mb_client.wait_for_result()
        rospy.loginfo(" | Movebase Goal Reached, Backing up...")
        
        # Backup
        # self.backup_robot()
    
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            self.update_status(ERROR)
        else:
            self.update_status(ARRIVED_AT_SCAN_LOC)
            return self.mb_client.get_result()
        

    def arm_return(self):
        self.update_status(ARM_RETURNING)
        try:
            lower_arm = rospy.ServiceProxy(self._lower_arm_service_name, SetBool)
            res = lower_arm(False)
        except rospy.ServiceException as e:
            rospy.logwarn("Arm Return Failed")

    def arm_touchdown(self):
        self.update_status(ARM_LOWERING)
        try:
            lower_arm = rospy.ServiceProxy(self._lower_arm_service_name, SetBool)
            res = lower_arm(True)
        except rospy.ServiceException as e:
            rospy.logwarn("Arm Touchdown Failed")
    
    def fake_pxrf(self):
        self.pxrf_complete = True
        self.pxrf_mean_value = self.fake_pxrf_values.pop(0)
        rospy.loginfo(f'PXRF Mean Value: {self.pxrf_mean_value}')
    
    def reset_callback(self, data):
        """
        Resets manager
        """
        if data.status == True:
            rospy.logwarn("| Reset ")
            self.adaptiveROS = None
            self.gridROS = None
            self.conversion = Conversion(cells_per_meter=self._cells_per_meter)
            self.searchBoundary = []
            if FAKE_PXRF in self.fake_hardware_flags:
                if self.original_fake_pxrf_values:
                    self.fake_pxrf_values = deepcopy(self.original_fake_pxrf_values)
                else:
                    raise('Need Fake PXRF Values!')
        return True

    ############# Next Waypoint Sampling Algorithms #############
    # TODO:Might need to convert to GPS coordinates
    def run_waypoint_algo(self):
        """
        Send next waypoint from stored waypoints
        """
        # Update status
        self.update_status(RUNNING_WAYPOINT_ALGO)
        # Reset PXRF
        self.pxrf_complete = False
        self.pxrf_mean_value = None
        # Finish if no waypoints left
        if not len(self.waypoints):
            self.update_status(DONE)
            return
        # Get next location
        self.nextScanLoc = self.waypoints.pop(0)
        self.send_location_to_GUI(self.nextScanLoc[0], self.nextScanLoc[1])
        self.nav_goal_gps = [self.nextScanLoc[0], self.nextScanLoc[1]]
        self.update_status(RECEIVED_NEXT_SCAN_LOC)

    def run_adaptive_search_algo(self):
        """
        Uses Adaptive Search algorithm to predict next waypoint
        """
        # Check initialization
        if self.adaptiveROS is None:
            rospy.logerr("Adaptive Search algorithm requires boundary to be set! Aborting.")
            self.run_type = None
            self.run_once_flag = False
            return

        # Update status
        self.update_status(RUNNING_SEARCH_ALGO)
        # If there is already a scan done, update the values in the algorithm
        if self.pxrf_complete == True and self.pxrf_mean_value != None:
            pos = self.conversion.gps2map(self.lat, self.lon)
            r,c = self.conversion.map2grid(pos[0], pos[1])
            rospy.loginfo(f"{Back.YELLOW}{Fore.BLACK} | Updating GPR with value={self.pxrf_mean_value} at (GPS|Map|Grid): {(self.lat, self.lon)} | {pos} | {(r,c)} {Style.RESET_ALL}")
            self.adaptiveROS.update(r, c, self.pxrf_mean_value)
        # Reset PXRF
        self.pxrf_complete = False
        self.pxrf_mean_value = None
        # Predict next location
        self.nextScanLoc = self.adaptiveROS.predict(True)
        if self._sim_mode:
            # No conversion needed for simulation
            self.nav_goal_map = None
            self.nav_goal_gps = self.nextScanLoc
        else:
            self.nav_goal_map = self.conversion.grid2map(self.nextScanLoc[0], self.nextScanLoc[1])
            self.nav_goal_gps = self.conversion.map2gps(self.nav_goal_map[0], self.nav_goal_map[1])

        rospy.loginfo(f"{Back.GREEN}{Fore.BLACK} | Sending Adaptive Algorithm Location (GPS|Map|Grid): {self.nav_goal_gps} | {self.nav_goal_map} | {self.nextScanLoc} {Style.RESET_ALL}")
        self.send_location_to_GUI(self.nav_goal_gps[0], self.nav_goal_gps[1])
        self.update_status(RECEIVED_NEXT_SCAN_LOC)

    def run_grid_algo(self):
        """
        Uses Grid algo to get next waypoint
        """
        # Check initialization
        if self.gridROS is None:
            rospy.logerr("Grid algorithm requires boundary to be set! Aborting.")
            self.run_type = None
            self.run_once_flag = False
            return
        
        # Update status
        self.update_status(RUNNING_GRID_ALGO)
        # Reset PXRF
        self.pxrf_complete = False
        self.pxrf_mean_value = None
        # Get next location
        self.nextScanLoc = self.gridROS.next()
        self.nav_goal_map = self.conversion.grid2map(self.nextScanLoc[0], self.nextScanLoc[1])
        self.nav_goal_gps = self.conversion.map2gps(self.nav_goal_map[0], self.nav_goal_map[1])
        self.send_location_to_GUI(self.nav_goal_gps[0], self.nav_goal_gps[1])
        self.update_status(RECEIVED_NEXT_SCAN_LOC)
        

if __name__ == "__main__":    
    manager = Manager()
    manager.run()
