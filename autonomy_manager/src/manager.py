#!/usr/bin/env python3
import rospy
from autonomy_manager.msg import ManagerStatus
from autonomy_manager.srv import (
    SetSearchBoundary,
    NavigateGPS,
    RunSensorPrep,
    Complete,
    Waypoints,
)
from std_srvs.srv import Empty, EmptyResponse
from pxrf.msg import CompletedScanData
from sensor_msgs.msg import NavSatFix, Image
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from adaptiveROS import adaptiveROS
from gridROS import gridROS
from boundaryConversion import Conversion
from sensor_msgs.msg import Joy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from pyproj import Transformer
import rosnode
from autonomy_manager.srv import AutonomyParams
from gps_gui.srv import SetString, SetStringResponse
from env_utils.algo_constants import *
from env_utils.pxrf_utils import PXRF
from colorama import Fore, Back, Style
from utils import visualizer_recreate_real
from sklearn.gaussian_process.kernels import RBF
import math
from roverrobotics_navigation.fake_gps_publisher import FakeGPSPublisher
import tf2_ros

class Manager(object):
    def __init__(self, skip_checks = False, debug_flag = False, show_plot = False, fake_hardware_flags=[]):
        
        rospy.init_node("manager", anonymous=False)
        rospy.sleep(0.1)
        
        self.debug_flag = debug_flag
        self.show_plot = show_plot
        
        self.load_ros_params()

        self.statusPub = rospy.Publisher(
            self._status_topic, ManagerStatus, queue_size=10, latch=True
        )
        self.update_status(INIT)
        
        # Flags
        self.pxrf_complete = False
        self.pxrf_mean_value = None
        self.is_full_nav_achieved = False
        self.nav_goal_gps = None
        self.lat = None
        self.lon = None
        self.run_type = None
        self.run_once_flag = False
        self.is_arm_in_home_pose = None
        self._gps_sub = rospy.Subscriber(self._gq7_ekf_llh_topic, NavSatFix, self.gps_callback)
        
        self.sensorPrep = rospy.ServiceProxy(
            self._sensor_prep_service_name, RunSensorPrep
        )
        

        # Check if Full Nav achieved
        self.odom_sub = rospy.Subscriber(
            self._gps_odom_topic, Odometry, self.gps_odom_callback
        )

        # intialize adaptive sampling class and conversion class
        self.adaptiveROS = None
        self.gridROS = None
        self.conversion = Conversion(cells_per_meter=self._cells_per_meter)
        self.searchBoundary = []
        self.waypoints = []

        self.transformer = Transformer.from_crs(self._crs_GPS, self._crs_UTM)
        
        self._set_search_boundary_service = rospy.Service(
            self._set_search_boundary_name, SetSearchBoundary, self.set_search_boundary_callback
        )
        
        self._reset_service = rospy.Service(self._clear_service_name, Complete, self.reset_callback)
        
        self._waypoints_service = rospy.Service(
            self._waypoints_service_name, Waypoints, self.set_waypoints
        )
        self._run_loop_service = rospy.Service(
            self._run_loop_service_name, SetString, self.run_loop_callback
        )

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        if self._sim_mode:
            self.image_pub = rospy.Publisher(self._fake_pxrf_img_topic, Image, queue_size=10)

        if not skip_checks:
            # Wait until GPS Full Nav is achieved
            while not self.is_full_nav_achieved:
                self.update_status(WAITING_FOR_GPS_INIT)
                rospy.loginfo_throttle(3,"Waiting for GPS Initialization...")
                rospy.sleep(1)
            
            rospy.loginfo("GPS Full Navigation Achieved!") 
        self.odom_sub.unregister()
        
        self.update_status(READY)
        
        self.setup_hardware(fake_hardware_flags, skip_checks)
        
        
        # Reset and Get rosparam
        self.reset_algo_type = False
        if self.reset_algo_type:
            rospy.loginfo(" | Waiting to start (Choose a sampling algorithm)")
            rospy.set_param(self._algorithm_type_param_name, ALGO_NONE)
            self.algorithm_type = ALGO_NONE
            rospy.sleep(0.5)
            while self.algorithm_type == ALGO_NONE:
                self.algorithm_type = rospy.get_param(self._algorithm_type_param_name)
                self.algorithm_total_samples = rospy.get_param("algorithm_total_samples")
                rospy.sleep(1)
        else:
            self.algorithm_type = ALGO_ADAPTIVE
            
        if self.algorithm_type == ALGO_ADAPTIVE:
            rospy.loginfo(f" | Algorithm Set to ADAPTIVE with number of samples = {self.algorithm_total_samples}")
        elif self.algorithm_type == ALGO_WAYPOINT:
            rospy.loginfo(f" | Algorithm Set to WAYPOINT")
        elif self.algorithm_type == ALGO_GRID:
            rospy.loginfo(f" | Algorithm Set to GRID")
        
        rospy.loginfo(f"{Fore.GREEN}{Back.BLACK} ----------- READY ----------- {Style.RESET_ALL}")
        
    def setup_hardware(self, fake_hardware_flags, skip_checks):
        self.fake_hardware_flags = fake_hardware_flags
        self.skip_checks = skip_checks
        self.fake_hardware_mode = len(self.fake_hardware_flags) > 1
        
        if self.fake_hardware_mode:
            rospy.logwarn('>>> USING FAKE HARDWARE <<<<')
            rospy.logwarn(f'Fake Hardware Flags: {fake_hardware_flags}')
            
        if FAKE_MOVE_BASE not in self.fake_hardware_flags:
            self.mb_client = actionlib.SimpleActionClient(self._move_base_action_server_name, MoveBaseAction)
            if skip_checks:
                rospy.loginfo(" | Waiting for move_base server")
                self.mb_client.wait_for_server()
        
        if FAKE_PXRF in self.fake_hardware_flags:
            self.fake_pxrf_values = []
        else:
            self._scan_completed_sub = rospy.Subscriber(self._scan_recorded_to_disk_topic, CompletedScanData, self.pxrf_scan_completed_callback)
            self.pxrf = PXRF()
            rospy.logwarn("Started PXRF")    
    
    def _unused(self):
        rospy.Subscriber(self._joy_topic, Joy, self.manual_behavior_skip)
        self.isOverride = False
        
        def manual_behavior_skip(self, data):
            self.isOverride = data.buttons[5] == 1
            if data.buttons[1] > 0 and (
                not hasattr(self, "last_skip_button_status")
                or not self.last_skip_button_status
            ):
                if self.status == RAKING:
                    self.sensorPrep(False)
                    self.update_status(FINISHED_RAKING)
                elif self.status == NAVIGATION_TO_SCAN_LOC:
                    cancel_goal = rospy.ServiceProxy(self._cancel_goal_topic, NavigateGPS)
                    cancel_goal(0, 0)
                    self.update_status(ARRIVED_AT_SCAN_LOC)
                elif self.status == FINISHED_RAKING:
                    self.update_status(FINISHED_SCAN)
            self.last_skip_button_status = data.buttons[1] > 0
    
    def gps_odom_callback(self, data: Odometry):
        self.is_full_nav_achieved = True

    def run_once(self):
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
            if self.fake_hardware_mode:
                if FAKE_ARM in self.fake_hardware_flags:
                    if FAKE_PXRF in self.fake_hardware_flags:
                        self.fake_arm_and_pxrf()
                    else:
                        self.update_status(ARM_LOWERED)
            else:
                self.arm_touchdown()
        elif self.status == ARM_LOWERED or self.status == FINISHED_RAKING:
            if FAKE_PXRF in self.fake_hardware_flags:
                self.fake_pxrf()
            else:
                self.scan()
        elif self.status == FINISHED_SCAN:
            if FAKE_ARM in self.fake_hardware_flags:
                self.update_status(ARM_RETURNED)
            else:
                self.arm_return()
        elif self.status == ERROR:
            manual_status = rospy.get_param(self._manager_set_status_after_error_param_name, ERROR)
            rospy.logwarn(f"Set Status to: {manual_status}")
            self.update_status(manual_status)
            
            # Reset to ERROR
            rospy.set_param(self._manager_set_status_after_error_param_name, ERROR)
            
        
        rospy.loginfo("----------- Manager Loop END -----------")
    
    def run(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.status != SCANNING:
                if self.run_type == "State Step" and self.run_once_flag == True:
                    self.run_once()
                elif self.run_type == "Sample Step" and self.status != ARM_RETURNED and self.status != ERROR:
                    self.run_once()
                    self.run_once_flag = True
                elif self.run_type == "Continuous" and self.status != ERROR:
                    self.run_once()
                    self.run_once_flag = True
            
            rate.sleep()
    
    def run_loop_callback(self, data):
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
        # self._sensor_prep_status_topic = rospy.get_param("sensor_prep_status_topic")
        self._joy_topic = rospy.get_param("joy_topic")
        self._tf_utm_odom_frame = rospy.get_param("tf_utm_odom_frame")
        self._gq7_ekf_llh_topic = rospy.get_param("gq7_ekf_llh_topic")
        self._move_base_action_server_name = rospy.get_param('move_base_action_server_name')
        self._crs_GPS = rospy.get_param("crs_GPS")
        self._crs_UTM = rospy.get_param("crs_UTM")
        self._manager_set_status_after_error_param_name = rospy.get_param("manager_set_status_after_error_param_name")
        self._gps_odom_topic = rospy.get_param("gps_odom_topic")
        self._scan_recorded_to_disk_topic = rospy.get_param("scan_recorded_to_disk_topic")
        self._is_arm_in_home_pose_param_name = rospy.get_param("is_arm_in_home_pose_param_name")
        self._algorithm_type_param_name = rospy.get_param("algorithm_type_param_name")
        
        self.algorithm_type = rospy.get_param(self._algorithm_type_param_name)
        self.algorithm_total_samples = rospy.get_param("algorithm_total_samples")
        
        
        # Load service names into params
        self._sensor_prep_service_name = rospy.get_param("sensor_prep_service_name")
        self._set_search_boundary_name = rospy.get_param("set_search_boundary_name")
        self._clear_service_name = rospy.get_param("clear_service_name")
        self._waypoints_service_name = rospy.get_param("waypoints_service_name")
        self._grid_points_service_name = rospy.get_param("grid_points_service_name")
        self._next_goal_to_GUI_service_name = rospy.get_param("next_goal_to_GUI_service_name")
        self._lower_arm_service_name = rospy.get_param("lower_arm_service_name")
        self._start_scan_service_name = rospy.get_param("start_scan_service_name")
        self._cancel_goal_topic = rospy.get_param("cancel_goal_topic")
        self._run_loop_service_name = rospy.get_param("manager_run_loop_service_name")
        self._autonomy_params_service_name = rospy.get_param("autonomy_params_service_name")
        
        self._start_utm_x_param = rospy.get_param("start_utm_x_param")
        self._start_utm_y_param = rospy.get_param("start_utm_y_param")
        self._start_utm_lat_param = rospy.get_param("start_utm_lat_param")
        self._start_utm_lon_param = rospy.get_param("start_utm_lon_param")
        self._cells_per_meter = rospy.get_param("cells_per_meter")
        self._constant_velocity_commander_service_name = rospy.get_param("constant_velocity_commander_service_name")
        
        # Sim params
        self._sim_mode = rospy.get_param("sim_mode")
        if self._sim_mode:
            self._fake_pxrf_img_topic = rospy.get_param("fake_pxrf_img_topic")


    def scan(self):
        self.update_status(SCANNING)
        self.pxrf.start_scan()

    def pxrf_scan_completed_callback(self, data):
        if data.status == True:
            self.pxrf_complete = True
            self.pxrf_mean_value = data.mean
            rospy.loginfo(f'PXRF Mean Value: {self.pxrf_mean_value}')
            
            self.update_status(FINISHED_SCAN)
        return True

    def send_autonomy_params(self, boundary_lat, boundary_lon, width, height):
        start_utm_x = rospy.get_param(self._start_utm_x_param)
        start_utm_y = rospy.get_param(self._start_utm_y_param)
        start_utm_lat = rospy.get_param(self._start_utm_lat_param)
        start_utm_lon = rospy.get_param(self._start_utm_lon_param)
        
        try:
            send_autonomy_params_client = rospy.ServiceProxy(self._autonomy_params_service_name, AutonomyParams)
            res = send_autonomy_params_client(boundary_lat,
                                              boundary_lon,
                                              start_utm_x,
                                              start_utm_y,
                                              start_utm_lat,
                                              start_utm_lon,
                                              width,
                                              height,
                                              self.algorithm_total_samples)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            rospy.logerr("Send Autonomy Params service call failed!")
            
    
    def set_search_boundary_callback(self, data):
        rospy.loginfo(f"----------------\n Boundary Points:\n {list(zip(data.boundary_x, data.boundary_y))}\n----------------")
        is_gps_type = data.boundary_type != 1 # Anything but 1 (map) will default to gps
        rospy.loginfo(f"Boundary type: {data.boundary_type} ({'gps' if is_gps_type else 'map'})")
        
        # data.boundary_x and data.boundary_y lists, put then in the format of [[lat1,lon1],[lat2,lon2],...]
        self.searchBoundary = [[x, y] for x,y in zip(data.boundary_x, data.boundary_y)]
        
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
                total_number=self.algorithm_total_samples,
                boundary = []
            )
            self.adaptiveROS.update_boundary(boundary_in_grid)
            self.gridROS = gridROS(
                self.conversion.width, self.conversion.height, [0, 0], self.algorithm_total_samples
            )
            
            # self.gridROS.updateBoundary(boundary_utm_offset)
            # Call ros service to pass all the grid points
            lat = []
            lon = []
            for i in range(len(self.gridROS.grid_points)):
                gps = self.conversion.map2gps(
                    self.gridROS.grid_points[i][0], self.gridROS.grid_points[i][1]
                )
                lat.append(gps[0])
                lon.append(gps[1])
            rospy.loginfo(f"-----------------\n Grid Points of length = {len(lat)}:\n Lat: {lat}\n Lon: {lon}\n -----------------\n")
        
            self.adaptiveROS = adaptiveROS(
                size_x=width_in_grid,
                size_y=height_in_grid,
                startpoint=[startx_in_grid, starty_in_grid],
                total_number=self.algorithm_total_samples,
                boundary = [],
                kernel=RBF(length_scale=100, length_scale_bounds=(5, 1e06)),
                conversion=self.conversion
            )
            # TODO: Check width and height
            self.adaptiveROS.update_boundary(boundary_in_grid)
            self.gridROS = gridROS(
                self.conversion.width, self.conversion.height, [0, 0], self.algorithm_total_samples
            )

            self.init_pos_gps = (self.lat, self.lon)
            self.init_pos_map = self.conversion.gps2map(self.init_pos_gps[0], self.init_pos_gps[1])
            self.init_pos_grid = self.conversion.map2grid(self.init_pos_map[0], self.init_pos_map[1])
            
            rospy.loginfo(f'{Back.YELLOW}{Fore.BLACK} | Inital Robot Location (GPS|Map|Grid): {self.init_pos_gps} | {self.init_pos_map} | {self.init_pos_grid} {Style.RESET_ALL}')
            
        else:
            # Mode #2: [Map] type, used for simulation without need to convert between gps and map
            min_x, max_x = min(data.boundary_x), max(data.boundary_x)
            min_y, max_y = min(data.boundary_y), max(data.boundary_y)
            self.conversion.width = math.ceil(max_x - min_x)
            self.conversion.height = math.ceil(max_y - min_y)
            self.conversion.origin_tf = [min_x, min_y]

            boundary_offset = [[data.boundary_x[i] - self.conversion.origin_tf[0], 
                                data.boundary_y[i] - self.conversion.origin_tf[1]] 
                                for i in range(len(data.boundary_x))]
        
            width_in_grid = self.conversion.width * self.conversion.cells_per_meter
            height_in_grid = self.conversion.height * self.conversion.cells_per_meter
            boundary_in_grid = [self.conversion.map2grid(p[0], p[1]) for p in boundary_offset]
            rover_x, rover_y, rover_z = FakeGPSPublisher.get_rover_pos()
            startx, starty = self.conversion.tf2map(rover_x, rover_y)
            startx_in_grid, starty_in_grid = self.conversion.map2grid(startx, starty)

            rospy.loginfo(f'Width: {self.conversion.width}')
            rospy.loginfo(f'Height: {self.conversion.height}')
            rospy.loginfo(f"adaptiveROS args: size_x={width_in_grid}, size_y={height_in_grid}, startpoint={[startx_in_grid, starty_in_grid]}, total_number={self.algorithm_total_samples}")

            self.adaptiveROS = adaptiveROS(
                size_x=width_in_grid,
                size_y=height_in_grid,
                startpoint=[startx_in_grid, starty_in_grid],
                total_number=self.algorithm_total_samples,
                boundary=[],
                kernel=RBF(length_scale=100, length_scale_bounds=(5, 1e06)),
                conversion=self.conversion
            )
            self.adaptiveROS.update_boundary(boundary_in_grid)
            self.gridROS = gridROS(self.conversion.width, self.conversion.height, [0, 0], self.algorithm_total_samples)
            
        
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

        
        return True

    def set_waypoints(self, data):
        self.waypoints = []
        for i in range(len(data.waypoints_lat)):
            self.waypoints.append([data.waypoints_lat[i], data.waypoints_lon[i]])
        # fake search area to faciliate the state machine
        rospy.loginfo(f"-----------------\n Received Waypoints:\n {self.waypoints} \n-----------------")
        self.update_status(RECEIVED_SEARCH_AREA)
        return True

    def navigate_to_scan_loc(self):
        self.update_status(NAVIGATION_TO_SCAN_LOC)
        # self.navigation(self.nextScanLoc[0],self.nextScanLoc[1])
        if self._sim_mode:
            self.publish_move_base_goal(self.nav_goal_tf_map[0], self.nav_goal_tf_map[1])
        else:
            self.publish_move_base_goal(self.nav_goal_gps[0], self.nav_goal_gps[1])

    def run_sensor_prep(self):
        self.sensorPrep(True)
        # TODO: Perform Sensor Prep
        self.update_status(RAKING)

    def update_status(self, newStatus):
        self.status = newStatus
        msg = ManagerStatus()
        msg.status = self.status
        msg.header.stamp = rospy.Time.now()
        self.statusPub.publish(msg)
        if self.debug_flag:
            rospy.loginfo(f'{Back.BLUE}{Fore.WHITE} < Status: {self.status} > {Style.RESET_ALL}')
            # input("Press Enter to Continue")

    def gps_callback(self, data: NavSatFix):
        self.lat = data.latitude
        self.lon = data.longitude

    def send_location_to_GUI(self, x, y):
        # rospy.wait_for_service('next_goal')
        try:
            next_goal_to_GUI = rospy.ServiceProxy(self._next_goal_to_GUI_service_name, NavigateGPS)
            res = next_goal_to_GUI(x, y)
        except rospy.ServiceException as e:
            rospy.logwarn("Sending location to GUI failed")

    def publish_move_base_goal(self, lat, lon):
        if FAKE_ARM not in self.fake_hardware_flags:
            while '/arm_control' not in rosnode.get_node_names():
                rospy.loginfo("Waiting for Arm Control Node")
                rospy.sleep(1)
            self.is_arm_in_home_pose = rospy.get_param(
                    self._is_arm_in_home_pose_param_name
                    ) # Arm pose flag that persists across restarts
                
            if not self.is_arm_in_home_pose:
                rospy.logerr("Arm is not in home pose, will not publish move_base goal!")
                self.update_status(ERROR)
                return
        
        if FAKE_MOVE_BASE not in self.fake_hardware_flags: 
            #TODO: Orientation for goal
            if self._sim_mode:
                # No UTM conversion needed for simulation
                pos_x = lat
                pos_y = lon
            else:
                # Convert GPS to UTM
                self.goal_x_UTM, self.goal_y_UTM  = self.transformer.transform(lat, lon)
                
                self.x_UTM_start = rospy.get_param(self._start_utm_x_param)
                self.y_UTM_start = rospy.get_param(self._start_utm_y_param)

                pos_x = self.goal_x_UTM - self.x_UTM_start
                pos_y = self.goal_y_UTM - self.y_UTM_start
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self._tf_utm_odom_frame
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = pos_x
            goal.target_pose.pose.position.y = pos_y
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation.x = 0
            goal.target_pose.pose.orientation.y = 0
            goal.target_pose.pose.orientation.z = 0
            goal.target_pose.pose.orientation.w = 1 

            self.mb_client.send_goal(goal)
            rospy.loginfo(" | Goal Sent to movebase...")
            wait = self.mb_client.wait_for_result()
            rospy.loginfo(" | Movebase Goal Reached, Backing up...")
            self.update_status(ARRIVED_AT_SCAN_LOC)
            
            # Backup
            try:
                constant_vel_cmder_client = rospy.ServiceProxy(self._constant_velocity_commander_service_name, Empty)
                res = constant_vel_cmder_client()
            except rospy.ServiceException as e:
                rospy.logerr(e)
                rospy.logerr("Backup Service Failed!")
        
            
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
                self.update_status(ERROR)
            else:
                return self.mb_client.get_result()       

    def arm_return(self):
        self.update_status(ARM_RETURNING)
        try:
            lower_arm = rospy.ServiceProxy(self._lower_arm_service_name, SetBool)
            res = lower_arm(False)
        except rospy.ServiceException as e:
            rospy.logwarn("Arm Return Failed")
        
        self.update_status(ARM_RETURNED)

    def arm_touchdown(self):
        self.update_status(ARM_LOWERING)
        try:
            lower_arm = rospy.ServiceProxy(self._lower_arm_service_name, SetBool)
            res = lower_arm(True)
        except rospy.ServiceException as e:
            rospy.logwarn("Arm Touchdown Failed")
        
        self.update_status(ARM_LOWERED)
        
    def reset_callback(self, data):
        if data.status == True:
            rospy.logwarn("| Reset ")
            self.adaptiveROS = None
            self.gridROS = None
            self.conversion = Conversion(cells_per_meter=self._cells_per_meter)
            self.searchBoundary = []

        return True

    # TODO:Might need to convert to GPS coordinates
    def run_waypoint_algo(self):
        self.update_status(RUNNING_WAYPOINT_ALGO)
        self.pxrf_complete = False
        self.pxrf_mean_value = None
        if not len(self.waypoints):
            self.update_status(DONE)
            return
        self.nextScanLoc = self.waypoints.pop(0)
        self.send_location_to_GUI(self.nextScanLoc[0], self.nextScanLoc[1])
        self.nav_goal_gps = [self.nextScanLoc[0], self.nextScanLoc[1]]
        self.update_status(RECEIVED_NEXT_SCAN_LOC)

    def run_adaptive_search_algo(self):
        self.update_status(RUNNING_SEARCH_ALGO)
        
        if self.pxrf_complete == True and self.pxrf_mean_value != None:
            if self._sim_mode:
                rover_x, rover_y, rover_z = FakeGPSPublisher.get_rover_pos()
                pos = self.conversion.tf2map(rover_x, rover_y)
                r,c = self.conversion.map2grid(pos[0], pos[1])
                rospy.loginfo(f"{Back.YELLOW}{Fore.BLACK} | Updating GPR with value={self.pxrf_mean_value} at (TF Map|Map|Grid): {(rover_x, rover_y)} | {pos} | {(r,c)} {Style.RESET_ALL}")
            else:
                pos = self.conversion.gps2map(self.lat, self.lon)
                r,c = self.conversion.map2grid(pos[0], pos[1])
                rospy.loginfo(f"{Back.YELLOW}{Fore.BLACK} | Updating GPR with value={self.pxrf_mean_value} at (GPS|Map|Grid): {(self.lat, self.lon)} | {pos} | {(r,c)} {Style.RESET_ALL}")
            self.adaptiveROS.update(r, c, self.pxrf_mean_value)
        # reset
        self.pxrf_complete = False
        self.pxrf_mean_value = None
        
        self.nextScanLoc = self.adaptiveROS.predict()

        if self._sim_mode:
            self.adaptiveROS.pub_plot_img(self.conversion.origin_tf, self.image_pub, self.tf_broadcaster)
        if self.show_plot:
            self.adaptiveROS.plot()

        self.nav_goal_map = self.conversion.grid2map(self.nextScanLoc[0], self.nextScanLoc[1])
        if self._sim_mode:
            self.nav_goal_tf_map = self.conversion.map2tf(self.nav_goal_map[0], self.nav_goal_map[1])
            rospy.loginfo(f"{Back.GREEN}{Fore.BLACK} | Sending Adaptive Algorithm Location (TF Map|Map|Grid): {self.nav_goal_tf_map} | {self.nav_goal_map} | {self.nextScanLoc} {Style.RESET_ALL}")
        else:
            self.nav_goal_gps = self.conversion.map2gps(self.nav_goal_map[0], self.nav_goal_map[1])
            self.send_location_to_GUI(self.nav_goal_gps[0], self.nav_goal_gps[1])
            rospy.loginfo(f"{Back.GREEN}{Fore.BLACK} | Sending Adaptive Algorithm Location (GPS|Map|Grid): {self.nav_goal_gps} | {self.nav_goal_map} | {self.nextScanLoc} {Style.RESET_ALL}")
            

        self.update_status(RECEIVED_NEXT_SCAN_LOC)

    def run_grid_algo(self):
        self.update_status(RUNNING_GRID_ALGO)
        self.pxrf_complete = False
        self.pxrf_mean_value = None
        self.nextScanLoc = self.gridROS.next()
        self.nav_goal_map = self.conversion.grid2map(self.nextScanLoc[0], self.nextScanLoc[1])
        self.nav_goal_gps = self.conversion.map2gps(self.nav_goal_map[0], self.nav_goal_map[1])
        self.send_location_to_GUI(self.nav_goal_gps[0], self.nav_goal_gps[1])
        self.update_status(RECEIVED_NEXT_SCAN_LOC)

    
    def show(self, save_to_disk=False, filename="plot.png"):
        # self.env_map = normalization(self.env_map)
        # self.surface_mu = normalization(self.surface_mu)
        
        # self.env_map = standardization(self.env_map)
        # self.surface_mu = standardization(self.surface_mu)
        visualizer_recreate_real(
            sampled=self.adaptiveROS.sampled,
            surface_mu=self.adaptiveROS.mu,
            adaptive=self.adaptiveROS,
            predicted_mapsize=(self.adaptiveROS.size_x, self.adaptiveROS.size_y),
            save_to_disk=save_to_disk,
            filename=filename
        )
        
        rospy.loginfo(f"Sampled at {self.adaptiveROS.sampled[-1]} with value = {self.adaptiveROS.sampled_val[-1]}")
        rospy.loginfo(f"Adaptive Norm Range: {self.adaptiveROS.norm_range:.4f}")

    def fake_arm_and_pxrf(self):
        self.pxrf_complete = True
        self.pxrf_mean_value = self.fake_pxrf_values.pop(0)
        rospy.loginfo(f'PXRF Mean Value: {self.pxrf_mean_value}')
        
        self.update_status(ARM_RETURNED)
    
    def fake_pxrf(self):
        self.pxrf_complete = True
        self.pxrf_mean_value = self.fake_pxrf_values.pop(0)
        rospy.loginfo(f'PXRF Mean Value: {self.pxrf_mean_value}')
        self.update_status(FINISHED_SCAN)
        

if __name__ == "__main__":    
    manager = Manager(fake_hardware_flags=[FAKE_ARM, FAKE_PXRF], show_plot=False)
    manager.fake_pxrf_values = [i for i in range(100)]
    manager.run()
