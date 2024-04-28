#!/usr/bin/env python3
import rospy
from autonomy_manager.msg import ManagerStatus
from autonomy_manager.srv import (
    DeployAutonomy,
    NavigateGPS,
    RunSensorPrep,
    Complete,
    Waypoints,
)
from pxrf.msg import CompletedScanData
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from adaptiveROS import adaptiveROS
from gridROS import gridROS
from boundaryConversion import Conversion
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from pyproj import Transformer

INIT = "Initialization"
RECEIVED_SEARCH_AREA = "Received search area"
RECEIVED_NEXT_SCAN_LOC = "Received next scan loc"
ARRIVED_AT_SCAN_LOC = "Arrived at scan loc"
FINISHED_RAKING = "Finished raking"
FINISHED_SCAN = "Finished scan"
RAKING = "Raking"
NAVIGATION_TO_SCAN_LOC = "Navigating to scan loc"
RUNNING_SEARCH_ALGO = "Running search algo"
READY = "Ready"
WAITING_FOR_GPS_INIT = "Waiting for GPS init"
RUNNING_GRID_ALGO = "Running grid algo"
RUNNING_WAYPOINT_ALGO = "running waypoint algorithm"
SCANNING = "Scanning"
ARM_RETURNING = "Arm returning"
ARM_LOWERING = "Arm lowering"
ARM_RETURNED = "Arm returned"
ARM_LOWERED = "Arm lowered"


DEBUG_FLAG = False 

class Manager(object):
    def __init__(self):
        rospy.init_node("manager", anonymous=True)
        rospy.sleep(1)
        self.load_ros_params()

        self.statusPub = rospy.Publisher(
            self._status_topic, ManagerStatus, queue_size=10, latch=True
        )
        self.update_status(INIT)
        
        self.is_full_nav_achieved = False
        
        self._gps_sub = rospy.Subscriber(self._gq7_ekf_llh_topic, NavSatFix, self.gps_callback)
        self._scan_completed_sub = rospy.Subscriber(self._scan_completed_topic, CompletedScanData, self.pxrf_scan_completed_callback)
        
        self.sensorPrep = rospy.ServiceProxy(
            self._sensor_prep_service_name, RunSensorPrep
        )

        # Check if Full Nav achieved
        self.utm_odom_sub = rospy.Subscriber(
            self._gps_odom_topic, Odometry, self.gps_odom_callback
        )

        # intialize adaptive sampling class and conversion class
        self.adaptiveROS = None
        self.gridROS = None
        self.conversion = Conversion()
        self.searchBoundary = []
        self.waypoints = []
        self.grid_sampling = False
        self.adaptive_sampling = False
        self.waypoint_sampling = False

        #######################change it to None#################################
        self.lat = None
        self.lon = None
        self.nav_goal_complete = False
        self.pxrf_complete = False
        self.pxrf_mean_value = -1
        self.gps = None
        self.run_loop_flag = False
        
        self.transformer = Transformer.from_crs(self._crs_GPS, self._crs_UTM)
        
        self._set_search_boundary_service = rospy.Service(
            self._set_search_boundary_name, DeployAutonomy, self.set_search_boundary_callback
        )
        
        self._reset_service = rospy.Service(self._clear_service_name, Complete, self.reset_callback)
        
        self._waypoints_service = rospy.Service(
            self._waypoints_service_name, Waypoints, self.set_waypoints
        )
        self._run_loop_service = rospy.Service(
            self._run_loop_service_name, Trigger, self.run_loop_callback
        )

        # Action Services
        self.mb_client = actionlib.SimpleActionClient(self._move_base_action_server_name, MoveBaseAction)
        # print(" | Waiting for move_base server")
        # self.mb_client.wait_for_server()
        
        # wait until GPS Full Nav is achieved
        while not self.is_full_nav_achieved:
            self.update_status(WAITING_FOR_GPS_INIT)
            rospy.loginfo_throttle(3,"Waiting for GPS Initialization...")
            rospy.sleep(1)
            
        rospy.loginfo("GPS Full Navigation Achieved!") 
        self.utm_odom_sub.unregister()
        
        self.update_status(READY)

        # get ros param
        rospy.loginfo(" | Waiting to start (Choose a sampling algorithm)")
        while (
            self.grid_sampling == False
            and self.adaptive_sampling == False
            and self.waypoint_sampling == False
        ):
            self.grid_sampling = rospy.get_param("grid", False)
            self.adaptive_sampling = rospy.get_param("adaptive", False)
            self.waypoint_sampling = rospy.get_param("waypoint", False)
            self.number_points = rospy.get_param("number_points", 9)

        rospy.loginfo(" | Algorithm Set")
        rospy.loginfo("----------- READY -----------")
        
        
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
        self.run_loop_flag = False
        rospy.loginfo("----------- Manager Loop START-----------")
        
        if self.status == RECEIVED_SEARCH_AREA:
            # print(" | Recevied search area")
            if self.adaptive_sampling:
                self.run_search_algo()
            elif self.grid_sampling:
                self.run_grid_algo()
            elif self.waypoint_sampling:
                self.run_waypoint_algo()
            print(self.nextScanLoc)
        elif self.status == RECEIVED_NEXT_SCAN_LOC:
            # print(" | Receiced next scan location")
            self.navigate_to_scan_loc()
        elif self.status == ARRIVED_AT_SCAN_LOC:
            # print(" |Arrived at scan location")
            # self.run_sensor_prep()
            self.update_status(ARM_LOWERING)
        elif self.status == ARM_LOWERING:
            # print(" | Arm touchdown")
            self.arm_touchdown()
        elif self.status == ARM_RETURNING:
            # print(" | Arm return")
            self.arm_return()
        elif self.status == ARM_LOWERED or self.status == FINISHED_RAKING:
            self.scan()
        elif self.status == FINISHED_SCAN:
            # print("algo")
            if self.grid_sampling:
                self.run_grid_algo()
            elif self.adaptive_sampling:
                self.run_search_algo()
            elif self.waypoint_sampling:
                self.run_waypoint_algo()
                
        rospy.loginfo("----------- Manager Loop END -----------")
    
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._run_type == 0 or self.run_loop_flag == True:
                self.run_once()
            rate.sleep()
    
    def run_loop_callback(self, data):
        self.run_loop_flag = True

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
        self._gps_odom_topic = rospy.get_param("gps_odom_topic")
        self._scan_completed_topic = rospy.get_param("scan_completed_topic")
        
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
        
        self._start_utm_x_param = rospy.get_param("start_utm_x_param")
        self._start_utm_y_param = rospy.get_param("start_utm_y_param")
        self._run_type = rospy.get_param("manager_run_type")

    def scan(self):
        self.update_status(SCANNING)
        # call ros service to start scanning
        # rospy.wait_for_service('scan_start')
        try:
            scan = rospy.ServiceProxy(self._start_scan_service_name, Complete)
            res = scan(True)
        except rospy.ServiceException as e:
            print("Scan failed")

        if self.pxrf_complete == True:
            print("Scan Completed")

        self.update_status(FINISHED_SCAN)

    def pxrf_scan_completed_callback(self, data):
        if data.status == True:
            self.pxrf_complete = True
            self.pxrf_mean_value = data.mean
            print("PXRF Mean Value: " + str(self.pxrf_mean_value))
        else:
            self.pxrf_complete = False
        return True

    def set_search_boundary_callback(self, data):
        # data.boundary_lat and data.boundary_lon lists, put then in the format of [[lat1,lon1],[lat2,lon2],...]
        for i in range(len(data.boundary_lat)):
            self.searchBoundary.append([data.boundary_lat[i], data.boundary_lon[i]])

        
        # Initialize the zone, define boundary in utm coordinates
        self.conversion.get_zone(self.lat, self.lon)
        
        boundary_utm_offset = self.conversion.boundary_conversion(self.searchBoundary)
        
        # print("Data: \n", data)
        # print("boundary_utm_offset: \n", boundary_utm_offset)
        # print("conversion width and height: ", self.conversion.width, self.conversion.height)

        # When you receive the search area, define the robot position as the starting point, make sure to drive the
        # the robot into the boundary first
        
        startx, starty = self.conversion.gps2map(self.lat, self.lon)
        self.adaptiveROS = adaptiveROS(
            self.conversion.width,
            self.conversion.height,
            [startx, starty],
            self.number_points,
        )
        self.adaptiveROS.updateBoundary(boundary_utm_offset)
        self.gridROS = gridROS(
            self.conversion.width, self.conversion.height, [0, 0], self.number_points
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
        print("Length of points: ", len(lat))
        print("Grid Points: \n", lat, "\n", lon, "-------\n")
        
        try:
            grid_points = rospy.ServiceProxy(self._grid_points_service_name, Waypoints)
            res = grid_points(lat, lon)
        except rospy.ServiceException as e:
            print(e)
            print("grid points display failed")

        # self.gridROS.updateBoundary(boundary_utm_offset)
        # print(boundary_utm_offset)
        self.update_status(RECEIVED_SEARCH_AREA)
        return True

    def set_waypoints(self, data):
        self.waypoints = []
        for i in range(len(data.waypoints_lat)):
            self.waypoints.append([data.waypoints_lat[i], data.waypoints_lon[i]])
        # fake search area to faciliate the state machine
        print("received waypoints")
        self.update_status(RECEIVED_SEARCH_AREA)
        return True

    def navigate_to_scan_loc(self):
        self.update_status(NAVIGATION_TO_SCAN_LOC)
        # self.navigation(self.nextScanLoc[0],self.nextScanLoc[1])
        self.publish_move_base_goal(self.gps[0], self.gps[1])

    def run_sensor_prep(self):
        self.sensorPrep(True)
        # TODO: Perform Sensor Prep
        self.update_status(RAKING)

    def update_status(self, newStatus, debug_flag=DEBUG_FLAG):
        self.status = newStatus
        msg = ManagerStatus()
        msg.status = self.status
        msg.header.stamp = rospy.Time.now()
        self.statusPub.publish(msg)
        if debug_flag:
            print ("Status: ", self.status)
            input("Press Enter to Continue")

    def gps_callback(self, data: NavSatFix):
        self.lat = data.latitude
        self.lon = data.longitude

    def send_location_to_GUI(self, x, y):
        # rospy.wait_for_service('next_goal')
        try:
            next_goal_to_GUI = rospy.ServiceProxy(self._next_goal_to_GUI_service_name, NavigateGPS)
            res = next_goal_to_GUI(x, y)
        except rospy.ServiceException as e:
            print("location failed")

    def publish_move_base_goal(self, lat, lon):
        #TODO: Orientation for goal
        self.goal_x_UTM, self.goal_y_UTM  = self.transformer.transform(lat, lon)
        
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
        print(" | Goal Sent to movebase...")
        wait = self.mb_client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.mb_client.get_result()

        print(" | Movebase Goal Reached")
        
        self.nav_goal_complete = True
        self.update_status(ARRIVED_AT_SCAN_LOC)

    def arm_return(self):
        self.update_status(ARM_RETURNING)
        try:
            lower_arm = rospy.ServiceProxy(self._lower_arm_service_name, SetBool)
            res = lower_arm(False)
        except rospy.ServiceException as e:
            print("Arm Return Failed")
        
        self.update_status(ARM_RETURNED)

    def arm_touchdown(self):
        try:
            lower_arm = rospy.ServiceProxy(self._lower_arm_service_name, SetBool)
            res = lower_arm(True)
        except rospy.ServiceException as e:
            print("Arm Touchdown Failed")
        
        self.update_status(ARM_LOWERED)
        
    def reset_callback(self, data):
        if data.status == True:
            print("| Reset ")
            self.adaptiveROS = None
            self.gridROS = None
            self.conversion = Conversion()
            self.searchBoundary = []

        return True

    def run_waypoint_algo(self):
        self.update_status(RUNNING_WAYPOINT_ALGO)
        self.pxrf_complete = False
        self.pxrf_mean_value = None
        self.nextScanLoc = self.waypoints.pop(0)
        self.send_location_to_GUI(self.nextScanLoc[0], self.nextScanLoc[1])
        self.gps = [self.nextScanLoc[0], self.nextScanLoc[1]]
        self.update_status(RECEIVED_NEXT_SCAN_LOC)

    def run_search_algo(self):
        self.update_status(RUNNING_SEARCH_ALGO)
        if self.pxrf_complete == True and self.pxrf_mean_value != -1:
            pos = self.conversion.gps2map(self.lat, self.lon)
            self.adaptiveROS.update(pos[0], pos[1], self.pxrf_mean_value)
        # reset
        self.pxrf_complete = False
        self.pxrf_mean_value = None
        
        self.nextScanLoc = self.adaptiveROS.predict()
        self.gps = self.conversion.map2gps(self.nextScanLoc[0], self.nextScanLoc[1])
        print(" | Sending Adaptive Algorithm Location: ", self.gps)
        self.send_location_to_GUI(self.gps[0], self.gps[1])
        self.update_status(RECEIVED_NEXT_SCAN_LOC)

    def run_grid_algo(self):
        self.update_status(RUNNING_GRID_ALGO)
        self.pxrf_complete = False
        self.pxrf_mean_value = None
        self.nextScanLoc = self.gridROS.next()
        self.gps = self.conversion.map2gps(self.nextScanLoc[0], self.nextScanLoc[1])
        self.send_location_to_GUI(self.gps[0], self.gps[1])
        self.update_status(RECEIVED_NEXT_SCAN_LOC)


if __name__ == "__main__":
    manager = Manager()
    manager.run()
