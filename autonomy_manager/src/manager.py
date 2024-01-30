#!/usr/bin/env python3
import rospy
from autonomy_manager.msg import ManagerStatus
from autonomy_manager.srv import DeployAutonomy, NavigateGPS, RunSensorPrep, Complete, ScanData, Waypoints
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from dummy_services import dummy_search
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from adaptiveROS import adaptiveROS
from gridROS import gridROS
from boundaryConversion import conversion
from sensor_msgs.msg import Joy

RECEIVED_SEARCH_AREA = 'received search area'
RECEIVED_NEXT_SCAN_LOC = 'received next scan loc'
ARRIVED_AT_SCAN_LOC = 'arrived at scan loc'
FINISHED_RAKING = 'finished raking'
FINISHED_SCAN = 'finished scan'
RAKING = 'raking'
NAVIGATION_TO_SCAN_LOC = 'navigating to scan loc'
RUNNING_SEARCH_ALGO = 'running search algo'
STANDBY = 'standby'
WAITING_FOR_CALIBRATION_TO_FINISH = 'waiting for calibration to finish'
RUNNING_GRID_ALGO = 'running grid algo'
RUNNING_WAYPOINT_ALGO = 'running waypoint algorithm'
SCANNING = 'scanning'

class Manager(object):
    def __init__(self):
        rospy.init_node('manager', anonymous=True)
        rospy.sleep(1)
        self.load_ros_params()

        self.statusPub = rospy.Publisher(self._status_topic, ManagerStatus, queue_size=10,latch=True)
        self.update_status(STANDBY)
        rospy.Subscriber(self._sensor_prep_status_topic,Bool,self.check_sensor_prep_status)
        rospy.Subscriber(self._joy_topic, Joy, self.manual_behavior_skip)
        self.isOverride = False
        #########################change it back to /gps_avg#########################################################
        rospy.Subscriber(self._gps_topic, Odometry, self.gps_callback)
        self.sensorPrep = rospy.ServiceProxy(self._sensor_prep_service_name, RunSensorPrep)
        #intialize adaptive sampling class and conversion class
        self.adaptiveROS = None
        self.gridROS = None
        self.conversion = Conversion()
        self.searchBoundary = []
        self.waypoints = []
        self.grid_sampling = False
        self.adaptive_sampling = False
        self.waypoint_sampling = False
        rospy.set_param('grid', False)
        rospy.set_param('adaptive', False)
        rospy.set_param('waypoint', False)

        #######################change it to None#################################
        self.lat = None
        self.lon = None
        self.goalComplete = False
        self.pxrfComplete = False
        self.value = -1
        self.gps = None
        deploy_service = rospy.Service(self._deploy_service_name, DeployAutonomy,self.deploy_service)
        goal_reach = rospy.Service(self._goal_reach_service, Complete, self.goal_reach)
        clear = rospy.Service(self._clear_service_name, Complete, self.clear)
        pxrf = rospy.Service(self._pxrf_service_name, ScanData, self.pxrf)
        waypoints = rospy.Service(self._waypoints_service_name, Waypoints, self.set_waypoints)

        # wait until the gps calibration is finished
        while self.lon is None or self.lat is None: 
             self.update_status(WAITING_FOR_CALIBRATION_TO_FINISH)
             print(self.status)
             rospy.sleep(1)
        
        self.update_status(STANDBY)

        # get ros param 
        print("Waiting to start")
        while self.grid_sampling == False and self.adaptive_sampling == False and self.waypoint_sampling == False:
            self.grid_sampling = rospy.get_param('grid', False)
            self.adaptive_sampling = rospy.get_param('adaptive', False)
            self.waypoint_sampling = rospy.get_param('waypoint', False)
            self.number_points = rospy.get_param('number_points', 9)

    def run(self):
        # publish status
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            print(self.status)
            if self.status == RECEIVED_SEARCH_AREA:
                print("receive")
                if self.adaptive_sampling:
                    self.run_search_algo()
                elif self.grid_sampling:
                    self.run_grid_algo()
                elif self.waypoint_sampling:
                    self.run_waypoint_algo()
                print(self.nextScanLoc)
            elif self.status == RECEIVED_NEXT_SCAN_LOC:
                print("next")
                self.navigate_to_scan_loc()
            elif self.status == ARRIVED_AT_SCAN_LOC:
                print("arrive")
                self.run_sensor_prep()
            elif self.status == FINISHED_RAKING:
                print("scan")
                runScan = True
                for i in range(100):
                    rospy.sleep(0.1)
                    if self.status != FINISHED_RAKING:
                        runScan = False
                        break
                if runScan:
                    self.scan()
            elif self.status == FINISHED_SCAN:
                print("algo")
                if self.grid_sampling:
                    self.run_grid_algo()
                elif self.adaptive_sampling:
                    self.run_search_algo()
                elif self.waypoint_sampling:
                    self.run_waypoint_algo()
            rate.sleep()

    def load_ros_params(self):
        # Load topic names into params
        self._status_topic = rospy.get_param('status_topic')
        self._sensor_prep_status_topic = rospy.get_param('sensor_prep_status_topic')
        self._joy_topic = rospy.get_param('joy_topic')
        self._gps_topic = rospy.get_param('gps_topic')
        
        # Load service names into params
        self._sensor_prep_service_name = rospy.get_param('sensor_prep_service_name')
        self._deploy_service_name = rospy.get_param('deploy_service_name')
        self._goal_reach_service_name = rospy.get_param('goal_reach_service_name')
        self._clear_service_name = rospy.get_param('clear_service_name')
        self._pxrf_service_name = rospy.get_param('pxrf_service_name')
        self._waypoints_service_name = rospy.get_param('waypoints_service_name')
        self._grid_points_service_name = rospy.get_param('grid_points_service_name')
        self._next_goal_service_name = rospy.get_param('next_goal_service_name')
        self._next_goal_nav_service_name = rospy.get_param('next_goal_nav_service_name')

    def run_waypoint_algo(self):
        self.update_status(RUNNING_WAYPOINT_ALGO)
        self.pxrfComplete = False
        self.value = None
        self.nextScanLoc = self.waypoints.pop(0)
        self.send_location(self.nextScanLoc[0],self.nextScanLoc[1])
        self.gps = [self.nextScanLoc[0],self.nextScanLoc[1]]
        self.update_status(RECEIVED_NEXT_SCAN_LOC)

    def manual_behavior_skip(self,data):
        self.isOverride = data.buttons[5] == 1
        if data.buttons[1]>0 and (not hasattr(self,'lastSkipButtonStatus') or not self.lastSkipButtonStatus):
            if self.status == RAKING:
                self.sensorPrep(False)
                self.update_status(FINISHED_RAKING)
            elif self.status == NAVIGATION_TO_SCAN_LOC:
                cancel_goal = rospy.ServiceProxy('cancel_goal',NavigateGPS)
                cancel_goal(0,0)
                self.update_status(ARRIVED_AT_SCAN_LOC)
            elif self.status == FINISHED_RAKING:
                self.update_status(FINISHED_SCAN)
        self.lastSkipButtonStatus = data.buttons[1]>0

    def scan(self):
        self.update_status(SCANNING)
        #call ros service to start scanning 
        #rospy.wait_for_service('scan_start')
        try:
            scan = rospy.ServiceProxy('scan_start', Complete)
            res = scan(True)
        except rospy.ServiceException as e:
            print("scan failed")

        while self.pxrfComplete == False:
            print("scan in progress")
            rospy.sleep(1)
        print("scan complete")
        lowerPXRF = rospy.ServiceProxy('/deploy_sensor',SetBool)
        lowerPXRF(False)
        rospy.sleep(2)
        self.update_status(FINISHED_SCAN)

    def pxrf(self, data):
        if data.status == True:
            self.pxrfComplete = True
            self.value = data.mean
            print("the value is :" + str(self.value))
        else:
            self.pxrfComplete = False
        return True

    def deploy_service(self,data):
        # data.boundart_lServiceat and data.boundary_lon lists, put then in the format of [[lat1,lon1],[lat2,lon2],...]
        for i in range(len(data.boundary_lat)):
            self.searchBoundary.append([data.boundary_lat[i],data.boundary_lon[i]])
        
        #initialize the zone, define boundary in utm coordinates 
        self.conversion.get_zone(self.lat, self.lon)
        boundary_utm_offset = self.conversion.boundaryConversion(self.searchBoundary)
        #when you receive the search area, define the robot position as the starting point, make sure to drive the 
        # the robot into the boundary first 
        startx, starty = self.conversion.gps2map(self.lat,self.lon)
        self.adaptiveROS = adaptiveROS(self.conversion.width, self.conversion.height, [startx, starty], self.number_points)
        self.adaptiveROS.updateBoundary(boundary_utm_offset)
        self.gridROS = gridROS(self.conversion.width, self.conversion.height, [0, 0], self.number_points)
        #self.gridROS.updateBoundary(boundary_utm_offset)
        # call ros service to pass all the grid points
        lat = []
        lon = []
        for i in range(len(self.gridROS.grid_points)):
            gps = self.conversion.map2gps(self.gridROS.grid_points[i][0],self.gridROS.grid_points[i][1])
            lat.append(gps[0])
            lon.append(gps[1])
        print(len(lat))
        try:
            grid_points = rospy.ServiceProxy(self._grid_points_service_name, Waypoints)
            res = grid_points(lat,lon)
        except rospy.ServiceException as e:
            print(e)
            print("grid points display failed")

        #self.gridROS.updateBoundary(boundary_utm_offset)
        #print(boundary_utm_offset)
        self.update_status(RECEIVED_SEARCH_AREA)
        return True
    
    def set_waypoints(self,data):
        self.waypoints = []
        for i in range(len(data.waypoints_lat)):
            self.waypoints.append([data.waypoints_lat[i],data.waypoints_lon[i]])
        # fake search area to faciliate the state machine
        print("received waypoints")
        self.update_status(RECEIVED_SEARCH_AREA)
        return True
    
    def run_search_algo(self):
        self.update_status(RUNNING_SEARCH_ALGO)
        if self.pxrfComplete == True and self.value != -1:
            pos = self.conversion.gps2map(self.lat, self.lon)
            self.adaptiveROS.update(pos[0], pos[1], self.value)
        #reset
        self.pxrfComplete = False
        self.value = None
        #predict the next location
        print(self.adaptiveROS)
        self.nextScanLoc = self.adaptiveROS.predict()
        self.gps = self.conversion.map2gps(self.nextScanLoc[0],self.nextScanLoc[1])
        print("sending")
        self.send_location(self.gps[0],self.gps[1])
        self.update_status(RECEIVED_NEXT_SCAN_LOC)

    def run_grid_algo(self):
        self.update_status(RUNNING_GRID_ALGO)
        self.pxrfComplete = False
        self.value = None
        self.nextScanLoc = self.gridROS.next()
        self.gps = self.conversion.map2gps(self.nextScanLoc[0],self.nextScanLoc[1])
        self.send_location(self.gps[0],self.gps[1])
        self.update_status(RECEIVED_NEXT_SCAN_LOC)

    def navigate_to_scan_loc(self):
        self.update_status(NAVIGATION_TO_SCAN_LOC)
        #self.navigation(self.nextScanLoc[0],self.nextScanLoc[1])
        self.send_nav(self.gps[0], self.gps[1])

    def run_sensor_prep(self):
        self.sensorPrep(True)
        self.update_status(RAKING)
        
    def check_sensor_prep_status(self,data):
        if data.data == False and self.status == RAKING and not self.isOverride:
            self.update_status(FINISHED_RAKING)

    def update_status(self,newStatus):
        self.status = newStatus
        msg = ManagerStatus()
        msg.status = self.status
        msg.header.stamp = rospy.Time.now()
        self.statusPub.publish(msg)
    
    def gps_callback(self,data):
        self.lat = data.pose.pose.position.y 
        self.lon = data.pose.pose.position.x 
    
    def send_location(self,x,y):
        #rospy.wait_for_service('next_goal')
        try:
            next_goal = rospy.ServiceProxy(self._next_goal_service_name, NavigateGPS)
            res = next_goal(x,y)
        except rospy.ServiceException as e:
            print("location failed")
        
    def send_nav(self,x,y):
        #note x,y are in gps coordinate
        #rospy.wait_for_service('next_goal_nav')
        try:
            next_goal = rospy.ServiceProxy(self._next_goal_nav_service_name, NavigateGPS)
            res = next_goal(x,y)
        except rospy.ServiceException as e:
            print("Navigation failed")

    def goal_reach(self, data):
        if data.status == True:
            self.goalComplete = True
            self.update_status(ARRIVED_AT_SCAN_LOC) 
        else:
            self.goalComplete = False
        
        return True

    def clear(self, data):
        if data.status == True:
            print("clear")
            self.adaptiveROS = None
            self.gridROS = None
            self.conversion = Conversion()
            self.searchBoundary = []

        return True
            
if __name__ == '__main__':
    manager = Manager()
    manager.run()
