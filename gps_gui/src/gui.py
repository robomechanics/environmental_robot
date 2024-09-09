#!/usr/bin/python3
import rospy
import sys
import os
import numpy as np

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from std_msgs.msg import String, Bool, Int32
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_srvs.srv import SetBool
from sensor_msgs.msg import NavSatFix
from autonomy_manager.msg import ManagerStatus
from tile import TileMap
import actionlib
from pxrf.msg import CompletedScanData
import rospkg
rospack = rospkg.RosPack()
pxrf_path = rospack.get_path('pxrf')
sys.path.insert(0, os.path.abspath(os.path.join(pxrf_path, "scripts")))
from plot import generate_plot
from autonomy_manager.srv import NavigateGPS, SetSearchBoundary, Complete, Waypoints
from tf.transformations import euler_from_quaternion
import tf
import argparse
from gui_utils import read_location, PlotWithClick, PolyLineROINoHover
from copy import deepcopy
import qdarktheme
from move_base_msgs.msg import MoveBaseAction
import rosnode
from gps_gui.srv import SetString
from visualization_msgs.msg import Marker, MarkerArray
from roverrobotics_description.srv import PointsToCoords

from env_utils.algo_constants import *
from env_utils.pxrf_utils import PXRF

qss = """
QPushButton {
    font-size: 11pt; 
    font-weight: 400;  
}"""


class GpsNavigationGui:
    def __init__(self, lat, lon, zoom, width, height):
        # Load ROS Params
        self.loadROSParams()
        
        # PXRF
        self.scanData = CompletedScanData()
        self.scanData.file_name = os.path.expanduser(self._pxrf_test_results_file) # test file if no sample was made yet
        self.pxrfManualSampleRunning = False
        self.numMeasurements = {
            ALGO_ADAPTIVE: 0,
            ALGO_GRID: 0,
            ALGO_WAYPOINT: 0,
            ALGO_MANUAL: 0,
        }
        
        self.pxrf = PXRF(self.scanCompletedCallback)
        
        # Set variables
        self.quaternion = tf.transformations.random_quaternion()
        self.prev_lat = 0
        self.prev_lon = 0
        self.prev_heading = 0
        self.latitude = None
        self.longitude = None

        # Get the map
        self.satMap = TileMap(coord=(lat, lon), dim=(width, height), zoom=zoom)

        # init widget
        pg.setConfigOption('imageAxisOrder', 'row-major')
        self.widget = pg.LayoutWidget()

        # Add plot for map
        satGUI = pg.GraphicsLayoutWidget()
        self.widget.addWidget(satGUI, row=0, col=0, colspan=10)
        # satGUI.setBackground('black')
        self.click_plot = PlotWithClick()
        satGUI.addItem(self.click_plot)

        # Show satellite image
        img = pg.ImageItem(self.satMap.map_array)
        img.setBorder({'color': 'w', 'width': 3})
        self.click_plot.addItem(img)
        #self.click_plot.showAxes(True)
        self.click_plot.setAspectLocked()
        self.click_plot.invertY()

        # Add arrow to show robot
        self.robotArrow = pg.ArrowItem(headLen=40, tipAngle=30, brush='r')
        self.robotArrow.setStyle(angle = 90)
        self.click_plot.addItem(self.robotArrow)

        # init ROI for editing path
        self.pathRoi = PolyLineROINoHover([], closed=False)

        # init waypoints display
        self.click_plot.addItem(self.pathRoi)

        # init plot for path
        self.pathPlotPoints: list[list] = []
        self.pathGPS = []
        self.boundaryPath = []
        self.heading = 0
        self.waypointsGPS = []
        self.waypointsPath = []
        self.pathPlot = self.click_plot.plot(symbolBrush=(255, 0, 255))
        self.boundaryPlot = self.click_plot.plot(symbolBrush=(255, 255, 255))
        self.currentGoalMarker = self.click_plot.plot(symbolBrush=(0, 0, 255))
        self.pathIndex = 0

        # interaction for adding path points
        self.click_plot.click_handlers = [self.addROIPoint]

        # Robot's history (path measured by gps)
        self.lastHistoryDrawTime = rospy.Time.now() - rospy.Duration(10)
        self.lastRobotDrawTime = rospy.Time.now() - rospy.Duration(10)
        self.lastGPSUpdateTime = rospy.Time.now() - rospy.Duration(10)
        self.historyPoints: list[list[int]] = []
        self.historyPlot = self.click_plot.plot(pen=pg.mkPen('g', width=2))
        self.setHistory()
        
        self.setupWidgets()

        self.setupROS()
        
    # This function converts the current path from gps coordinates to pixels
    def gpsToPixels(self):
        self.pathPlotPoints = []
        for gpsPoint in self.pathGPS:
            lat = gpsPoint[0]
            lon = gpsPoint[1]
            point = list(self.satMap.coord2Pixel(lat, lon))
            self.pathPlotPoints.append(point)

    # This fuction converts the current path from pixels to gps coordinates
    def pixelsToGps(self, pathPlotPoints):
        pathGPS = []
        for point in pathPlotPoints:
            gpsPoint = list(self.satMap.pixel2Coord(point))
            gpsPoint.append(1)
            pathGPS.append(gpsPoint)
        return pathGPS
    
    def addMarkerAt(self, lat: float, lon: float, label=None, size=20):
        pos = self.satMap.coord2Pixel(lat, lon)
        marker = pg.TargetItem(
            pos=pos,
            movable=False,
            size=size,
            label=label)

        self.click_plot.addItem(marker)

    def setupROS(self):
        # Services
        # self.parking_brake = rospy.ServiceProxy(self._parking_brake_service, SetBool)
        self.nextPoint = rospy.Service(self._next_point_service, NavigateGPS, self.onNextGoalUpdate)
        self.grid_points = rospy.Service(self._grid_points_service, Waypoints, self.onGridPoints)
        
        # Publishers
        #self.navigation_sub = rospy.Subscriber('/gps_navigation/current_goal', PoseStamped, self.readNavigation) # get status of navigation controller
        self.goalPub = rospy.Publisher(self._goal_pub_topic, PoseStamped, queue_size=5)
        
        self.estop_enable_publisher = rospy.Publisher(
            self._estop_enable_topic, Bool, queue_size=1
        )
        self.estop_reset_publisher = rospy.Publisher(
            self._estop_reset_topic, Bool, queue_size=1
        )

        # Subscribers        
        self.locationSub = rospy.Subscriber(self._gps_moving_avg_topic, NavSatFix, self.onGpsUpdate) # Use GPS moving avg llh position
        # self.location_sub = rospy.Subscriber(self._gps_sub_topic, NavSatFix, self.onGpsUpdate) # Use GPS llh position
        
        # self.scanCompletedSub = rospy.Subscriber(self._scan_recorded_to_disk_topic, CompletedScanData, self.scanCompletedCallback)
        self.gpsSub = rospy.Subscriber(self._location_sub_topic, Odometry, self.robotUpdate) # plotRobotPosition
        self.statusSub = rospy.Subscriber(self._status_sub_topic, ManagerStatus, self.managerStatusUpdate)
        #self.next_goal_sub = rospy.Subscriber('/next_goal', NavSatFix, self.onNextGoalUpdate) # display the next goal on the map
        
        self.roverBatterySub = rospy.Subscriber(self._rover_battery_percentage_topic, Int32, self.roverBatteryCallback)
        self.lipoBatterySub = rospy.Subscriber(self._lipo_battery_percentage_topic, String, self.lipoBatteryCallback)
    
        self.rvizPoints = []
        if self._sim_mode:
            self.clickedPointSub = rospy.Subscriber("/clicked_point", PointStamped, self.onRvizClickedPoint)
            self.rvizMarkerPub = rospy.Publisher("/exploration_polygon_marker", Marker, queue_size=10)
            self.points2coordsSrv = rospy.ServiceProxy('/fake_gps/points_to_coords', PointsToCoords)

    def loadROSParams(self):
        # Load topic names into params
        self._location_sub_topic = rospy.get_param('gq7_ekf_odom_map_topic')
        self._gps_sub_topic = rospy.get_param('gq7_ekf_llh_topic')
        self._pxrf_response_topic = rospy.get_param("pxrf_response_topic")
        self._gps_moving_avg_topic = rospy.get_param("gps_moving_avg_topic")
        self._goal_pub_topic = rospy.get_param('goal_pub_topic')
        self._status_sub_topic = rospy.get_param('status_topic')
        self._manager_run_loop_service_name = rospy.get_param("manager_run_loop_service_name")
        self._scan_recorded_to_disk_topic = rospy.get_param("scan_recorded_to_disk_topic")
        self._rover_battery_percentage_topic = rospy.get_param("rover_battery_percentage_topic")
        self._lipo_battery_percentage_topic = rospy.get_param("lipo_battery_percentage_topic")
        self._is_arm_in_home_pose_param_name = rospy.get_param("is_arm_in_home_pose_param_name")
        self._algorithm_type_param_name = rospy.get_param("algorithm_type_param_name")
        
        # Load service names into params
        # self._parking_brake_service = rospy.get_param('parking_break_service_name')
        self._next_point_service = rospy.get_param('next_goal_to_GUI_service_name')
        self._grid_points_service = rospy.get_param('grid_points_service_name')
        self._set_search_boundary_name = rospy.get_param('set_search_boundary_name')
        self._lower_arm_service_name = rospy.get_param('lower_arm_service_name')
        self._start_scan_service_name = rospy.get_param('start_scan_service_name')
        self._fake_start_scan_service_name = rospy.get_param('fake_start_scan_service_name')
        self._clear_service_name = rospy.get_param('clear_service_name')
        self._waypoints_service_name = rospy.get_param("waypoints_service_name")
        self._move_base_action_server_name = rospy.get_param('move_base_action_server_name')
        
        # Load action client topic names
        self._pxrf_client_topic = rospy.get_param('pxrf_client_topic_name')
        self._estop_enable_topic = rospy.get_param("estop_enable_topic")
        self._estop_reset_topic = rospy.get_param("estop_reset_topic")
        
        # Load constants
        self._pxrf_test_results_file = rospy.get_param('pxrf_test_results_file')
        self._sim_mode = rospy.get_param('sim_mode')

    def setupWidgets(self):
        def clearHistory():
            self.setHistory(clear = True)

        # add buttons 
        clearHistoryBtn = QtWidgets.QPushButton('Clear History')
        clearHistoryBtn.setStyleSheet("color: orange")
        clearHistoryBtn.clicked.connect(clearHistory)
        
        clearPathBtn = QtWidgets.QPushButton('Clear Path')
        clearPathBtn.setStyleSheet("color: orange")
        clearPathBtn.clicked.connect(self.clearPath)
        
        self.editPathMode = False
        self.editPathBtn = QtWidgets.QPushButton('Edit Waypoints')
        self.editPathBtn.setStyleSheet("color: orange")
        self.editPathBtn.clicked.connect(self.toggleEditPathMode)
        
        # loadPathFileBtn = QtWidgets.QPushButton('Load Path')
        # loadPathFileBtn.setStyleSheet("color: orange")
        # loadPathFileBtn.clicked.connect(self.loadPathFile)
        # savePathBtn = QtWidgets.QPushButton('Save Path')
        # savePathBtn.setStyleSheet("color: orange")
        # savePathBtn.clicked.connect(self.savePath)
        
        self.startPauseBtn = QtWidgets.QPushButton('Start')
        self.startPauseBtn.setStyleSheet("color: lightgreen")
        self.startPauseBtn.clicked.connect(self.startPause)
        
        self.is_navigating = False
        
        self.eStop = False
        self.eStopBtn = QtWidgets.QPushButton('E-STOP')
        self.eStopBtn.setStyleSheet("color: red")
        self.eStopBtn.clicked.connect(self.eStopCallback)
        
        self.stopStatus = True
        # self.parking_brake(self.stopStatus)
        
        # self.parkBtn = QtWidgets.QPushButton('PARK ON')
        # self.parkBtn.setStyleSheet("background-color : red")
        # self.parkBtn.clicked.connect(self.toggle_brake)
        
        self.sampleBtn = QtWidgets.QPushButton('Sample')
        self.sampleBtn.setStyleSheet("color: lightblue")
        self.sampleBtn.clicked.connect(self.togglePxrfCollection)

        self.fakeSampleBtn = QtWidgets.QPushButton('Fake Sample')
        self.fakeSampleBtn.setStyleSheet("color: lightblue")
        self.fakeSampleBtn.clicked.connect(self.toggleFakePxrfCollection)
        
        self.ArmBtn = QtWidgets.QPushButton('Toggle Arm')
        self.ArmBtn.setStyleSheet("color: lightblue")
        self.ArmBtn.clicked.connect(self.toggleArm)

        
        self.statusGPS = QtWidgets.QLineEdit()
        self.statusGPS.setText('GPS Connecting...')
        self.statusGPS.setReadOnly(False)
        
        self.showPxrfBtn = QtWidgets.QPushButton('PXRF Results')
        self.showPxrfBtn.setStyleSheet("color: lightblue")
        self.showPxrfBtn.clicked.connect(self.showPXRFResults)
        
        
        self.statusManager = QtWidgets.QLineEdit()
        self.statusManager.setText('Waiting for manager...')
        self.statusManager.setReadOnly(True)
        
        # set the boundary on the map 
        self.editBoundaryMode = False
        self.addBoundaryBtn = QtWidgets.QPushButton('Edit Bound')
        self.addBoundaryBtn.setStyleSheet("color: yellow")

        self.addBoundaryBtn.clicked.connect(self.toggleEditBoundaryMode)
        
        self.resetBtn = QtWidgets.QPushButton('Reset')
        self.resetBtn.setStyleSheet("color: yellow")
        self.resetBtn.clicked.connect(self.reset)
        
        self.adaptive = False
        self.adaptiveBtn = QtWidgets.QPushButton('Start Adaptive')
        self.adaptiveBtn.setStyleSheet("color: yellow")
        self.adaptiveBtn.clicked.connect(self.toggleAdaptive)
        
        self.grid = False
        self.gridBtn = QtWidgets.QPushButton('Start Grid')
        self.gridBtn.setStyleSheet("color: yellow")
        self.gridBtn.clicked.connect(self.toggleGrid)
        
        self.managerComboBox = QtWidgets.QComboBox()
        self.managerComboBox.addItems(['State Step', 'Sample Step', 'Continuous'])
        self.managerComboBox.setStyleSheet("color: lightgreen")
        self.managerComboBox.currentTextChanged.connect(self.managerComboBoxChanged)
        
        self.managerStepOnceBtn = QtWidgets.QPushButton('Manager Step')
        self.managerStepOnceBtn.setStyleSheet("color: lightgreen")
        self.managerStepOnceBtn.clicked.connect(self.managerStepOnce)
        
        self.statusDetailed = QtWidgets.QLineEdit("")
        self.statusDetailed.setReadOnly(True)
        
        self.statusRoverBattery = QtWidgets.QLineEdit("XX")
        self.statusRoverBattery.setStyleSheet("background-color: lightpurple")
        self.statusRoverBattery.setReadOnly(True)
        
        self.statusLIPOBattery = QtWidgets.QLineEdit("XX")
        self.statusLIPOBattery.setStyleSheet("background-color: lightpurple")
        self.statusLIPOBattery.setReadOnly(True)
        
        self.cancelMBGaolsBtn = QtWidgets.QPushButton('Cancel Goals')
        self.cancelMBGaolsBtn.setStyleSheet("color: red")
        self.cancelMBGaolsBtn.clicked.connect(self.cancelMoveBaseGoals)

        # add buttons to the layout
        self.widget.addWidget(self.statusGPS,          row=1, col=0, colspan=1)
        self.widget.addWidget(self.statusManager,      row=1, col=1, colspan=3)
        self.widget.addWidget(self.statusDetailed,     row=1, col=4, colspan=3)
        self.widget.addWidget(self.statusRoverBattery, row=1, col=7, colspan=1)
        self.widget.addWidget(self.statusLIPOBattery,  row=1, col=8, colspan=1)
        
        self.widget.addWidget(self.editPathBtn,        row=2, col=0, colspan=2)
        self.widget.addWidget(clearPathBtn,            row=2, col=2, colspan=2)
        self.widget.addWidget(clearHistoryBtn,         row=2, col=4, colspan=2)
        # self.widget.addWidget(loadPathFileBtn,       row=2, col=2, colspan=1)
        # self.widget.addWidget(savePathBtn,           row=2, col=3, colspan=1)
        self.widget.addWidget(self.startPauseBtn,      row=2, col=6, colspan=2)
        
        self.widget.addWidget(self.adaptiveBtn,        row=3, col=0, colspan=2)
        self.widget.addWidget(self.resetBtn,           row=3, col=2, colspan=2)
        self.widget.addWidget(self.addBoundaryBtn,     row=3, col=4, colspan=2)
        self.widget.addWidget(self.managerStepOnceBtn, row=3, col=6, colspan=1)
        self.widget.addWidget(self.managerComboBox,    row=3, col=7, colspan=1)

        # self.widget.addWidget(self.parkBtn,          row=4, col=6, colspan=2)
        self.widget.addWidget(self.sampleBtn,          row=4, col=0, colspan=1)
        self.widget.addWidget(self.fakeSampleBtn,      row=4, col=1, colspan=1)
        self.widget.addWidget(self.ArmBtn,             row=4, col=2, colspan=2)
        self.widget.addWidget(self.showPxrfBtn,        row=4, col=4, colspan=2)
        self.widget.addWidget(self.eStopBtn,           row=4, col=6, colspan=1)
        self.widget.addWidget(self.cancelMBGaolsBtn,   row=4, col=7, colspan=1)
        self.widget.addWidget(self.gridBtn,            row=4, col=8, colspan=2)

    def cancelMoveBaseGoals(self, data):
        mb_client = actionlib.SimpleActionClient(self._move_base_action_server_name, MoveBaseAction)
        rospy.loginfo(f"Cancelling all Movebase goals...")
        mb_client.cancel_all_goals()
        rospy.loginfo(f"Cancelled all Movebase goals!")
            
    def roverBatteryCallback(self, data: Int32):
        self.statusRoverBattery.setText(f"Rover: {data.data} %")
    
    def lipoBatteryCallback(self, data: String):
        battery_level = float(data.data)
        if battery_level:
            voltage = (0.08*battery_level) + 25.6
            self.statusLIPOBattery.setText(f"LIPO: {battery_level} ({voltage:.1} V) %")
        
    def showPXRFResults(self):
        generate_plot(self.scanData.file_name)
        
    def scanCompletedCallback(self, data):
        self.scanData = deepcopy(data)
        
        algorithm_type = rospy.get_param(self._algorithm_type_param_name)
        self.numMeasurements[algorithm_type] += 1
        self.sampleBtn.setText('Sample')
        self.statusDetailed.setText(f"{self.scanData.element}: {self.scanData.mean:.2}")
        
        if self.pxrfManualSampleRunning:
            self.pxrfManualSampleRunning = False
            
            self.addMarkerAt(self.prev_lat, self.prev_lon, f"#M{self.numMeasurements[algorithm_type]}: {self.scanData.mean:.2}")
            
            # Rest algorithm type
            rospy.set_param(self._algorithm_type_param_name, self.algorithm_type_before_manual_sample)
            rospy.sleep(1.0)
            
            # Raise Arm
            try:
                lower_arm_service = rospy.ServiceProxy(self._lower_arm_service_name, SetBool)
                lower_arm_service(False)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)
        else:
            self.addMarkerAt(self.prev_lat, self.prev_lon, f"#{self.numMeasurements[algorithm_type]}: {self.scanData.mean:.2}")
    
        return True
    
    def managerStatusUpdate(self, data:ManagerStatus):
        self.statusManager.setText(data.status)
        
    def managerComboBoxChanged(self, selected_item):
        if selected_item == "State Step" or selected_item == "Sample Step":
            self.managerStepOnceBtn.setEnabled(True)
        else:
            self.managerStepOnceBtn.setEnabled(False)
    
    def eStopCallback(self):
        self.eStop = not self.eStop
        
        if self.eStop:
            self.estop_enable_publisher.publish(True)
            self.eStopBtn.setStyleSheet("color: red")
            self.eStopBtn.setText('E-STOP ON')
        else:
            self.estop_reset_publisher.publish(True)
            self.eStopBtn.setStyleSheet("color: red")
            self.eStopBtn.setText('E-STOP')
            
    def managerStepOnce(self):
        try:
            managerStepOnce_client = rospy.ServiceProxy(self._manager_run_loop_service_name, SetString)
            managerStepOnce_client(str(self.managerComboBox.currentText()))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
     
    # This function adds points to roi (when user is editing path)
    def addROIPoint(self, point):
        if self.editPathMode or self.editBoundaryMode:
            points = [[handle['pos'].x(),handle['pos'].y()] for handle in self.pathRoi.handles]
            rospy.loginfo(f'Clicked at point: {point}')
            points.append(point)
            self.pathRoi.setPoints(points)
    
    #CHECK
    # This function clears robot's history path
    def setHistory(self, clear=False):
        if clear:
            self.historyPoints=[]
        
        if len(self.historyPoints) == 0:
            x = []
            y = []
        else:
            x, y = zip(*self.historyPoints)

        self.historyPlot.setData(x=list(x), y=list(y))
    
    # This function clears the current path
    def clearPath(self):
        self.pathRoi.setPoints([])
        self.pathPlotPoints= []
        self.pathGPS = []
        self.pathPlot.setData(x=[], y=[])
        self.updateGoalMarker()
        self.is_navigating = False
        self.startPauseBtn.setText('Start')
        self.startPauseBtn.setStyleSheet("color: orange")
        
    #This function clears the current boundary
    def reset(self):
        self.pathRoi.setPoints([])
        self.pathPlotPoints= []
        self.boundaryPath = []
        self.boundaryPlot.setData(x=[], y=[])
        self.pathPlot.setData(x=[], y=[])
        self.updateGoalMarker()
        self.clearMap()
    
    # this is a utility function to pass the boundary points
    def sendBoundary(self, boundary):
        rospy.loginfo(f" Sending Boundary Points:\n {boundary} \n----------------")
        try:
            sendBoundaryClient = rospy.ServiceProxy(self._set_search_boundary_name, SetSearchBoundary)
            if len(boundary) > 0:
                boundary.pop()
            lat = [float(lat[0]) for lat in boundary]
            lon = [float(lon[1]) for lon in boundary]
            boundary_type = 1 if self._sim_mode else 0 # Use map 1 (map) if in sim_mode else 0 (gps)
            res = sendBoundaryClient(lat,lon, boundary_type) 
            rospy.loginfo("Boundary Sent")
        except rospy.ServiceException as e:
            rospy.logerr("Boundary was not sent successfully: %s", e)

    def sendWaypoints(self, waypoints):
        #rospy.wait_for_service('/waypoints')
        try:
            sendWaypointsClient = rospy.ServiceProxy(self._waypoints_service_name, Waypoints)
            lat = [float(lat[0]) for lat in waypoints]
            lon = [float(lon[1]) for lon in waypoints]
            res = sendWaypointsClient(lat,lon)
            rospy.loginfo("Waypoints sent!")
        except rospy.ServiceException:
            rospy.logerr("Waypoints were not sent successfully")

    # this function turns on/off editing mode for the boundary
    def toggleEditBoundaryMode(self):
        if self.editPathMode:
            rospy.logwarn("Warning: Please finish editing the path first")
            return
        elif self.editBoundaryMode and self.pathRoi.handles == [] and self.rvizPoints == []:
            rospy.logwarn('Warning: No boundary to edit, please draw a boundary first')
            return

        self.editBoundaryMode = not self.editBoundaryMode
        
        if self.editBoundaryMode:
            self.addBoundaryBtn.setText('Confirm')
            self.pathRoi.setPoints([])
            #FIX: Below code might be unecessary
            for point in self.pathPlotPoints:
                self.addROIPoint(point)
        else:
            # When boundary is confirmed
            self.addBoundaryBtn.setText('Edit Bound')
            self.pathPlotPoints = []
            if self._sim_mode:
                self.pathPlotPoints = self.rvizPoints
                # Append the first point to the end to close the boundary
                self.pathPlotPoints.append(self.pathPlotPoints[0])
                self.boundaryPath = self.pathPlotPoints
                self.drawRvizPolygon(self.boundaryPath)
            else:
                for handle in self.pathRoi.handles:
                    pos = handle['pos']
                    self.pathPlotPoints.append([pos.x(), pos.y()])
            
                # Append the first point to the end to close the boundary
                self.pathPlotPoints.append(self.pathPlotPoints[0])

                self.boundaryPath = self.pixelsToGps(self.pathPlotPoints)

            x, y = zip(*self.pathPlotPoints)
            self.boundaryPlot.setData(x=list(x), y=list(y))

            self.pathRoi.setPoints([])
            self.pathPlotPoints = []
            self.rvizPoints = []
            self.sendBoundary(self.boundaryPath)


    def onRvizClickedPoint(self, msg):
        if not self.editBoundaryMode:
            rospy.logwarn("Received rviz /clicked_point but not in edit boundary mode! Aborting!")
            return
        p = msg.point
        point = [float(p.x), float(p.y)]
        self.rvizPoints.append(point)
        rospy.loginfo(f'Received rviz point: {point}')

    def drawRvizPolygon(self, points):
        points = np.array(points)
        # Create a Marker message
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "boundary"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # Use LINE_STRIP to draw a polygon
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Define the color and scale
        marker.scale.x = 0.1 
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Publish
        marker.points = [Point(x, y, 0) for (x,y) in points]
        self.rvizMarkerPub.publish(marker)

        # Draw the dimensions of square containing it
        marker.header.stamp = rospy.Time.now()
        marker.ns = "boundary_square"
        marker.id = 1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        min_x, max_x = min(points[:,0]), max(points[:,0])
        min_y, max_y = min(points[:,1]), max(points[:,1])
        width = max(max_x-min_x, max_y-min_y)
        square_points = [[min_x, min_y], # bottom left
                         [min_x+width, min_y], # bottom right
                         [min_x+width, min_y+width], # top right
                         [min_x, min_y+width]] # top left
        # Append the first point to the end to close the boundary
        square_points.append(square_points[0])

        # Publish
        marker.points = [Point(x, y, 0) for (x,y) in square_points]
        self.rvizMarkerPub.publish(marker)


    # This function turns on/off editing mode
    def toggleEditPathMode(self):
        if self.editBoundaryMode:
            rospy.logwarn("Warning: Please finish editing the boundary first")
            return
        #check if pathRoi is empty
        elif self.editPathMode and self.pathRoi.handles == []:
            rospy.logwarn('Warning: No path to edit, please draw a path first')
            return

        self.editPathMode = not self.editPathMode
        if self.editPathMode:
            self.editPathBtn.setText('Apply')
            self.pathRoi.setPoints([])
            for point in self.pathPlotPoints:
                self.addROIPoint(point)
        else:
            self.editPathBtn.setText('Edit Waypoints')
            self.pathPlotPoints = []
            for handle in self.pathRoi.handles:
                pos = handle['pos']
                self.pathPlotPoints.append([pos.x(), pos.y()])

            self.pathGPS = self.pixelsToGps(self.pathPlotPoints)
            x, y = zip(*self.pathPlotPoints)
            #size of x
            self.pathPlot.setData(x=list(x), y=list(y))
            rospy.loginfo(f"Waypoints: {self.pathGPS}")
            self.pathRoi.setPoints([])
            self.sendWaypoints(self.pathGPS)
    
    def toggleAdaptive(self):
        self.adaptive = not self.adaptive
        
        if self.adaptive:
            rospy.set_param(self._algorithm_type_param_name, ALGO_ADAPTIVE)
            self.adaptiveBtn.setText('Stop Adaptive')
        else:
            rospy.set_param(self._algorithm_type_param_name, ALGO_NONE)
            self.adaptiveBtn.setText('Start Adaptive')
    
    def toggleGrid(self):
        self.grid = not self.grid
        if self.grid:
            rospy.loginfo("grid mode")
            rospy.set_param(self._algorithm_type_param_name, ALGO_GRID)
            x, y = zip(*self.waypointsPath)
            self.pathPlot.setData(x=list(x), y=list(y))
            self.pathRoi.setPoints([])
            self.pathGPS = self.waypointsGPS
            self.waypointsPath = []
            self.gridBtn.setText('Stop Grid')
        else:
            rospy.set_param(self._algorithm_type_param_name, ALGO_NONE)
            self.gridBtn.setText('Start Grid')

    #CHECK
    # This function starts/ pauses the navigation
    def startPause(self):
        # if (len(self.pathGPS) == 0):
        #     return
        # self.is_navigating = not self.is_navigating
        # if self.is_navigating:
        #     self.startPauseBtn.setText('Pause')
        #     self.startPauseBtn.setStyleSheet("background-color : green")
        #     rospy.set_param('/waypoint', True)
        #     rospy.set_param('/adaptive', False)
        #     rospy.set_param('/grid', False)
        # elif self.pathIndex == 0:
        #     self.startPauseBtn.setText('Start')
        #     rospy.set_param('/waypoint', False)
        # else:
        #     self.startPauseBtn.setText('Continue')
        
        if self.is_navigating:
            self.startPauseBtn.setText('Pause')
            self.startPauseBtn.setStyleSheet("background-color : green")
            rospy.set_param(self._algorithm_type_param_name, ALGO_WAYPOINT)
        elif self.pathIndex == 0:
            self.startPauseBtn.setText('Start')
            rospy.set_param(self._algorithm_type_param_name, ALGO_NONE)
        else:
            self.startPauseBtn.setText('Continue')

    # This function updates the marked goal on the path
    def updateGoalMarker(self, point = None):
        if point == None:
            self.currentGoalMarker.setData(x=[], y=[])
        else:
            self.currentGoalMarker.setData(x=[point[0]], y=[point[1]])
       
    # This function is called by subscriber of gps sensor
    def robotUpdate(self, data: Odometry):
        if (rospy.Time.now() - self.lastRobotDrawTime).secs > 0.5:
            if self.latitude == None or self.longitude == None:
                return
            
            lat = self.latitude
            lon = self.longitude

            #calculate heading based on gps coordinates 
            pixX, pixY = self.satMap.coord2Pixel(lat, lon)
            
            # rospy.loginfo_throttle(10, "Robot Lat/Lon    : %s %s", self.latitude, self.longitude)
            # rospy.loginfo_throttle(10, "Tile Map Lat/Lon : %s %s", self.satMap.lat, self.satMap.lon)
            # rospy.loginfo_throttle(10, "Tile Map X/Y     : %s %s", self.satMap.x, self.satMap.y)
            # rospy.loginfo_once("Robot Pixel Pos  : %s %s", pixX, pixY)
            
            self.quaternion[0] = data.pose.pose.orientation.x
            self.quaternion[1] = data.pose.pose.orientation.y
            self.quaternion[2] = data.pose.pose.orientation.z
            self.quaternion[3] = data.pose.pose.orientation.w
            robotHeading = euler_from_quaternion(self.quaternion, "sxyz")[2]

            # if not self.robotArrow is None:
            self.robotArrow.setStyle(angle = 180 - (robotHeading*180.0/np.pi))
            self.robotArrow.setPos(pixX, pixY)
            self.robotArrow.update()
            self.lastRobotDrawTime = rospy.Time.now()
            
            if (rospy.Time.now() - self.lastHistoryDrawTime).secs > 5:
                self.historyPoints.append([pixX, pixY])
                self.setHistory()
                self.prev_lat = lat
                self.prev_lon = lon
                self.prev_heading = robotHeading
                
                self.lastHistoryDrawTime = rospy.Time.now()
                
    
    # This function updates the value of longitude and latitude information
    def onGpsUpdate(self, data: NavSatFix):
        self.latitude = data.latitude
        self.longitude = data.longitude
        
        if (rospy.Time.now() - self.lastGPSUpdateTime).secs > 5:
            self.statusGPS.setText("GPS:" + str(round(self.latitude,6)) + " | " + str(round(self.longitude, 6)))
            self.lastGPSUpdateTime = rospy.Time.now()

    #CHECK
    # This function updates the goal and displays it on the map
    def onNextGoalUpdate(self, req: NavigateGPS):
        if self.adaptive:
            rospy.loginfo("adaptive mode")
            self.pathGPS.append([req.goal_lat, req.goal_lon])
            self.pathPlotPoints.append(self.satMap.coord2Pixel(req.goal_lat, req.goal_lon))
            #self.pathPlot.setData(x = x_loc, y = y_loc)
            x, y = zip(*self.pathPlotPoints)
            self.pathPlot.setData(x=list(x), y=list(y))
            self.pathRoi.setPoints([])
            #self.pathPlotPoints = []
            point = self.satMap.coord2Pixel(req.goal_lat, req.goal_lon)
            self.updateGoalMarker(point)
        else:
            rospy.loginfo("Next point")
            point = self.satMap.coord2Pixel(req.goal_lat, req.goal_lon)
            self.updateGoalMarker(point)

        return True
    
    #CHECK 
    def onGridPoints(self, req: Waypoints):
        #display it on the map by setting the data of the pathPlot
        if True:
            rospy.loginfo("Printing grid points")
            self.waypointsGPS = []
            self.waypointsPath = []
            for i in range(len(req.waypoints_lat)):
                self.waypointsGPS.append([req.waypoints_lat[i], req.waypoints_lon[i]])
                self.waypointsPath.append(self.satMap.coord2Pixel(req.waypoints_lat[i], req.waypoints_lon[i]))
        return True

    def clearMap(self):
        try:
            clear_service_client = rospy.ServiceProxy(self._clear_service_name, Complete)
            res = clear_service_client(True)
        except rospy.ServiceException as e:
            rospy.loginfo("Reset Failed")

    def togglePxrfCollection(self):
        try:
            lower_arm_service = rospy.ServiceProxy(self._lower_arm_service_name, SetBool)
            lower_arm_service(True)
            
            self.algorithm_type_before_manual_sample = rospy.get_param(self._algorithm_type_param_name)
            rospy.set_param(self._algorithm_type_param_name, ALGO_MANUAL)
            rospy.sleep(1.0)
            rospy.loginfo(f"Algo Type: {rospy.get_param(self._algorithm_type_param_name)}")
            
            self.pxrf.start_scan()
            
            self.pxrfManualSampleRunning = True
            self.sampleBtn.setText("Collecting")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)
    
    def toggleFakePxrfCollection(self):
        try:   
            self.algorithm_type_before_manual_sample = rospy.get_param(self._algorithm_type_param_name)
            rospy.set_param(self._algorithm_type_param_name, ALGO_MANUAL)
            rospy.sleep(1.0)
            rospy.loginfo(f"Algo Type: {rospy.get_param(self._algorithm_type_param_name)}")
            
            fake_start_scan_service = rospy.ServiceProxy(self._fake_start_scan_service_name, Complete)
            fake_start_scan_service(True)

            
            self.fakeSampleBtn.setText("Collecting")
            rospy.sleep(1.0)
            self.fakeSampleBtn.setText("Fake Sample")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)
    
    def toggleArm(self):
        while '/arm_control' not in rosnode.get_node_names():
            rospy.loginfo("Waiting for Arm Control Node")
            rospy.sleep(1)
        self.is_arm_in_home_pose = rospy.get_param(
                self._is_arm_in_home_pose_param_name
                ) # Arm pose flag that persists across restarts
        
        rospy.loginfo(f"Arm in home Pose: {self.is_arm_in_home_pose}")
        
        try:
            lower_arm_service = rospy.ServiceProxy(self._lower_arm_service_name, SetBool)
            lower_arm_service(self.is_arm_in_home_pose)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Environmental Sensing GPS GUI')
    parser.add_argument("-o", "--option", type=int, default=3, 
                        help='1: Change Map. 2: New Map. 3: Continue.')

    args = parser.parse_args()

    rospy.init_node('gps_gui',anonymous=True)
    
    try:
       lat = float(rospy.get_param('~lat'))
       lon = float(rospy.get_param('~lon'))
       zoom = int(rospy.get_param('~zoom'))
       height = int(rospy.get_param('~height'))
       width = int(rospy.get_param('~width'))
    except:
        # Sser can input the location manually
        location_input = read_location(map_option=args.option)
        lat, lon = [float(n) for n in location_input[1:3]]
        zoom = int(location_input[3])
        width, height = [int(n) for n in location_input[4:6]]

    # Main window
    app = QtWidgets.QApplication([])
    
    # Set Color Scheme
    app.setStyleSheet(qdarktheme.load_stylesheet())
    qdarktheme.setup_theme(custom_colors={"primary": "#D0BCFF"}, additional_qss=qss)
    mw = QtWidgets.QMainWindow()
    gps_node = GpsNavigationGui(lat, lon, zoom, width, height)
    mw.setCentralWidget(gps_node.widget)
    mw.show()
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        try:
            QtWidgets.QApplication.instance().exec_()
        except:
            QtWidgets.QApplication.instance().exec()