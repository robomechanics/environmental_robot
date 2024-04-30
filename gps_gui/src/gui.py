#!/usr/bin/python3
import rospy
import sys
import os
import numpy as np

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from std_msgs.msg import String, Bool
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import NavSatFix, NavSatStatus
from autonomy_manager.msg import ManagerStatus
from tile import TileMap
import actionlib
from pxrf.msg import CompletedScanData
import rospkg
rospack = rospkg.RosPack()
pxrf_path = rospack.get_path('pxrf')
sys.path.insert(0, os.path.abspath(os.path.join(pxrf_path, "scripts")))
from plot import generate_plot
from autonomy_manager.srv import NavigateGPS, DeployAutonomy, Complete, Waypoints
from tf.transformations import euler_from_quaternion
import tf
import argparse
from gui_utils import read_location, PlotWithClick, PolyLineROINoHover
from copy import deepcopy
import qdarktheme

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
        self.pxrfComplete = True
        self.pxrfRunning = False
        self.numMeasurements = 0
        
        # Set variables
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
        self.widget.addWidget(satGUI, row=0, col=0, colspan=8)
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

        # robot's history (path measured by gps)
        self.historyPoints: list[list[int]] = []
        self.historyPlot = self.click_plot.plot(pen=pg.mkPen('g', width=2))
        self.setHistory()

        self.setupROS()
        
        self.setupWidgets()

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
        self.parking_brake = rospy.ServiceProxy(self._parking_brake_service, SetBool)
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
        
        self.scanCompletedSub = rospy.Subscriber(self._scan_completed_topic, CompletedScanData, self.scanCompletedCallback)
        self.gpsSub = rospy.Subscriber(self._location_sub_topic, Odometry, self.robotUpdate) # plotRobotPosition
        self.statusSub = rospy.Subscriber(self._status_sub_topic, ManagerStatus, self.managerStatusUpdate)
        #self.next_goal_sub = rospy.Subscriber('/next_goal', NavSatFix, self.onNextGoalUpdate) # display the next goal on the map
        
        self.pxrfSub = rospy.Subscriber(self._pxrf_response_topic, String, self.pxrfResponseCallback)
        
    def loadROSParams(self):
        # Load topic names into params
        self._location_sub_topic = rospy.get_param('gq7_ekf_odom_map_topic')
        self._gps_sub_topic = rospy.get_param('gq7_ekf_llh_topic')
        self._pxrf_response_topic = rospy.get_param("pxrf_response_topic")
        self._gps_moving_avg_topic = rospy.get_param("gps_moving_avg_topic")
        self._goal_pub_topic = rospy.get_param('goal_pub_topic')
        self._status_sub_topic = rospy.get_param('status_topic')
        self._manager_run_loop_service_name = rospy.get_param("manager_run_loop_service_name")
        self._scan_completed_topic = rospy.get_param("scan_completed_topic")
        
        # Load service names into params
        self._parking_brake_service = rospy.get_param('parking_break_service_name')
        self._next_point_service = rospy.get_param('next_goal_to_GUI_service_name')
        self._grid_points_service = rospy.get_param('grid_points_service_name')
        self._set_search_boundary_name = rospy.get_param('set_search_boundary_name')
        self._lower_arm_service_name = rospy.get_param('lower_arm_service_name')
        self._start_scan_service_name = rospy.get_param('start_scan_service_name')
        self._clear_service_name = rospy.get_param('clear_service_name')
        self._waypoints_service_name = rospy.get_param("waypoints_service_name")
        
        # Load action client topic names
        self._pxrf_client_topic = rospy.get_param('pxrf_client_topic_name')
        self._estop_enable_topic = rospy.get_param("estop_enable_topic")
        self._estop_reset_topic = rospy.get_param("estop_reset_topic")
        
        self._pxrf_test_results_file = rospy.get_param('pxrf_test_results_file')

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

        
        self.statusGPS = QtWidgets.QLineEdit()
        self.statusGPS.setText('GPS Connecting...')
        self.statusGPS.setReadOnly(True)
        
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
        self.managerComboBox.addItems(['Step', 'Continuous'])
        self.managerComboBox.setStyleSheet("color: lightgreen")
        self.managerComboBox.currentTextChanged.connect(self.managerComboBoxChanged)
        
        self.managerStepOnceBtn = QtWidgets.QPushButton('Manager Step')
        self.managerStepOnceBtn.setStyleSheet("color: lightgreen")
        self.managerStepOnceBtn.clicked.connect(self.managerStepOnce)
        
        self.statusDetailed = QtWidgets.QLineEdit("")
        self.statusDetailed.setReadOnly(True)

        # add buttons to the layout
        self.widget.addWidget(self.statusGPS,          row=1, col=0, colspan=2)
        self.widget.addWidget(self.statusManager,      row=1, col=2, colspan=2)
        self.widget.addWidget(self.statusDetailed,     row=1, col=4, colspan=2)
        
        self.widget.addWidget(clearHistoryBtn,         row=2, col=4, colspan=2)
        self.widget.addWidget(clearPathBtn,            row=2, col=2, colspan=2)
        self.widget.addWidget(self.editPathBtn,        row=2, col=0, colspan=2)
        # self.widget.addWidget(loadPathFileBtn,       row=2, col=2, colspan=1)
        # self.widget.addWidget(savePathBtn,           row=2, col=3, colspan=1)
        self.widget.addWidget(self.startPauseBtn,      row=2, col=6, colspan=2)
        
        self.widget.addWidget(self.adaptiveBtn,        row=3, col=0, colspan=2)
        self.widget.addWidget(self.resetBtn,           row=3, col=2, colspan=2)
        self.widget.addWidget(self.addBoundaryBtn,     row=3, col=4, colspan=2)
        self.widget.addWidget(self.managerStepOnceBtn, row=3, col=6, colspan=1)
        self.widget.addWidget(self.managerComboBox,    row=3, col=7, colspan=1)

        # self.widget.addWidget(self.parkBtn,          row=4, col=6, colspan=2)
        self.widget.addWidget(self.sampleBtn,          row=4, col=0, colspan=2)
        self.widget.addWidget(self.showPxrfBtn,        row=4, col=2, colspan=2)
        self.widget.addWidget(self.eStopBtn,           row=4, col=4, colspan=2)
        self.widget.addWidget(self.gridBtn,            row=4, col=6, colspan=2)

    def showPXRFResults(self):
        generate_plot(self.scanData.file_name)

    def scanCompletedCallback(self, data):
        if data.status == True:
            self.pxrfComplete = True
            self.scanData = deepcopy(data)
            self.statusDetailed.setText(self.scanData.element + "Scan Value: " + str(self.scanData.mean))
        else:
            self.pxrfComplete = False
        return True
    
    def managerStatusUpdate(self, data:ManagerStatus):
        self.statusManager.setText(data.status)
        
    def managerComboBoxChanged(self, selected_item):
        if selected_item == "Step":
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
            managerStepOnce_client = rospy.ServiceProxy(self._manager_run_loop_service_name, Trigger)
            managerStepOnce_client()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
     
    def pxrfResponseCallback(self, result:String):
        if self.pxrfRunning and result.data == "201":
            self.pxrfRunning = False
            self.sampleBtn.setText('Sample')
            self.statusDetailed.setText("Ready to collect")

            self.numMeasurements += 1
            
            self.addMarkerAt(self.prev_lat, self.prev_lon, f'Sample#{self.numMeasurements}')
            
            try:
                lower_arm_service = rospy.ServiceProxy(self._lower_arm_service_name, SetBool)
                lower_arm_service(False)
            
                rospy.sleep(1.0)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

    # This function adds points to roi (when user is editing path)
    def addROIPoint(self, point):
        if self.editPathMode or self.editBoundaryMode:
            points = [[handle['pos'].x(),handle['pos'].y()] for handle in self.pathRoi.handles]
            print(f'Clicked at point: {point}')
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
        rospy.loginfo(f" Sending Boundary Points: {boundary}")
        try:
            sendBoundaryClient = rospy.ServiceProxy(self._set_search_boundary_name, DeployAutonomy)
            if len(boundary) > 0:
                boundary.pop()
            lat = [float(lat[0]) for lat in boundary]
            lon = [float(lon[1]) for lon in boundary]
            res = sendBoundaryClient(lat,lon)
            print("Boundary Sent")
        except rospy.ServiceException as e:
            print(e)
            print("Boundary sent unsuccessfully")

    def sendWaypoints(self, waypoints):
        #rospy.wait_for_service('/waypoints')
        try:
            sendWaypointsClient = rospy.ServiceProxy(self._waypoints_service_name, Waypoints)
            lat = [float(lat[0]) for lat in waypoints]
            lon = [float(lon[1]) for lon in waypoints]
            res = sendWaypointsClient(lat,lon)
            print("Waypoints sent! ")
        except rospy.ServiceException:
            print("Waypoints sent unsuccessfully")
        
    # this function turns on/off editing mode for the boundary
    def toggleEditBoundaryMode(self):
        if self.editPathMode:
            print("Warning: Please finish editing the path first")
            return
        elif self.editBoundaryMode and self.pathRoi.handles == []:
            print('Warning: No boundary to edit, please draw a boundary first')
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
            self.sendBoundary(self.boundaryPath)

    # This function turns on/off editing mode
    def toggleEditPathMode(self):
        if self.editBoundaryMode:
            print("Warning: Please finish editing the boundary first")
            return
        #check if pathRoi is empty
        elif self.editPathMode and self.pathRoi.handles == []:
            print('Warning: No path to edit, please draw a path first')
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
            print(self.pathGPS)
            self.pathRoi.setPoints([])
            self.sendWaypoints(self.pathGPS)
    
    def toggleAdaptive(self):
        if not self.editPathMode and not self.editBoundaryMode:
            self.adaptive = not self.adaptive
            rospy.set_param('/adaptive', True)
            rospy.set_param('/grid', False)
            rospy.set_param('/waypoint', False)
            self.adaptiveBtn.setText('Stop Adaptive')
        else:
            self.adaptiveBtn.setText('Start Adaptive')
            rospy.set_param('/adaptive', False)
    
    def toggleGrid(self):
        if not self.editPathMode and not self.editBoundaryMode:
            print("start grid")
            self.grid = not self.grid
            rospy.set_param('/grid', True)
            rospy.set_param('/adaptive', False)
            rospy.set_param('/waypoint', False)
            x, y = zip(*self.waypointsPath)
            self.pathPlot.setData(x=list(x), y=list(y))
            self.pathRoi.setPoints([])
            self.pathGPS = self.waypointsGPS
            self.waypointsPath = []
            self.gridBtn.setText('Stop Grid')
        else:
            self.gridBtn.setText('Start Grid')
            rospy.set_param('/grid', False)

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
            rospy.set_param('/waypoint', True)
            rospy.set_param('/adaptive', False)
            rospy.set_param('/grid', False)
        elif self.pathIndex == 0:
            self.startPauseBtn.setText('Start')
            rospy.set_param('/waypoint', False)
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
        if self.latitude == None or self.longitude == None:
            return
        
        lat = self.latitude
        lon = self.longitude

        #calculate heading based on gps coordinates 
        pixX, pixY = self.satMap.coord2Pixel(lat, lon)
        
        # rospy.loginfo_throttle(10, "Robot Lat/Lon    : %s %s", self.latitude, self.longitude)
        # rospy.loginfo_throttle(10, "Tile Map Lat/Lon : %s %s", self.satMap.lat, self.satMap.lon)
        # rospy.loginfo_throttle(10, "Tile Map X/Y     : %s %s", self.satMap.x, self.satMap.y)
        rospy.loginfo_once("Robot Pixel Pos  : %s %s", pixX, pixY)
        
        self.quaternion = tf.transformations.random_quaternion()
        self.quaternion[0] = data.pose.pose.orientation.x
        self.quaternion[1] = data.pose.pose.orientation.y
        self.quaternion[2] = data.pose.pose.orientation.z
        self.quaternion[3] = data.pose.pose.orientation.w
        
        eulerVals = euler_from_quaternion(self.quaternion, "sxyz")
        robotHeading = eulerVals[2]

        if not self.robotArrow is None:
            self.robotArrow.setStyle(angle = 180 - robotHeading*180.0/np.pi)
            self.robotArrow.setPos(pixX, pixY)
            self.robotArrow.update()

        self.historyPoints.append([pixX, pixY])
        self.setHistory()
        self.prev_lat = lat
        self.prev_lon = lon
        self.prev_heading = robotHeading
    
    # This function updates the value of longitude and latitude information
    def onGpsUpdate(self, data: NavSatFix):
        self.latitude = data.latitude
        self.longitude = data.longitude
        
        self.statusGPS.setText("GPS: " + str(round(self.latitude,4)) + " | " + str(round(self.longitude, 4)))

    #CHECK
    # This function updates the goal and displays it on the map
    def onNextGoalUpdate(self, req: NavigateGPS):
        if self.adaptive:
            print("adaptive mode")
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
            print("Next point")
            point = self.satMap.coord2Pixel(req.goal_lat, req.goal_lon)
            self.updateGoalMarker(point)

        return True
    
    #CHECK 
    def onGridPoints(self, req: Waypoints):
        #display it on the map by setting the data of the pathPlot
        if True:
            print("Printing grid points")
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
            print("Reset Failed")

    def togglePxrfCollection(self):
        try:
            lower_arm_service = rospy.ServiceProxy(self._lower_arm_service_name, SetBool)
            lower_arm_service(True)
            
            rospy.sleep(1.5)
            
            start_scan_service = rospy.ServiceProxy(self._start_scan_service_name, Complete)
            start_scan_service(True)
            
            self.pxrfRunning = True
            self.statusDetailed.setText("Collecting Sample")
            self.sampleBtn.setText("Stop PXRF")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

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