#!/usr/bin/python3
import rospy
import sys
import os
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import numpy as np
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import pandas as pd
import csv
from std_srvs.srv import SetBool
from sensor_msgs.msg import NavSatFix
from microstrain_inertial_msgs.msg import FilterHeading
from tile import TileMap
import actionlib
from pxrf.msg import TakeMeasurementAction, TakeMeasurementGoal, TakeMeasurementResult
import rospkg
rospack = rospkg.RosPack()
pxrf_path = rospack.get_path('pxrf')
sys.path.insert(0, os.path.abspath(os.path.join(pxrf_path, "scripts")))
from plot import generate_plot
from gps_user_location import read_location
from autonomy_manager.srv import NavigateGPS, DeployAutonomy, Complete, RunSensorPrep
#testing
lat_set = 0
lon_set = 0
zoom_set = 0
width_set = 0
height_set = 0

class MeasurementMarker(pg.GraphicsObject):
    def __init__(self, x, y, parent=None):
        super().__init__(parent)
        size = 50
        self._rect = QtCore.QRectF(x - size // 2, y - size // 2, size, size)
        self.picture = QtGui.QPicture()
        self._generate_picture()

    @property
    def rect(self):
        return self._rect

    def _generate_picture(self):
        painter = QtGui.QPainter(self.picture)
        painter.setPen(pg.mkPen("w"))
        painter.setBrush(pg.mkBrush("g"))
        painter.drawRect(self.rect)
        painter.end()

    def paint(self, painter, option, widget=None):
        painter.drawPicture(0, 0, self.picture)

    def boundingRect(self):
        return QtCore.QRectF(self.picture.boundingRect())

class PlotWithClick(pg.PlotItem):
    def mouseClickEvent(self, ev):
        xClick = self.getViewBox().mapSceneToView(ev.scenePos()).x()
        yClick = self.getViewBox().mapSceneToView(ev.scenePos()).y()
        for handler in self.click_handlers:
            handler([xClick, yClick])

class PolyLineROI_noHover(pg.PolyLineROI):
    def hoverEvent(self, ev):
        pass

class GpsNavigationGui:
    def __init__(self, lat, lon, zoom, width, height):
        self.prev_lat = 0
        self.prev_lon = 0
        self.prev_heading = 0

        #pxrf control
        self.pxrfRunning = False

        #speed display
        self.highSpeed = True

        #get the map
        self.satMap = TileMap(coord=(lat, lon), dim=(width, height), zoom=zoom)

        # init widget
        pg.setConfigOption('imageAxisOrder', 'row-major')
        self.widget = pg.LayoutWidget()

        # add plot for map
        satGUI = pg.GraphicsLayoutWidget()
        self.widget.addWidget(satGUI, row=0, col=0, colspan=8)
        satGUI.setBackground('w')
        self.click_plot = PlotWithClick()
        satGUI.addItem(self.click_plot)

        # show satellite image
        img = pg.ImageItem(self.satMap.map_array)
        img.setBorder({'color': 'b', 'width': 3})
        self.click_plot.addItem(img)
        #self.click_plot.showAxes(True)
        self.click_plot.setAspectLocked()
        self.click_plot.invertY()

        # with open('locations.csv') as f:
        #     reader = csv.reader(f)
        #     self.num_measurements = 0
        #     for row in reader:
        #         self.num_measurements += 1
        #         label = row[0]
        #         lat = float(row[1])
        #         lon = float(row[2])
        #         self.add_marker_at(lat, lon, label=label)

        # add arrow to show robot
        self.robotArrow = pg.ArrowItem(headLen=40, tipAngle=30, brush='r')
        self.click_plot.addItem(self.robotArrow)

        # init ROI for editing path
        self.pathRoi = PolyLineROI_noHover([], closed=False)

        # init waypoints display
        self.click_plot.addItem(self.pathRoi)

        # init plot for path
        self.pathPlotPoints: list[list] = []
        self.pathGPS = []
        self.boundaryPath = []
        self.heading = 0
        self.pathPlot = self.click_plot.plot(symbolBrush=(255, 0, 255))
        self.boundaryPlot = self.click_plot.plot(symbolBrush=(255, 255, 255))
        self.currentGoalMarker = self.click_plot.plot(symbolBrush=(0, 0, 255))
        self.pathIndex = 0

        # interaction for adding path points
        self.click_plot.click_handlers = [self.addROIPoint]

        # robot's history (path measured by gps)
        self.historyPoints: list[list[int]] = []
        self.historyPlot = self.click_plot.plot(pen=pg.mkPen('g', width=3))
        self.setHistory()

        # ros services
        self.parking_brake = rospy.ServiceProxy('/parking_brake', SetBool)
        self.nextPoint = rospy.Service('next_goal', NavigateGPS, self.on_next_goal_update)
        #############################################
        # add buttons
        self.setup_widgets()

        # ros action clients
        self.pxrf_client = actionlib.SimpleActionClient('/take_measurement', TakeMeasurementAction)

        # ros sub pub
        #self.navigation_sub = rospy.Subscriber('/gps_navigation/current_goal', PoseStamped, self.readNavigation) # get status of navigation controller
        self.goal_pub = rospy.Publisher('/gps_navigation/goal', PoseStamped, queue_size=5)

        self.location_sub = rospy.Subscriber('/gnss1/fix', NavSatFix, self.on_gps_update)
        #self.heading_sub = rospy.Subscriber('/nav/heading', FilterHeading, self.on_heading_update)
        self.odom_sub = rospy.Subscriber('/nav/odom_throttle', Odometry, self.on_odom_update) # plotRobotPosition
        #self.next_goal_sub = rospy.Subscriber('/next_goal', NavSatFix, self.on_next_goal_update) # display the next goal on the map
        
        #rospy.spin()
    def add_marker_at(self, lat: float, lon: float, label=None, size=20):
        pos = self.satMap.coord2Pixel(lat, lon)

        marker = pg.TargetItem(
            pos=pos,
            movable=False,
            size=size,
            label=label)

        self.click_plot.addItem(marker)

    def setup_widgets(self):
        def prevGoal():
            self.changeGoal(changeDir=-1)

        def nextGoal():
            self.changeGoal(changeDir=1)

        def clearHistory():
            self.setHistory(clear = True)

        # add buttons 
        clearHistoryBtn = QtWidgets.QPushButton('Clear History')
        clearHistoryBtn.setStyleSheet("background-color : yellow")
        clearHistoryBtn.clicked.connect(clearHistory)
        clearPathBtn = QtWidgets.QPushButton('Clear Path')
        clearPathBtn.setStyleSheet("background-color : yellow")
        clearPathBtn.clicked.connect(self.clearPath)
        self.editPathBtn = QtWidgets.QPushButton('Edit Path')
        self.editPathBtn.setStyleSheet("background-color : yellow")
        self.editPathMode = False
        self.editPathBtn.clicked.connect(self.toggleEditPathMode)
        loadPathFileBtn = QtWidgets.QPushButton('Load Path')
        loadPathFileBtn.setStyleSheet("background-color : yellow")
        loadPathFileBtn.clicked.connect(self.loadPathFile)
        savePathBtn = QtWidgets.QPushButton('Save Path')
        savePathBtn.setStyleSheet("background-color : yellow")
        savePathBtn.clicked.connect(self.savePath)
        self.startPauseBtn = QtWidgets.QPushButton('Start')
        self.startPauseBtn.setStyleSheet("background-color : yellow")
        self.is_navigating = False
        self.startPauseBtn.clicked.connect(self.startPause)
        prevGoalBtn = QtWidgets.QPushButton('Prev Goal')
        prevGoalBtn.setStyleSheet("background-color : yellow")
        prevGoalBtn.clicked.connect(prevGoal)
        nextGoalBtn = QtWidgets.QPushButton('Next Goal')
        nextGoalBtn.setStyleSheet("background-color : yellow")
        nextGoalBtn.clicked.connect(nextGoal)
        self.stopStatus = True
        self.parking_brake(self.stopStatus)
        self.parkBtn = QtWidgets.QPushButton('PARK ON')
        self.parkBtn.setStyleSheet("background-color : red")
        self.parkBtn.clicked.connect(self.toggle_brake)
        self.pxrfStatus = False
        self.pxrfBtn = QtWidgets.QPushButton('Sample')
        self.pxrfBtn.clicked.connect(self.toggle_pxrf_collection)
        self.statusGPS = QtWidgets.QLineEdit()
        self.statusGPS.setText('GPS Connecting')
        self.statusGPS.setReadOnly(True)
        self.statusNav = QtWidgets.QLineEdit()
        self.statusNav.setText('Manual Mode')
        self.statusNav.setReadOnly(True)
        self.statusPxrf = QtWidgets.QLineEdit()
        self.statusPxrf.setText('Ready to collect')
        self.statusPxrf.setReadOnly(True)
        
        # set the boundary on the map 
        self.addBoundaryBtn = QtWidgets.QPushButton('Edit Bound')
        self.addBoundaryBtn.setStyleSheet("background-color : orange")
        self.editBoundaryMode = False
        self.addBoundaryBtn.clicked.connect(self.toggleEditBoundaryMode)
        clearBoundaryBtn = QtWidgets.QPushButton('Clear Boundary')
        clearBoundaryBtn.setStyleSheet("background-color : orange")
        clearBoundaryBtn.clicked.connect(self.clearBoundary)
        self.adaptive = False
        self.adaptiveBtn = QtWidgets.QPushButton('Start Adaptive')
        self.adaptiveBtn.setStyleSheet("background-color : orange")
        self.adaptiveBtn.clicked.connect(self.toggleAdaptive)
        self.grid = False
        self.gridBtn = QtWidgets.QPushButton('Start Grid')
        self.gridBtn.setStyleSheet("background-color : orange")
        self.gridBtn.clicked.connect(self.toggleGrid)
    
        # add buttons to the layout
        self.widget.addWidget(self.statusGPS,     row=1, col=0, colspan=3)
        self.widget.addWidget(self.statusNav,     row=1, col=3, colspan=1)
        self.widget.addWidget(self.statusPxrf,    row=1, col=4, colspan=2)
        
        self.widget.addWidget(clearHistoryBtn,    row=2, col=4, colspan=2)
        self.widget.addWidget(clearPathBtn,       row=2, col=1, colspan=1)
        self.widget.addWidget(self.editPathBtn,   row=2, col=0, colspan=1)
        self.widget.addWidget(loadPathFileBtn,    row=2, col=2, colspan=1)
        self.widget.addWidget(savePathBtn,        row=2, col=3, colspan=1)
        self.widget.addWidget(nextGoalBtn,        row=2, col=6, colspan=1)
        self.widget.addWidget(self.startPauseBtn, row=2, col=7, colspan=1)
       
        #self.widget.addWidget(prevGoalBtn,        row=2, col=7, colspan=1)
        
        self.widget.addWidget(self.addBoundaryBtn,row=3, col=0, colspan=1)
        self.widget.addWidget(clearBoundaryBtn,   row=3, col=1, colspan=1)
        self.widget.addWidget(self.gridBtn,       row=3, col=6, colspan=2)
        self.widget.addWidget(self.adaptiveBtn,   row=3, col=4, colspan=2)

        self.widget.addWidget(self.pxrfBtn,       row=4, col=0, colspan=2)
        self.widget.addWidget(self.parkBtn,       row=4, col=6, colspan=2)

    def on_pxrf_measurement_complete(self, status, result: TakeMeasurementResult):
        print(f'pxrf cb result: {result.result.data}')
        self.pxrfRunning = False
        self.pxrfBtn.setText('Sample')
        self.statusPxrf.setText("Ready to collect")
        self.pxrfStatus = False

        self.num_measurements += 1
        with open('measurement_locations.csv', 'a') as f:
            f.write(f'Sample{self.num_measurements},{self.prev_lat},{self.prev_lon}\n')

        self.add_marker_at(self.prev_lat, self.prev_lon, f'Sample#{self.num_measurements}')

        if result.result.data == "201":
            generate_plot()

    #This function sets the heading of the robot
    def on_heading_update(self, data: FilterHeading):
        if(data.heading_rad == 0 and data.heading_deg == 0):
            return
        print('heading update')
        self.heading = data.heading_rad 

        #self.prev_heading = self. heading

    # This function adds points to roi (when user is editing path)
    def addROIPoint(self, point):
        if self.editPathMode or self.editBoundaryMode:
            print('click: Add Point')
            points = [[handle['pos'].x(),handle['pos'].y()] for handle in self.pathRoi.handles]

            points.append(point)
            self.pathRoi.setPoints(points)
    
    # This function converts the current path from gps coordinates to pixels
    def gps_to_pixels(self):
        self.pathPlotPoints = []
        for gpsPoint in self.pathGPS:
            lat = gpsPoint[0]
            lon = gpsPoint[1]
            point = list(self.satMap.coord2Pixel(lat, lon))
            self.pathPlotPoints.append(point)

    # This fuction converts the current path from pixels to gps coordinates
    def pixels_to_gps(self, pathPlotPoints):
        pathGPS = []
        for point in pathPlotPoints:
            gpsPoint = list(self.satMap.pixel2Coord(point))
            gpsPoint.append(1)
            pathGPS.append(gpsPoint)
        return pathGPS
    
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
        self.startPauseBtn.setText('start')

    #This function clears the current boundary
    def clearBoundary(self):
        self.pathRoi.setPoints([])
        self.pathPlotPoints= []
        self.boundaryPath = []
        self.boundaryPlot.setData(x=[], y=[])
        self.pathPlot.setData(x=[], y=[])
        self.updateGoalMarker()
        self.clear_map()
    
    # this is a utility function to pass the boundary points
    def sendBoundary(self, boundary):
        rospy.wait_for_service('/autonomy_manager/deploy_autonomy')
        try:
            print("try")
            sendBoundary = rospy.ServiceProxy('/autonomy_manager/deploy_autonomy', DeployAutonomy)
            if len(boundary) > 0:
                boundary.pop()
            lat = [float(lat[0]) for lat in boundary]
            lon = [float(lon[1]) for lon in boundary]
            print(lat)
            res = sendBoundary(lat,lon)
            print("set")
        except rospy.ServiceException:
            print("boundary sent unsuccessfully")
            
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
                for point in self.pathPlotPoints:
                    self.addROIPoint(point)
            else:
                self.addBoundaryBtn.setText('Edit Bound')
                self.pathPlotPoints = []
                for handle in self.pathRoi.handles:
                    pos = handle['pos']
                    self.pathPlotPoints.append([pos.x(), pos.y()])

                #append the first point to the end to close the boundary
                self.pathPlotPoints.append(self.pathPlotPoints[0])

                self.boundaryPath = self.pixels_to_gps(self.pathPlotPoints)
                #self.boundaryPath = self.pathPlotPoints
                x, y = zip(*self.pathPlotPoints)
                self.boundaryPlot.setData(x=list(x), y=list(y))
                self.pathRoi.setPoints([])
                self.pathPlotPoints = []
                self.sendBoundary(self.boundaryPath)
                print(self.boundaryPath)

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
            self.editPathBtn.setText('Edit Path')
            self.pathPlotPoints = []
            for handle in self.pathRoi.handles:
                pos = handle['pos']
                self.pathPlotPoints.append([pos.x(), pos.y()])
            self.pathGPS = self.pixels_to_gps(self.pathPlotPoints)
            x, y = zip(*self.pathPlotPoints)
            self.pathPlot.setData(x=list(x), y=list(y))
            self.pathRoi.setPoints([])
            self.updateGoalMarker()
    
    def toggleAdaptive(self):
        if not self.editPathMode and not self.editBoundaryMode and not self.pathRoi.handles == []:
            self.adaptive = not self.adaptive
            rospy.wait_for_service('/autonomy_manager/deploy_autonomy')
            try:
                start = rospy.ServiceProxy('/start', RunSensorPrep)
                res = start(True)
            except rospy.ServiceException:
                print("start sent unsuccessfully")
            self.adaptiveBtn.setText('Stop Adaptive')
        else:
            self.adaptiveBtn.setText('Start Adaptive')
        
    def toggleGrid(self):
        self.grid = not self.grid
        #to do
        pass

    # This function loads path from csv file chosen by user
    def loadPathFile(self):
        fn = str(QtGui.QFileDialog.getOpenFileName()[0])
        #pathFn = easygui.fileopenbox()
        if fn =='':
            return
        if self.editPathMode:
            self.toggleEditPathMode()
        self.clearPath()
        csvData = pd.read_csv(fn)
        csvData = np.array(csvData.iloc[:,:])
        for i in range(csvData.shape[0]):
            toStop = True
            if csvData.shape[1]>2:
                toStop = bool(csvData[i,2])
            gpsPoint = [csvData[i,0],csvData[i,1],toStop]
            self.pathGPS.append(gpsPoint)
        self.gps_to_pixels()
        self.pathPlot.setData(x=[point[0] for point in self.pathPlotPoints],y=[point[1] for point in self.pathPlotPoints])
        self.changeGoal(reset=True)

    # this function saves the current path as csv file
    def savePath(self):
        fn = str(QtGui.QFileDialog.getSaveFileName()[0])
        csvFile = open(fn,'w',newline='')
        csvWriter = csv.writer(csvFile,delimiter=',')
        csvWriter.writerow(['lat','lon','stop'])
        for gpsPoint in self.pathGPS:
            csvWriter.writerow(gpsPoint)

    # This function starts/ pauses the navigation
    def startPause(self):
        if (len(self.pathGPS) == 0):
            return
        self.is_navigating = not self.is_navigating
        if self.is_navigating:
            self.startPauseBtn.setText('Pause')
        elif self.pathIndex == 0:
            self.startPauseBtn.setText('Start')
        else:
            self.startPauseBtn.setText('Continue')

    # This function changes the goal to another waypoint on path
    def changeGoal(self,reset=False,changeDir=1):
        self.pathIndex = self.pathIndex + changeDir
        if self.pathIndex < 0:
            self.pathIndex = max(self.pathIndex+len(self.pathPlotPoints), 0)
        if reset or self.pathIndex >= len(self.pathPlotPoints):
            self.pathIndex = 0
            if self.is_navigating:
                self.startPause()
        self.updateGoalMarker()

    # This function updates the marked goal on the path
    def updateGoalMarker(self):
        if self.pathIndex < len(self.pathPlotPoints):
            point = self.pathPlotPoints[self.pathIndex]
            self.currentGoalMarker.setData(x=[point[0]], y=[point[1]])
        else:
            self.currentGoalMarker.setData(x=[], y=[])

    # This function is called by subscriber of gps sensor
    def on_odom_update(self,data: Odometry):
        #print('Incoming Odom')
        lat = data.pose.pose.position.x
        lon = data.pose.pose.position.y

        #calculate heading based on gps coordinates 
        pixX, pixY = self.satMap.coord2Pixel(lat, lon)
        #print(f"GPS -> pixels ({lat}, {lon}) -> {pixX}, {pixY}")
        prevpixX, prevpixY = self.satMap.coord2Pixel(self.prev_lat, self.prev_lon)
        robotHeading = self.heading
        #print("robotHeading "+ str(robotHeading))
        quat = data.pose.pose.orientation
        r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        #xVec = r.as_dcm()[:,0]
        #robotHeading = np.mod(math.atan2(xVec[1],xVec[0])+np.pi/3.0,2*np.pi)
        #robotHeading = self.heading #math.atan2(xVec[0],xVec[1])

        if not self.robotArrow is None:
            self.robotArrow.setStyle(angle = robotHeading*180.0/np.pi + 90.0)
            self.robotArrow.setPos(pixX, pixY)
            self.robotArrow.update()

        self.historyPoints.append([pixX, pixY])
        self.setHistory()
        self.prev_lat = lat
        self.prev_lon = lon
        self.prev_heading = robotHeading
    
    # This function updates the value of longitude and latitude information
    def on_gps_update(self, data: NavSatFix):
        if(data.longitude != 0.0 and data.latitude != 0.0):
            self.longitude = data.longitude
            self.latitude = data.latitude
            self.statusGPS.setText("lon: " + str(round(self.longitude,4)) + " " +"lat: " + str(round(self.latitude, 4)) )
        else:
            self.statusGPS.setText("GPS connecting")

    # This function updates the goal and displays it on the map
    def on_next_goal_update(self, req: NavigateGPS):
        
        if self.adaptive or self.grid or True: #remove this later
            #x_loc = self.gps_to_pixels(req.goal_lon)
            #y_loc = self.gps_to_pixels(req.goal_lat)
            self.pathGPS.append([req.goal_lat, req.goal_lon])
            self.gps_to_pixels()

            #self.pathPlot.setData(x = x_loc, y = y_loc)
       
            x, y = zip(*self.pathPlotPoints)
            self.pathPlot.setData(x=list(x), y=list(y))
            self.pathRoi.setPoints([])
            self.pathPlotPoints = []
            #self.pathRoi.setPoints([])
            #self.addROIPoint(x_loc, y_loc)
            self.updateGoalMarker()
            return True

    #this function checks status of navigation controller
    def readNavigation(self,data: PoseStamped):
        print('Incoming nav update')
        if self.is_navigating:
            self.statusNav.setText("Automatic navigation")
            currNavGoal = np.array([data.pose.position.y, data.pose.position.x])
            desNavGoal = np.array(self.pathGPS[self.pathIndex][0:2])
            onCurrentGoal = np.linalg.norm(desNavGoal-currNavGoal) < 1e-4
            navFinished = data.pose.position.z < 0
            if onCurrentGoal and navFinished:
                if self.pathGPS[self.pathIndex][2]:
                    self.startPause()
                self.changeGoal()
            else:
                msg = PoseStamped()
                msg.pose.position.y = desNavGoal[0]
                msg.pose.position.x = desNavGoal[1]
                self.goal_pub.publish(msg)
        else:
            # stop the navigation
            self.statusNav.setText("Manual")
            msg = PoseStamped()
            msg.pose.position.x = float('nan')
            msg.pose.position.y = float('nan')
            msg.pose.position.z = -1
            self.goal_pub.publish(msg)

    def toggle_brake(self):
        self.stopStatus = not self.stopStatus
        self.is_navigating = 0

        self.parking_brake(self.stopStatus)
        if self.stopStatus:
            self.parkBtn.setText('PARK ON')
            self.parkBtn.setStyleSheet("background-color : red")
        else:
            self.parkBtn.setText('PARK OFF')
            self.parkBtn.setStyleSheet("background-color : green")

    def clear_map(self):
        rospy.wait_for_service('clear')
        try:
            clear = rospy.ServiceProxy('clear', Complete)
            res = clear(True)
        except rospy.ServiceException as e:
            print("failed")



    def toggle_pxrf_collection(self):
        self.pxrfStatus = not self.pxrfStatus
        if self.pxrfStatus:
            self.statusPxrf.setText("Collecting")
            self.pxrfRunning = True
            self.pxrfBtn.setText("STOP pxrf")
            #self.pubCTRL.publish("start")
            goal = TakeMeasurementGoal()
            self.pxrf_client.send_goal(goal, done_cb=self.on_pxrf_measurement_complete)
        else:
            self.statusPxrf.setText("Ready to collect")
            self.pxrfRunning = False
            self.pxrfBtn.setText("Sample")
            #self.pubCTRL.publish("stop")
            self.pxrf_client.cancel_all_goals()

if __name__ == '__main__':
    rospy.init_node('gps_user_input',anonymous=True)
    try:
       lat = float(rospy.get_param('~lat'))
       lon = float(rospy.get_param('~lon'))
       zoom = int(rospy.get_param('~zoom'))
       height = int(rospy.get_param('~height'))
       width = int(rospy.get_param('~width'))
    except:
        #user can input the location manually
        location_input = read_location()
        lat, lon = [float(n) for n in location_input[1:3]]
        zoom = int(location_input[3])
        width, height = [int(n) for n in location_input[4:6]]

    # main window
    app = QtWidgets.QApplication([])
    mw = QtWidgets.QMainWindow()
    print(f'{lat}, {lon}, {width}, {height}')
    gps_node = GpsNavigationGui(lat, lon, zoom, width, height)
    mw.setCentralWidget(gps_node.widget)
    mw.show()
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtWidgets.QApplication.instance().exec_()
