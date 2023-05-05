#!/usr/bin/env python3
import rospy
from autonomy_manager.msg import ManagerStatus
from autonomy_manager.srv import DeployAutonomy, NavigateGPS, RunSensorPrep, Complete, ScanData
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from dummy_services import dummy_search
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from adaptiveROS import adaptiveROS
from gridROS import gridROS
from boundaryConversion import conversion
from sensor_msgs.msg import Joy

class manager(object):
    def __init__(self):
        rospy.init_node('manager', anonymous=True)
        self.statusPub = rospy.Publisher('/autonomy_manager/status', ManagerStatus, queue_size=10,latch=True)
        self.update_status('standby')
        rospy.Subscriber('/sensor_prep_status',Bool,self.checkSensorPrepStatus)
        rospy.Subscriber("/joy", Joy, self.manualBehaviorSkip)
        self.isOverride = False
        #########################change it back to /gps_avg#########################################################
        rospy.Subscriber('/gps_avg', Odometry, self.gps_callback)
        self.sensorPrep = rospy.ServiceProxy('/run_sensor_prep',RunSensorPrep)
        #intialize adaptive sampling class and conversion class
        self.adaptiveROS = None
        self.gridROS = None
        self.conversion = conversion()
        self.searchBoundary = []
        #######################change it to None#################################
        self.lat = 39.1893
        self.lon = -84.7638
        self.goalComplete = False
        self.pxrfComplete = False
        self.value = -1
        self.gps = None
        deployService = rospy.Service('/autonomy_manager/deploy_autonomy', DeployAutonomy,self.deployService)
        goal_reach = rospy.Service('goal_reach', Complete, self.goal_reach)
        clear = rospy.Service('clear', Complete, self.clear)
        pxrf = rospy.Service('pxrf_complete', ScanData, self.pxrf)

        # wait until the gps calibration is finished
        while self.lon is None or self.lat is None: 
             self.update_status('waiting for calibration to finish')
             print(self.status)
             rospy.sleep(1)
        
        self.update_status('standby')

        # publish status
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            print(self.status)
            if self.status == 'received search area':
                print("receive")
                self.runGridAlgo()
                #self.runSearchAlgo()
                print(self.nextScanLoc)
            elif self.status == 'received next scan loc':
                print("next")
                self.navigateToScanLoc()
            elif self.status == 'arrived at scan loc':
                print("arrive")
                self.runSensorPrep()
            elif self.status == 'finished raking':
                print("scan")
                runScan = True
                for i in range(100):
                    rospy.sleep(0.1)
                    if self.status != 'finished raking':
                        runScan = False
                        break
                if runScan:
                    self.scan()
            elif self.status == 'finished scan':
                print("algo")
                #self.runSearchAlgo()
                self.runGridAlgo()
            rate.sleep()

    def manualBehaviorSkip(self,data):
        self.isOverride = data.buttons[5] == 1
        if data.buttons[1]>0 and (not hasattr(self,'lastSkipButtonStatus') or not self.lastSkipButtonStatus):
            if self.status == 'raking':
                self.sensorPrep(False)
                self.update_status('finished raking')
            elif self.status == 'navigating to scan loc':
                cancel_goal = rospy.ServiceProxy('cancel_goal',NavigateGPS)
                cancel_goal(0,0)
                self.update_status('arrived at scan loc')
            elif self.status == 'finished raking':
                self.update_status('finished scan')
        self.lastSkipButtonStatus = data.buttons[1]>0

    def scan(self):
        self.update_status('scanning')
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
        self.update_status('finished scan')

    def pxrf(self, data):
        if data.status == True:
            self.pxrfComplete = True
            self.value = data.mean
            print("the value is :" + str(self.value))
        else:
            self.pxrfComplete = False
        return True

    def deployService(self,data):
        # data.boundart_lServiceat and data.boundary_lon lists, put then in the format of [[lat1,lon1],[lat2,lon2],...]
        for i in range(len(data.boundary_lat)):
            self.searchBoundary.append([data.boundary_lat[i],data.boundary_lon[i]])
        
        #initialize the zone, define boundary in utm coordinates 
        self.conversion.get_zone(self.lat, self.lon)
        boundary_utm_offset = self.conversion.boundaryConversion(self.searchBoundary)
        #when you receive the search area, define the robot position as the starting point, make sure to drive the 
        # the robot into the boundary first 
        startx, starty = self.conversion.gps2map(self.lat,self.lon)
        self.adaptiveROS = adaptiveROS(self.conversion.width, self.conversion.height, [startx, starty])
        self.adaptiveROS.updateBoundary(boundary_utm_offset)
        self.gridROS = gridROS(self.conversion.width, self.conversion.height, [0, 0], 25)
        self.gridROS.updateBoundary(boundary_utm_offset)
        print("received")
        self.update_status('received search area')
        return True
    
    def runGridAlgo(self):
        #print("here")
        self.update_status('running grid algo')
        self.nextScanLoc = self.gridROS.next()
        self.gps = self.conversion.map2gps(self.nextScanLoc[0],self.nextScanLoc[1])
        print("sending")
        print(self.gps)
        self.send_location(self.gps[0],self.gps[1])
        self.update_status('received next scan loc')


    def runSearchAlgo(self):
        self.update_status('running search algo')
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
        self.update_status('received next scan loc')

    def runGridAlgo(self):
        self.update_status('running grid algo')
        self.nextScanLoc = self.gridROS.next()
        gps = self.conversion.map2gps(self.nextScanLoc[0],self.nextScanLoc[1])
        self.send_location(gps[0],gps[1])
        self.update_status('received next scan loc - grid ROS')

    def navigateToScanLoc(self):
        self.update_status('navigating to scan loc')
        #self.navigation(self.nextScanLoc[0],self.nextScanLoc[1])
        self.send_nav(self.gps[0], self.gps[1])

    def runSensorPrep(self):
        self.sensorPrep(True)
        self.update_status('raking')
        
    def checkSensorPrepStatus(self,data):
        if data.data == False and self.status == 'raking' and not self.isOverride:
            self.update_status('finished raking')

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
            next_goal = rospy.ServiceProxy('next_goal', NavigateGPS)
            res = next_goal(x,y)
        except rospy.ServiceException as e:
            print("location failed")
        
    def send_nav(self,x,y):
        #note x,y are in gps coordinate
        #rospy.wait_for_service('next_goal_nav')
        try:
            next_goal = rospy.ServiceProxy('next_goal_nav', NavigateGPS)
            res = next_goal(x,y)
        except rospy.ServiceException as e:
            print("Navigation failed")

    def goal_reach(self, data):
        if data.status == True:
            self.goalComplete = True
            self.update_status('arrived at scan loc') 
        else:
            self.goalComplete = False
        
        return True

    def clear(self, data):
        if data.status == True:
            print("clear")
            self.adaptiveROS = None
            self.conversion = conversion()
            self.searchBoundary = []

        return True
            
if __name__ == '__main__':
    manager()
