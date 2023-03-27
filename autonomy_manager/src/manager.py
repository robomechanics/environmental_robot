#!/usr/bin/env python3
import rospy
from autonomy_manager.msg import ManagerStatus
from autonomy_manager.srv import DeployAutonomy, NavigateGPS, RunSensorPrep, Complete, ScanData
from sensor_msgs.msg import NavSatFix
from dummy_services import dummy_search
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from adaptiveROS import adaptiveROS
from boundaryConversion import conversion

class manager(object):
    def __init__(self):
        rospy.init_node('manager', anonymous=True)
        rospy.Subscriber('/sensor_prep_status',Bool,self.checkSensorPrepStatus)
        self.statusPub = rospy.Publisher('/autonomy_manager/status', ManagerStatus, queue_size=10,latch=True)
        #########################change it back to /gps_avg#########################################################
        rospy.Subscriber('/gnss1/fix', NavSatFix, self.gps_callback)
        self.update_status('waiting for navigation to be ready')
        self.sensorPrep = rospy.ServiceProxy('/run_sensor_prep',RunSensorPrep)
        self.update_status('standby')
        #intialize adaptive sampling class and conversion class
        self.adaptiveROS = None
        self.conversion = conversion()
        self.searchBoundary = []
        #######################change it to None#################################
        self.lat = 40.442083526
        self.lon = -79.94610353
        self.goalComplete = False
        self.pxrfComplete = False
        self.value = None
        deployService = rospy.Service('/autonomy_manager/deploy_autonomy', DeployAutonomy,self.deployService)
        goal_reach = rospy.Service('goal_reach', Complete, self.goal_reach)
        clear = rospy.Service('clear', Complete, self.clear)
        pxrf = rospy.Service('pxrf_complete', ScanData, self.pxrf)
        # wait until the gps calibration is finishedlon

        ################################uncomment#########################################
        # while self.lon is None or self.lat is None: 
        #     print("waiting for the calibration to finish")
        #     rospy.sleep(1)
        ##################################################################################

        # publish status
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            print(self.status)
            if self.status == 'received search area':
                print("receive")
                self.runSearchAlgo()
                print(self.nextScanLoc)
            elif self.status == 'received next scan loc':
                print("next")
                self.navigateToScanLoc()
            elif self.status == 'arrived at scan loc':
                print("arrive")
                self.runSensorPrep()
            elif self.status == 'finished raking':
                print("scan")
                self.scan()
            elif self.status == 'finished scan':
                print("algo")
                self.runSearchAlgo()
            rate.sleep()

    def scan(self):
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
        else:
            self.pxrfComplete = False

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
        print(startx, starty)
        self.adaptiveROS = adaptiveROS(self.conversion.width, self.conversion.height, [startx, starty])
        self.adaptiveROS.updateBoundary(boundary_utm_offset)
        print(self.adaptiveROS)
        self.update_status('received search area')
        return True

    def runSearchAlgo(self):
        self.update_status('running search algo')
        if self.pxrfComplete == True and self.value != None:
            pos = self.conversion.gps2map(self.lat, self.lon)
            self.adaptiveROS.update(pos[0], pos[1], self.value)
        #reset
        self.pxrfComplete = False
        self.value = None
        #predict the next location
        print(self.adaptiveROS)
        self.nextScanLoc = self.adaptiveROS.predict()
        gps = self.conversion.map2gps(self.nextScanLoc[0],self.nextScanLoc[1])
        print("sending")
        self.send_location(gps[0],gps[1])
        self.update_status('received next scan loc')

    def navigateToScanLoc(self):
        self.update_status('navigating to scan loc')
        #self.navigation(self.nextScanLoc[0],self.nextScanLoc[1])
        self.send_nav(self.nextScanLoc[0], self.nextScanLoc[1])

    def runSensorPrep(self):
        self.update_status('raking')
        self.sensorPrep(True)

    def checkSensorPrepStatus(self,data):
        if data.data == True and self.status == 'raking':
            self.update_status('finished raking')

    def update_status(self,newStatus):
        self.status = newStatus
        msg = ManagerStatus()
        msg.status = self.status
        msg.header.stamp = rospy.Time.now()
        self.statusPub.publish(msg)
    
    def gps_callback(self,data):
        self.lat = data.latitude
        self.lon = data.longitude
    
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
