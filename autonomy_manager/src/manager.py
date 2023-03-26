#!/usr/bin/env python3
import rospy
from autonomy_manager.msg import ManagerStatus
from autonomy_manager.srv import DeployAutonomy, NavigateGPS, RunSensorPrep
from sensor_msgs.msg import NavSatFix
from dummy_services import dummy_search
from std_msgs.msg import Bool
from adaptiveROS import adaptiveROS
from boundaryConversion import conversion

class manager(object):
    def __init__(self):
        rospy.init_node('manager', anonymous=True)
        rospy.Subscriber('/sensor_prep_status',Bool,self.checkSensorPrepStatus)
        self.statusPub = rospy.Publisher('/autonomy_manager/status', ManagerStatus, queue_size=10,latch=True)
        rospy.Subscriber('/gps_avg', NavSatFix, self.gps_callback)
        self.update_status('waiting for navigation to be ready')
        rospy.wait_for_service('/NavigateToGPS')
        self.navigation = rospy.ServiceProxy('/NavigateToGPS',NavigateGPS)
        self.sensorPrep = rospy.ServiceProxy('/run_sensor_prep',RunSensorPrep)
        self.update_status('standby')
        #intialize adaptive sampling class and conversion class
        self.adaptiveROS = None
        self.conversion = conversion()
        self.searchBoundary = []
        self.lat = None
        self.lon = None
        deployService = rospy.Service('/autonomy_manager/deploy_autonomy', DeployAutonomy,self.deployService)

        # wait until the gps calibration is finished
        while self.lon is None or self.lat is None: 
            print("waiting for the calibration to finish")
            rospy.sleep(1)



        # publish status
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.status == 'received search area':
                rospy.sleep(1)
                self.runSearchAlgo()
            elif self.status == 'received next scan loc':
                rospy.sleep(1)
                self.navigateToScanLoc()
                self.update_status('arrived at scan loc') # dummy status update to keep things flowing for now
            elif self.status == 'arrived at scan loc':
                rospy.sleep(1)
                self.runSensorPrep()
            elif self.status == 'finished raking':
                rospy.sleep(1)
                self.runSearchAlgo()
            rate.sleep()

    def deployService(self,data):
        # data.boundart_lat and data.boundary_lon lists, put then in the format of [[lat1,lon1],[lat2,lon2],...]
        for i in range(len(data.boundary_lat)):
            self.searchBoundary.append([data.boundary_lat[i],data.boundary_lon[i]])
        self.update_status('received search area')
        #initialize the zone, define boundary in utm coordinates 
        self.conversion.get_zone(self.lat,self.lon)
        boundary_utm_offset = self.conversion.get_boundary_utm_offset(self.searchBoundary)
        #when you receive the search area, define the robot position as the starting point, make sure to drive the 
        # the robot into the boundary first 
        startx, starty = self.conversion.gps2map(self.lat,self.lon)
        self.adaptiveROS = adaptiveROS(self.conversion.width, self.conversion.height, [startx, starty])
        self.adaptiveROS.updateBoundary(boundary_utm_offset)
        return True

    def runSearchAlgo(self):
        self.update_status('running search algo')
        self.nextScanLoc = self.adaptiveROS.predict()
        gps = self.conversion.map2gps(self.nextScanLoc[0],self.nextScanLoc[1])
        self.send_location(gps[0],gps[1])
        self.update_status('received next scan loc')

    def navigateToScanLoc(self):
        self.update_status('navigating to scan loc')
        self.navigation(self.nextScanLoc[0],self.nextScanLoc[1])

    def runSensorPrep(self):
        self.update_status('raking')
        self.sensorPrep(True)

    def checkSensorPrepStatus(self,data):
        if not data.data and self.status == 'raking':
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
    
    def send_location(x,y):
        rospy.wait_for_service('next_goal')
        try:
            next_goal = rospy.ServiceProxy('next_goal', NavigateGPS)
            res = next_goal(x,y)
        except rospy.ServiceException as e:
            print("failed")


if __name__ == '__main__':
    manager()
