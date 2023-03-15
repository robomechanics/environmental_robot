#!/usr/bin/env python3
import rospy
from autonomy_manager.msg import ManagerStatus
from autonomy_manager.srv import DeployAutonomy, NavigateGPS, RunSensorPrep
from dummy_services import dummy_search
from std_msgs.msg import Bool

class manager(object):
    def __init__(self):
        rospy.init_node('manager', anonymous=True)
        rospy.Subscriber('/sensor_prep_status',Bool,self.checkSensorPrepStatus)
        self.statusPub = rospy.Publisher('/autonomy_manager/status', ManagerStatus, queue_size=10,latch=True)
        self.update_status('waiting for navigation to be ready')
        rospy.wait_for_service('/NavigateToGPS')
        self.navigation = rospy.ServiceProxy('/NavigateToGPS',NavigateGPS)
        self.sensorPrep = rospy.ServiceProxy('/run_sensor_prep',RunSensorPrep)
        self.update_status('standby')
        deployService = rospy.Service('/autonomy_manager/deploy_autonomy', DeployAutonomy,self.deployService)

        # publish status
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.status == 'received search area':
                self.runSearchAlgo()
            elif self.status == 'received next scan loc':
                self.navigateToScanLoc()
                self.update_status('arrived at scan loc') # dummy status update to keep things flowing for now
            elif self.status == 'arrived at scan loc':
                self.runSensorPrep()
            elif self.status == 'finished raking':
                self.runSearchAlgo()
            rate.sleep()

    def deployService(self,data):
        self.searchBoundary = [data.boundary_lat,data.boundary_lon]
        self.update_status('received search area')
        return True

    def runSearchAlgo(self):
        self.update_status('running search algo')
        self.searchAlgo = dummy_search(self.searchBoundary)
        self.nextScanLoc = self.searchAlgo.next()
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

if __name__ == '__main__':
    manager()
