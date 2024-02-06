#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from microstrain_inertial_msgs.msg import MipGnssFixInfo as GNSSFixInfo, MipFilterStatus as FilterStatus
from sensor_msgs.msg import NavSatFix
import pprint
"""
things to check:
    - rtk fix
    - lidar timing
"""

class system_check(object):
    def __init__(self):
        rospy.init_node('system_check',anonymous=True)
        self.status = {'lidar_time': False,
                        'rtk_fix_1': False,
                        'rtk_fix_2': False,
                        'filter state': False,
                        'rtk_covariance_1': False,
                        'rtk_covariance_2': False}
        rospy.Subscriber('/livox/lidar',PointCloud2,self.readPC)
        rospy.Subscriber('/gnss1/fix_info',GNSSFixInfo,self.gnss1_fix_info)
        rospy.Subscriber('/gnss2/fix_info',GNSSFixInfo,self.gnss2_fix_info)
        rospy.Subscriber('/gnss1/fix',NavSatFix,self.gnss1_fix)
        rospy.Subscriber('/gnss2/fix',NavSatFix,self.gnss2_fix)
        rospy.Subscriber('/nav/status',FilterStatus,self.filter_status)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pp = pprint.PrettyPrinter(indent=4)
            print()
            pp.pprint(self.status)
            # print(self.status)
            rate.sleep()
    def readPC(self,msg):
        diff = rospy.Time.now()-msg.header.stamp
        self.status['lidar_time'] = diff.nsecs < 2e8 and diff.secs == 0
    def gnss1_fix_info(self,msg):
        if msg.fix_type == 5:
            self.status['rtk_fix_1'] = 'rtk float'
        elif msg.fix_type == 6:
            self.status['rtk_fix_1'] = 'rtk fix' # Best
        else:
            self.status['rtk_fix_1'] = False
    def gnss2_fix_info(self,msg):
        if msg.fix_type == 5:
            self.status['rtk_fix_2'] = 'rtk float'
        elif msg.fix_type == 6:
            self.status['rtk_fix_2'] = 'rtk fix' # Best
        else:
            self.status['rtk_fix_2'] = False
    def gnss1_fix(self,msg):
        rounded_covariance_1 = tuple(round(value, 4) for value in msg.position_covariance)
        self.status['rtk_cov1'] = str(rounded_covariance_1)
    def gnss2_fix(self,msg):
        rounded_covariance_2 = tuple(round(value, 4) for value in msg.position_covariance)
        self.status['rtk_cov2'] = str(rounded_covariance_2)
    def filter_status(self,msg):
        self.status['filter state'] = msg.filter_state == 4

if __name__ == '__main__':
    system_check()
