#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from microstrain_inertial_msgs.msg import HumanReadableStatus
from std_srvs.srv import SetBool, SetBoolResponse

# Global variable to track frozen state
freeze_position = False
current_lat = 0.0
current_lon = 0.0

def set_freeze_callback(req):
    global freeze_position, current_lat, current_lon
    freeze_position = req.data
    response = SetBoolResponse()
    
    if freeze_position:
        response.success = True
        response.message = "GPS position frozen"
        rospy.loginfo("GPS position frozen")
    else:
        response.success = True
        response.message = "GPS position unfrozen, resuming waypoint traversal"
        rospy.loginfo("GPS position unfrozen")
    
    return response

def gps_publisher():
    global freeze_position, current_lat, current_lon
    # Initialize the ROS node
    rospy.init_node('fake_moving_gps_publisher', anonymous=True)

    waypoints = [
        [40.45903077826049, -79.9580079317093],
        [40.459036900698614, -79.95798647403718],
        [40.459043023136154, -79.95796769857408],
        [40.459048125167016, -79.95794221758844],
        [40.45905730882159, -79.9579167366028],
        [40.459061390445456, -79.9579019844532],
        [40.459069553692416, -79.95787784457208],
        [40.45907771693842, -79.95785236358644],
        [40.45908485977782, -79.95783358812334],
        [40.45909302302193, -79.95781481266023],
        [40.45910220667037, -79.95779603719713],
        [40.45911343112788, -79.95777055621149],
        [40.459117512748335, -79.95774909853935],
        [40.45912363517852, -79.95772898197175],
        [40.45912771679836, -79.95770886540414],
        [40.45913179841793, -79.95767399668694],
        [40.45913179841793, -79.95765924453737],
        [40.45912567598848, -79.95763376355171],
        [40.45910832910198, -79.95760560035706],
        [40.45909200261648, -79.95760023593904],
        [40.45907567612701, -79.95759755373003],
        [40.45906037003952, -79.9576123058796],
        [40.45904608435472, -79.9576283991337],
        [40.459036900698614, -79.95764985680582],
        [40.459027717041245, -79.95767265558244],
        [40.459016492569404, -79.95768740773202],
        [40.45900526809571, -79.95771288871767],
        [40.45900220687528, -79.95773434638978],
        [40.45899506402706, -79.95775848627092],
        [40.45898690077106, -79.95778128504755],
        [40.45897975792123, -79.95780006051064],
        [40.45897669669964, -79.95781749486925],
        [40.45897057425604, -79.95783627033235],
        [40.458965472219276, -79.95786041021348],
        [40.45895832936718, -79.95788455009462],
        [40.45895730895967, -79.95791271328926],
        [40.45895628855216, -79.95793417096138],
        [40.45895526814462, -79.95796769857408],
        [40.45895628855216, -79.9579891562462],
        [40.45895730895967, -79.95801597833635],
        [40.458964451811894, -79.95803609490396],
        [40.45898077832838, -79.95804548263551],
        [40.45899404362013, -79.95804145932199],
        [40.45900934972273, -79.95803609490396],
        [40.45902159460228, -79.9580253660679]
    ]
    
    gps_pub = rospy.Publisher('/gps_moving_avg', NavSatFix, queue_size=1)
    
    # Create the service server
    freeze_service = rospy.Service('freeze_gps', SetBool, set_freeze_callback)

    gps_msg = NavSatFix()
    gps_msg.header.frame_id = "gq7_link"
    gps_msg.status = NavSatStatus(status=0, service=0)
    gps_msg.altitude = 278

    print("Publishing fake moving GPS data...")
    print("Use 'freeze_gps' service to freeze/unfreeze position")

    # Publish the message
    rate = rospy.Rate(0.5)
    counter = 0
    while not rospy.is_shutdown():
        gps_msg.header.stamp = rospy.Time.now()
        
        if not freeze_position:
            # Update position only if not frozen
            current_lat = waypoints[counter % len(waypoints)][0]
            current_lon = waypoints[counter % len(waypoints)][1]
            counter += 1
        
        gps_msg.latitude = current_lat
        gps_msg.longitude = current_lon
        gps_msg.position_covariance = [
            0.9874569211100415, 0.0, 0.0,
            0.0, 1.3351858854839662, 0.0,
            0.0, 0.0, 3.422781499451858
        ]
        gps_msg.position_covariance_type = 2
        
        gps_pub.publish(gps_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass
