#!/usr/bin/env python3
import rospy
import numpy as np

from pyproj import CRS
from pyproj.aoi import AreaOfInterest
from pyproj.database import query_utm_crs_info

# Because of transformations
#import tf_conversions

import tf2_ros
import geometry_msgs.msg

should_loop = True

north_lat = 39.1031
west_lon = 84.5120

south_lat = -north_lat
east_lon = -west_lon


    
if __name__ == '__main__':
	should_Loop = True
	
	utm_crs_list = query_utm_crs_info(
		datum_name="WGS 84",
		area_of_interest=AreaOfInterest(
			west_lon_degree=west_lon,
			south_lat_degree=south_lat,
			east_lon_degree=east_lon,
			north_lat_degree=north_lat,
		),
	)
	utm_crs = CRS.from_epsg(utm_crs_list[0].code)
	
	while should_loop == True:
		print(utm_crs)
	
	
