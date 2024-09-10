#!/usr/bin/env python3
import sys
sys.path.append('../')
import math
import pyproj
import numpy as np
from pyproj import CRS
from pyproj.aoi import AreaOfInterest
from pyproj.database import query_utm_crs_info
from pyproj import Proj
from boundaryCheck import *

#[40.44208352622619, -79.94610353523909, 1]
#[40.44236251851889, -79.94599539848328, 1]
#[40.44233453704325, -79.94584400702514, 1]
#[40.442054721646244, -79.9459456555756, 1]

# GPS: lat and lon
# UTM: easting and nothing
# Map: UTM coordinates with reference to initial (origin) UTM coordinates
class Conversion:
    def __init__(self, cells_per_meter):
        self.utm = 'EPSG:32617' #default to pittsburgh
        self.height = 0
        self.width = 0
        self.origin_utm = (0,0)
        self.new_boundary = []
        self.inner_boundary_offset = []
        self.cells_per_meter = cells_per_meter
        self.cell_size = 1/cells_per_meter
        self.half_cell_size = self.cell_size/2
    
    def boundary_conversion(self, boundaryPoints):
        x_coord = []
        y_coord = []
        x_coord_utm = []
        y_coord_utm = []
        boundary_offset = []
        for each in boundaryPoints:
            x_coord.append(each[0])
            y_coord.append(each[1])

        for i in range(len(x_coord)):
            x_coord_utm.append(self.get_utm(x_coord[i], y_coord[i])[0])
            y_coord_utm.append(self.get_utm(x_coord[i], y_coord[i])[1])

        self.origin_utm = (min(x_coord_utm), min(y_coord_utm))

        for i in range(len(x_coord)):
            boundary_offset.append([x_coord_utm[i] - self.origin_utm[0], y_coord_utm[i] - self.origin_utm[1]])

        #get the index of max(y_coord)
        max_index = y_coord_utm.index(max(y_coord_utm))
        self.height = math.ceil(y_coord_utm[max_index] - self.origin_utm[1])

        max_index = x_coord_utm.index(max(x_coord_utm))
        self.width = math.ceil(x_coord_utm[max_index] - self.origin_utm[0])

        #this is the new boundary (rectangle) and each point is the boundary of the map
        self.new_boundary = [self.origin_utm, 
                             [self.origin_utm[0] + self.width, self.origin_utm[1]], 
                             [self.origin_utm[0] + self.width, self.origin_utm[1] + self.height], 
                             [self.origin_utm[0], self.origin_utm[1] + self.height]]

        return boundary_offset
    
    def build_obs_map(self):
        if self.new_boundary == []:
            raise("Please run boundary_conversion first")
        
        map = np.full((self.width,self.height), 0)
        for i_row in range(len(map)):
            for j_col in range(len(map[i_row])):
                # Call the function on the current coordinate
                result = boundaryCheck(map, [i_row,j_col])
                # Set the value of the current coordinate to the result of the function
                if not result:
                    map[i_row][j_col] = 1

    # Get UTM from GPS
    # x - Easting, y - Northing
    def get_utm(self, lat, lon):
        wgs84 = pyproj.CRS('EPSG:4326') 
        utm17n = pyproj.CRS(self.get_zone(lat, lon)) 
        self.utm = utm17n
        transformer = pyproj.Transformer.from_crs(wgs84, utm17n)
        # print(self.utm,' 3')
        return transformer.transform(lat, lon)

    # Get the utm zone
    def get_zone(self, lat, lon):
        utm_crs_list = query_utm_crs_info( 
        datum_name="WGS 84", 
        area_of_interest=AreaOfInterest( 
            west_lon_degree=lon, 
            south_lat_degree=lat, 
            east_lon_degree=lon, 
            north_lat_degree=lat, 
            ), 
        )
        utm_crs = CRS.from_epsg(utm_crs_list[0].code)
        self.utm_zone = utm_crs
        return utm_crs

    # Get GPS from UTM
    def get_gps(self, utm_E, utm_N):
        utm_zone = self.utm
        wgs84 = pyproj.CRS('EPSG:4326') # WGS 84
        transformer = pyproj.Transformer.from_crs(utm_zone, wgs84)
        return transformer.transform(utm_E, utm_N)
    
    # Get Map from GPS
    def gps2map(self, lat, lon):
        if self.origin_utm == (0,0):
            raise("Please run boundary_conversion first")

        x, y = self.get_utm(lat, lon)
        x = x - self.origin_utm[0]
        y = y - self.origin_utm[1]
        return x, y

    def map2grid(self, x, y):
        return max(0, math.floor(x*self.cells_per_meter)-1), max(0, math.floor(y*self.cells_per_meter)-1)
    
    def grid2map(self, r, c):
        return (r*self.cell_size) + self.half_cell_size, (c*self.cell_size) + self.half_cell_size
    
    def map2gps(self, x, y): # distance in meters
        if self.origin_utm == (0,0):
            raise("Please run boundary_conversion first")
        
        posx = self.origin_utm[0] + x
        posy = self.origin_utm[1] + y
        return self.get_gps(posx, posy)

    # this function is not used. only for testing purpose
    def UNUSED_gps_to_meters(lat1, lon1, lat2, lon2):
        point1 = (lat1, lon1)
        point2 = (lat2, lon2)
        #getting the distance
        R = 6371000  # Earth's radius in meters
        dLat = math.radians(lat2 - lat1)
        dLon = math.radians(lon2 - lon1)
        a = math.sin(dLat / 2) * math.sin(dLat / 2) + \
            math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * \
            math.sin(dLon / 2) * math.sin(dLon / 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = R * c

        lat1 = math.radians(lat1)
        long1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        long2 = math.radians(lon1)
        bearing = math.atan2( \
        math.sin(long2 - long1) * math.cos(lat2), \
        math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(long2 - long1) \
        )
        
        return d
