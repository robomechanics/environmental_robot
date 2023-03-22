#!/usr/bin/env python3
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import numpy as np

# input the polygon points in clockwise order
def boundaryCheck(polygon, point):
    polygon = np.array(polygon)
    polygon = [tuple(row) for row in polygon]
    polygon = Polygon(polygon)
    point = Point(tuple(point))
    return point.within(polygon)    
