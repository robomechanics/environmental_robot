#!/usr/bin/env python3
import sys
sys.path.append('../')
from boundaryConversion import *

# 40.44236709308751, -79.94600177667006
# x, y = utm(40.44236709308751, -79.94600177667006)
# print(x)
# print(y)

#[40.44208352622619, -79.94610353523909, 1]
#[40.44236251851889, -79.94599539848328, 1]
#[40.44233453704325, -79.94584400702514, 1]
#[40.442054721646244, -79.9459456555756, 1]

boundary_point = [[40.44208352622619, -79.94610353523909, 1], [40.44236251851889, -79.94599539848328, 1], [40.44233453704325, -79.94584400702514, 1], [40.442054721646244, -79.9459456555756, 1]]

conv = conversion()

conv.get_zone(40.44208352622619, -79.94610353523909)
for each in boundary_point:
    lat, lon = conv.get_utm(each[0], each[1])
    print('-----------------')
    print(lat)
    print(lon)
    print('-----------------')

print('----Map dimension----')
width, height = conv.boundaryConversion(boundary_point)
print(width)
print(height)

print('---origin---')
print(conv.origin_utm)


lat, lon = conv.map2gps(11, 17)
print(lat)
print(lon)

east, north =  conv.get_utm(40.44208352622619, -79.94610353523909)
print(east)
print(north)

lat, lon = conv.get_gps(east, north)
print(lat)
print(lon)
# x, y = conv.get_utm(40.44236709308751, -79.94600177667006)
# lon, lat = conv.get_gps(x, y)
# print(lon)
# print(lat)