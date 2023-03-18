#!/usr/bin/env python3
import sys
sys.path.append('../') 
from adaptiveROS import adaptiveROS
from boundaryConversion import conversion

#[40.44208352622619, -79.94610353523909, 1]
#[40.44236251851889, -79.94599539848328, 1]
#[40.44233453704325, -79.94584400702514, 1]
#[40.442054721646244, -79.9459456555756, 1]

boundary = [[40.44208352622619, -79.94610353523909], [40.44236251851889, -79.94599539848328], [40.44233453704325, -79.94584400702514], [40.442054721646244, -79.9459456555756]]

conv = conversion()
conv.get_zone(40.44208352622619, -79.94610353523909)

boundary_utm_offset = conv.boundaryConversion(boundary)
for each in conv.new_boundary:
    print(each)
print('--------1---------')
for each in boundary_utm_offset:
    print(each)
print('--------2---------')
adaptive = adaptiveROS(conv.width, conv.height, [0,0])
adaptive.updateBoundary(boundary_utm_offset)
next_loc = adaptive.predict()
print(next_loc)
print('--------3---------')
adaptive.update(3,10,0.5)
next_loc = adaptive.predict()
print(next_loc)
print('--------4---------')
