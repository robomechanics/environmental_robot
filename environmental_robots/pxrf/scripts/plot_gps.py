#!/usr/bin/env python3
import rospy
import numpy as np
import sys
import math
import os
import wget
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg
#parameter
lat = 31.845339
lon = -101.992515
height = 5
width = 5
zoom = 20

def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = ((lon_deg + 180.0) / 360.0 * n)
    ytile = ((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return (xtile, ytile)

def num2deg(xtile, ytile, zoom):
    n = 2.0 ** zoom
    lon_deg = xtile / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
    lat_deg = math.degrees(lat_rad)
    return (lat_deg, lon_deg)

class satelliteImage(object):
    def __init__(self,coord=(0,0),dim=(0,0),zoom=0,loadFile=None):
        if loadFile is None:
            self.downloadAllTiles(coord,dim,zoom)
    def downloadAllTiles(self,coord,dim,zoom):
        x,y = deg2num(coord[0],coord[1],zoom)
        xs = np.arange(-dim[0],dim[0]+1) + round(x)
        ys = np.arange(-dim[1],dim[1]+1) + round(y)
        completeTile = None
        for i in range(len(xs)):
            verticalTile = None
            for j in range(len(ys)):
                tile = self.getMapTile(zoom,xs[i],ys[j])
                verticalTile = tile if verticalTile is None else np.concatenate((verticalTile,tile),axis=0)
            completeTile = verticalTile if completeTile is None else np.concatenate((completeTile,verticalTile),axis=1)
        print('\r')
        self.tile = completeTile
        self.x = xs[0]
        self.y = ys[0]
        self.zoom = zoom
    def pixel2Coord(self,pix):
        x = pix[0]/512.0 + self.x
        y = pix[1]/512.0 + self.y
        return num2deg(x,y,self.zoom)
    def coord2Pixel(self,coord):
        x,y = deg2num(coord[0],coord[1],self.zoom)
        x = round(512.0*(x-self.x))
        y = round(512.0*(y-self.y))
        return x,y
        
    def getMapTile(self,z,x,y):
        baseMapLink = "https://api.mapbox.com/styles/v1/mapbox/satellite-v9/tiles/512/%s/%s/%s?access_token=pk.eyJ1Ijoic2Vhbmp3YW5nIiwiYSI6ImNrb2c3d2M5bjBhcHcyb2xsd2VyNXdkNnEifQ.5uaSXmSX1HdSAlEf4LReNg"
        mapLink = baseMapLink%(z,x,y)
        mapDirectory = 'gps_navigation_satellite_tiles'
        if not os.path.isdir(mapDirectory):
            os.mkdir(mapDirectory)
        mapFile = os.path.join(mapDirectory,'satellite_%d_%d_%d'%(x,y,z))
        if not os.path.isfile(mapFile):
            wget.download(mapLink,out=mapFile)
        tile = np.asarray(mpimg.imread(mapFile))
        return tile


app = pg.QtGui.QApplication([])
mw = QtGui.QMainWindow()
satMap = satelliteImage(coord=(lat,lon),dim=(width,height),zoom=zoom)
widget = pg.LayoutWidget()
satGUI = pg.GraphicsLayoutWidget()
widget.addWidget(satGUI,row=0,col=0,colspan=8)
p = pg.PlotItem()
satGUI.addItem(p)
# show satellite image
pg.setConfigOption('imageAxisOrder', 'row-major')
img = pg.ImageItem(satMap.tile)
p.addItem(img)
p.setAspectLocked()
mw.setCentralWidget(widget)
mw.show()

if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
    QtGui.QApplication.instance().exec_()


