import wget
import sys
import os
from urllib.error import HTTPError
import numpy as np
import math
import matplotlib.image as mpimg

def deg2num(lat_deg: float, lon_deg: float, zoom: int):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = (lon_deg + 180.0) / 360.0 * n
    ytile = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n
    return (xtile, ytile)

def num2deg(xtile: float, ytile: float, zoom: int):
    n = 2.0 ** zoom
    lon_deg = xtile / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
    lat_deg = math.degrees(lat_rad)
    return (lat_deg, lon_deg)

class TileMap:
    TILE_SIZE_PX = 512

    def __init__(self, coord=(0,0), dim=(0,0), zoom=0):
        self.load_tiles(coord, dim, zoom)

    def load_tiles(self, coord, dim, zoom: int):
        lon: float = coord[0]
        lat: float = coord[1]
        
        self.lon = coord[0]
        self.lat = coord[1]
        
        width: int = dim[0]
        height: int = dim[1] 
        print(f'Window at Lon: {lon}, Lat: {lat}, W: {width}, H: {height}')

        x, y = deg2num(lon, lat, zoom)
        x = int(x)
        y = int(y)
        print(f'Tile X,Y: {x}, {y}')

        x_start = x - width
        x_end = x + width + 1
        y_start = y - height
        y_end = y + height + 1

        print(f'X from {x_start} to {x_end}')
        print(f'Y from {y_start} to {y_end}')

        completeTile = None
        #for i in range(len(xs)):
        for i in range(x_start, x_end):
            verticalTile = None
            #for j in range(len(ys)):
            for j in range(y_start, y_end):
                tile = self.getMapTile(zoom, int(i), int(j))
                verticalTile = tile if verticalTile is None else np.concatenate((verticalTile, tile), axis=0)
            completeTile = verticalTile if completeTile is None else np.concatenate((completeTile, verticalTile), axis=1)

        print('----------')
        self.map_array = completeTile
        self.x = x_start
        self.y = y_start
        self.zoom = zoom

    def pixel2Coord(self, pix):
        x = float(pix[0]) / self.TILE_SIZE_PX + self.x
        y = float(pix[1]) / self.TILE_SIZE_PX + self.y
        return num2deg(x, y, self.zoom)

    def coord2Pixel(self, lat: float, lon: float):
        x,y = deg2num(lat, lon, self.zoom)
        x = round(self.TILE_SIZE_PX * (x-self.x))
        y = round(self.TILE_SIZE_PX * (y-self.y))
        return x, y
        
    def getMapTile(self, zoom: int, x: int, y: int):
        mapDirectory = os.path.join(
            os.path.abspath(os.path.dirname(__file__)),
            'gps_navigation_satellite_tiles')

        if not os.path.isdir(mapDirectory):
            os.mkdir(mapDirectory)

        mapFile = os.path.join(mapDirectory, f'satellite_{zoom}_{x}_{y}.jpeg')

        if not os.path.isfile(mapFile):
            print(f"\rDownloading missing map tile x:{x} y:{y} zoom:{zoom}", end='')
            access_token = "pk.eyJ1Ijoic2Vhbmp3YW5nIiwiYSI6ImNrb2c3d2M5bjBhcHcyb2xsd2VyNXdkNnEifQ.5uaSXmSX1HdSAlEf4LReNg"
            mapLink = f"https://api.mapbox.com/styles/v1/mapbox/satellite-v9/tiles/512/{zoom}/{x}/{y}?access_token={access_token}"

            try:
                wget.download(mapLink, out=mapFile)
            except HTTPError:
                print(f'Bad Url: {mapLink}')
        tile = np.asarray(mpimg.imread(mapFile))
        return tile
