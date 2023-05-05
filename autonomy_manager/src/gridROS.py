import sys
sys.path.append('../')
import numpy as np
from math import *
import copy
from scipy.stats import norm
from scipy.spatial.distance import *
from boundaryCheck import *


class gridROS:
    #init 50,50, [0,0], [[1,0],[30,5],[35,40],[2,40]], 5, 15
    def __init__(self, sizex, sizey, startpoint, n_points, boundary = []):
        self.boundary =boundary
        self.sizex = sizex
        self.sizey = sizey
        self.startpoint = startpoint
        self.start_x = startpoint[0]
        self.start_y = startpoint[1]
        self.aspect = sizex/sizey
        self.n_x = int(np.round(np.sqrt(n_points * self.aspect)))
        self.n_y = int(np.round(n_points / self.n_x))
        while self.n_x * self.n_y < n_points:
            if self.n_x < self.n_y:
                self.n_x += 1
            else:
                self.n_y += 1
        
        self.spacing_x = self.sizex / (self.n_x - 1)
        self.spacing_y = self.sizey / (self.n_y - 1)

        #grid points
        self.grid_points = []
        for i in range(self.n_y):
            for j in range(self.n_x):
                x = self.start_x + j * self.spacing_x
                y = self.start_y + i * self.spacing_y
                self.grid_points.append((x, y))

    #function returns the next location to sample 
    def next(self): 
        if len(self.grid_points) > 0:
            return self.grid_points.pop(0)
        else:
            return None

    def updateBoundary(self, boundary):
        self.boundary = boundary
        self.x_bound = [coord[0] for coord in boundary]
        self.y_bound = [coord[1] for coord in boundary]
        self.x_bound.append(boundary[0][0])
        self.y_bound.append(boundary[0][1])

        #for point in self.grid_points:
        #    if not boundaryCheck(self.boundary, point):
        #        self.grid_points.remove(point)
    

