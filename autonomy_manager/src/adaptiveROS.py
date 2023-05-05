import sys
sys.path.append('../')

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF
from scipy.stats import *
from math import *
#from environmentGeneration import *
from postProcessing import *
import copy
from scipy.stats import norm
from scipy.spatial.distance import *
from boundaryCheck import *


class adaptiveROS:
    #init 50,50, [0,0], [[1,0],[30,5],[35,40],[2,40]], 5, 15
    def __init__(self, sizex, sizey, startpoint, minDist = 4, maxDist = 10, simu = True, mode = 1, boundary = []):
        self.sizex = sizex
        self.sizey = sizey
        self.startpoint = startpoint
        self.boundary = boundary
        self.minDist = minDist
        self.maxDist = maxDist
        self.mode = mode
        self.x_bound = [coord[0] for coord in boundary]
        self.y_bound = [coord[1] for coord in boundary]
        self.kernel = RBF(4.0) # covariance function
        self.gp = GaussianProcessRegressor(kernel=self.kernel)
        self.bin_entropy = np.full((sizex,sizey), 0.5)
        self.x1 = np.linspace(0,sizex - 1, sizex)
        self.x2 = np.linspace(0,sizey - 1, sizey)
        self.x1x2 = np.array([(a,b) for a in self.x1 for b in self.x2])
        self.lastLocation = []
        self.lastMap = np.zeros((sizex,sizey))

        #initialize values
        self.sampled = [[0,0]]
        self.sampledVal = [0]
        self.mu = []
        self.std_var = []
        self.path_len = 0
        self.delta = 0.6
        self.dim = 2
        self.beta = 2*log(self.dim*((self.path_len+1)**2)*(pi**2)/(6*self.delta))

    def update(self, x, y, val):
        self.sampled.append([x,y])
        self.sampledVal.append(val)
        self.path_len += 1
        self.gp.fit(self.sampled, self.sampledVal)
      

    def reset(self):
        self.sampled = []
        self.sampledVal = []
        self.path_len = 0
        self.mu = []
        self.bin_entropy = []
        self.std_var = []
        self.gp = GaussianProcessRegressor(kernel=self.kernel)
    
    #function returns the next location to sample 
    def predict(self): 
        self.mu, self.std_var = self.gp.predict(self.x1x2, return_std=True)
        self.mu = np.reshape(self.mu, (self.sizex, self.sizey))
        self.std_var = np.reshape(self.std_var, (self.sizex, self.sizey))
        self.bin_entropy = self.mu + sqrt(self.beta) * self.std_var
        bin_entropy_constraint = copy.deepcopy(self.bin_entropy)
        currx = self.sampled[-1][0]
        curry = self.sampled[-1][1]
        dist = np.sqrt((np.arange(bin_entropy_constraint.shape[0])[:,np.newaxis] - currx)**2 + (np.arange(bin_entropy_constraint.shape[1]) - curry)**2)
        idx_min = np.where(dist < self.minDist)
        idx_max = np.where(dist > self.maxDist)
        bin_entropy_constraint[idx_min] = -1 
        bin_entropy_constraint[idx_max] = -1    
 
        #include boundary constraint
        for i_row in range(len(bin_entropy_constraint)):
            for j_col in range(len(bin_entropy_constraint[i_row])):
        # Call the function on the current coordinate
                result = boundaryCheck(self.boundary, [i_row,j_col])
        # Set the value of the current coordinate to the result of the function
                if not result:
                    bin_entropy_constraint[i_row][j_col] = -1
    
        if self.mode == 1: # taking distance into account
            c = 1
            curr = [currx, curry]
            currDist = np.array([curr for i in range(self.x1x2.shape[0])])
            dist_to_location = np.sqrt((self.x1x2[:, 0] - currDist[:,0])**2 + (self.x1x2[:,1] - currDist[:,1])**2)
            dist_to_location += c
            dist_to_location = dist_to_location.reshape((self.sizex ,self.sizey))
            dist_to_location = np.interp(dist_to_location, (dist_to_location.min(), dist_to_location.max()), (1, 3))
            coeficient = 0.1

            new_bin = bin_entropy_constraint / (dist_to_location * coeficient)
            r2 = np.unravel_index(new_bin.argmax(), bin_entropy_constraint.shape)
            next_x, next_y = r2[0], r2[1]
            next = [next_x, next_y]

            #if the next location is the same as the last location, then we need to find a new next location
            while(next in self.lastLocation):
                print("repeated location, find a new location")
                new_bin[r2] = -1
                r2 = np.unravel_index(new_bin.argmax(), bin_entropy_constraint.shape)
                next_x, next_y = r2[0], r2[1]
                next = [next_x, next_y]

        elif self.mode == 2: # greedy approach
            r1 = np.unravel_index(bin_entropy_constraint.argmax(), bin_entropy_constraint.shape)
            next_x, next_y = r1[0], r1[1]
            next = [next_x, next_y]
        
        #check if two arrays are the same
        if self.lastMap.all() != bin_entropy_constraint.all():
            self.lastLocation = []

        self.lastLocation.append(next)
        self.lastMap = bin_entropy_constraint

        return next

    def plot(self):
        x_bound = [coord[0] for coord in self.boundary]
        y_bound = [coord[1] for coord in self.boundary]
        x_bound.append(self.boundary[0][0])
        y_bound.append(self.boundary[0][1])
        visualizer(self.sampled, None, self.mu, self.bin_entropy, x_bound, y_bound, animate = False)
        plt.show()
    
    def updateBoundary(self, boundary):
        self.boundary = boundary
        self.x_bound = [coord[0] for coord in boundary]
        self.y_bound = [coord[1] for coord in boundary]
        self.x_bound.append(boundary[0][0])
        self.y_bound.append(boundary[0][1])

