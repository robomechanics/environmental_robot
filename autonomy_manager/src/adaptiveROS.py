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
from sklearn.gaussian_process.kernels import Matern

class adaptiveROS:
    #init 50,50, [0,0], [[1,0],[30,5],[35,40],[2,40]], 5, 15
    def __init__(self, sizex, sizey, startpoint, total_number, minDist = 0, maxDist = 100,
                 simu = True, mode = 1, boundary = [], kernel = RBF(8e-4)): #Matern(length_scale=5, nu=0.5
        self.sizex = sizex
        self.sizey = sizey
        self.startpoint = startpoint
        self.boundary = boundary
        self.minDist = minDist
        self.maxDist = maxDist
        self.mode = mode
        self.total_number = total_number
        self.x_bound = [coord[0] for coord in boundary]
        self.y_bound = [coord[1] for coord in boundary]
        self.kernel = kernel # covariance function
        self.gp = GaussianProcessRegressor(kernel=self.kernel, n_restarts_optimizer=50)
        self.bin_entropy = np.full((sizex,sizey), 0.5)
        self.x1 = np.linspace(0,sizex - 1, sizex)
        self.x2 = np.linspace(0,sizey - 1, sizey)
        self.x1x2 = np.array([(a,b) for a in self.x1 for b in self.x2])

        self.norm_range = 0
        self.norm_min = 0
        self.min = 0
        self.max = 0

        #initialize values
        self.sampled = [[0,0]]
        self.sampledVal = [0]
        self.mu = []
        self.std_var = []
        self.path_len = 0
        self.delta = 1
        
        #scale beta from 0 to 30
        #beta balances mu and var. when the number of samples is low, the algorithm prioritizes high mean regions. when the number of samples approaches the set number, it prioritizes high-variance regions. Beta is dependent on how many samples have been collected already and delta. delta is a tuning parameter that needs to be scaled correctly based on the environment.
        #self.beta = self.path_len / self.total_number * self.delta
        self.beta1 = self.delta

    def update(self, x, y, val):
        self.sampled.append([x,y])
        self.sampledVal.append(val)
        self.min = np.min(self.sampledVal)
        self.max = np.max(self.sampledVal)
        self.norm_range = np.max(self.sampledVal) - np.min(self.sampledVal)
        self.norm_min = np.min(self.sampledVal)
        sampledVal_n = list(np.array(self.sampledVal-self.norm_min)/self.norm_range)
        self.path_len += 1
        #self.beta = self.path_len / self.total_number * self.delta
        self.beta1 = self.delta
        self.gp.fit(self.sampled, sampledVal_n)

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
        # print("@@@@@@@@@@@@@@@@@")
        # print(self.mu)
        # print("-----------------")

        # print(self.std_var)
        self.bin_entropy = 0*self.mu + sqrt(self.beta1) * self.std_var
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
    
        # set the locations of sampled points to -1 in bin_entropy_constraint
        for i in range(len(self.sampled)):
            locx = min(int(self.sampled[i][0]), self.sizex - 1)
            locy = min(int(self.sampled[i][1]), self.sizey - 1)
            bin_entropy_constraint[locx][locy] = -1

        if self.mode == 1: # taking distance into account
            c = 1
            curr = [currx, curry]
            currDist = np.array([curr for i in range(self.x1x2.shape[0])])
            dist_to_location = np.sqrt((self.x1x2[:, 0] - currDist[:,0])**2 + (self.x1x2[:,1] - currDist[:,1])**2)
            dist_to_location += c
            dist_to_location = dist_to_location.reshape((self.sizex ,self.sizey))
            dist_to_location = np.interp(dist_to_location, (dist_to_location.min(), dist_to_location.max()), (1, 3))
            coeficient = 0.5 #gamma 0.005
            #print(dist_to_location)

            #new_bin = bin_entropy_constraint / (dist_to_location * coeficient)
            new_bin = bin_entropy_constraint - dist_to_location * coeficient
            r2 = np.unravel_index(new_bin.argmax(), bin_entropy_constraint.shape)
            next_x, next_y = r2[0], r2[1]
            next = [next_x, next_y]
            #print(next)

        elif self.mode == 2: # greedy approach
            r1 = np.unravel_index(bin_entropy_constraint.argmax(), bin_entropy_constraint.shape)
            next_x, next_y = r1[0], r1[1]
            next = [next_x, next_y]
        
        return next

    def plot(self):
        x_bound = [coord[0] for coord in self.boundary]
        y_bound = [coord[1] for coord in self.boundary]
        x_bound.append(self.boundary[0][0])
        y_bound.append(self.boundary[0][1])

        #switch x and y of self.sampled 
        self.sampled = [[i[1], i[0]] for i in self.sampled]
        visualizer(self.sampled, None, self.mu, self.bin_entropy, x_bound, y_bound, animate = False)
        plt.show()
    
    def updateBoundary(self, boundary):
        self.boundary = boundary
        self.x_bound = [coord[0] for coord in boundary]
        self.y_bound = [coord[1] for coord in boundary]
        self.x_bound.append(boundary[0][0])
        self.y_bound.append(boundary[0][1])