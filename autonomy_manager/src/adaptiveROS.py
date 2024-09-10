import sys

sys.path.append("../")

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF
from scipy.stats import *
from math import *

from post_processing import *
import copy
from scipy.stats import norm
from scipy.spatial.distance import *
from utils import *
import random
from sklearn.preprocessing import StandardScaler

class adaptiveROS:
    # init 50,50, [0,0], [[1,0],[30,5],[35,40],[2,40]], 5, 15
    def __init__(
        self,
        size_x,
        size_y,
        startpoint,
        total_number,
        boundary=[],
        min_dist=10,
        max_dist=40,
        sim=True,
        mode=1,
        kernel=RBF(2.0),
        startpoint_val=0
    ):
        self.size_x = size_x
        self.size_y = size_y
        self.startpoint = startpoint
        self.startpoint_val = startpoint_val
        self.min_dist = min_dist
        self.max_dist = max_dist
        self.mode = mode
        self.total_number = total_number
        self.update_boundary(boundary)
        self.kernel = kernel  # covariance function
        self.x1 = np.linspace(0, self.size_x - 1, self.size_x)
        self.x2 = np.linspace(0, self.size_y - 1, self.size_y)
        self.x1x2 = np.array([(a, b) for a in self.x1 for b in self.x2])
        
        print(f'len(x1): {len(self.x1)}, len(x2): {len(self.x2)}, len(x1x2): {self.x1x2.shape}')

        self.norm_range = 0
        self.norm_min = 0
        self.min = 0
        self.max = 0

        # initialize values
        self.delta = 25
        self.reset()        

        # scale beta from 0 to 30
        # beta balances mu and var. when the number of samples is low, the algorithm prioritizes high mean regions. when the number of samples approaches the set number, it prioritizes high-variance regions. Beta is dependent on how many samples have been collected already and delta. delta is a tuning parameter that needs to be scaled correctly based on the environment.
        self.beta = self.path_len / self.total_number * self.delta
    
    def reset(self):
        self.sampled = [self.startpoint]
        self.sampled_val = [self.startpoint_val]
        self.scaler = StandardScaler()
        
        self.path_len = 0
        self.mu = []
        self.std_var = []
        self.bin_entropy = []
        print(f'Initial sampled_val_std: {sampled_val_std}')
        self.gp = GaussianProcessRegressor(kernel=self.kernel, alpha=0, n_restarts_optimizer=50)
        sampled_val_std = self.scaler.fit_transform(np.array(self.sampled_val).reshape(-1, 1))
        print(f'Initial sampled_val_std: {sampled_val_std}')
        self.gp.fit(self.sampled, sampled_val_std) #todo: Should we re-init self.gp?
        self.first_run = True
         

    def update(self, x, y, val):
        self.sampled.append([x, y])
        self.sampled_val.append(val)
        
        self.min = np.min(self.sampled_val)
        self.max = np.max(self.sampled_val)
        self.norm_range = np.max(self.sampled_val) - np.min(self.sampled_val)
        
        self.path_len += 1
        self.beta = self.path_len / self.total_number * self.delta
        
        sampled_val_std = self.scaler.fit_transform(np.array(self.sampled_val).reshape(-1, 1))
        
        self.gp.fit(self.sampled, sampled_val_std) #todo: Should we re-init self.gp?

    # function returns the next location to sample
    def predict(self, display_plots=False):
        self.mu, self.std_var = self.gp.predict(self.x1x2, return_std=True)
        
        self.mu = np.reshape(self.mu, (self.size_x, self.size_y))
        self.std_var = np.reshape(self.std_var, (self.size_x, self.size_y))
        
        self.mu = self.scaler.inverse_transform(self.mu)
        
        # self.bin_entropy = self.mu + sqrt(self.beta) * self.std_var
        if self.first_run:
            self.first_run = False
            self.bin_entropy = np.abs(self.mu) + 3*(self.path_len / self.total_number) * self.std_var
        else:
            self.bin_entropy = np.abs(normalization(self.mu)) + 3*(self.path_len / self.total_number) * self.std_var
        
        if display_plots:
            # print(f"Bin Entropy for {self.path_len} with max(mu) = {np.max(self.mu)} max(sigma) = {np.max(self.std_var)} sqrt(self.beta) = {sqrt(self.beta)}")
            display(self.mu, self.x_bound, self.y_bound, 'mu', self.sampled)
            display(normalization(self.mu), self.x_bound, self.y_bound, 'mu', self.sampled)
            display(self.std_var, self.x_bound, self.y_bound,'sigma', self.sampled)
            display(self.bin_entropy, self.x_bound, self.y_bound, 'bin_entropy', self.sampled)
        
        
        
        bin_entropy_constraint = copy.deepcopy(self.bin_entropy)
        
        curr_x = self.sampled[-1][0]
        curr_y = self.sampled[-1][1]
        dist = np.sqrt(
            (np.arange(bin_entropy_constraint.shape[0])[:, np.newaxis] - curr_x) ** 2
            + (np.arange(bin_entropy_constraint.shape[1]) - curr_y) ** 2
        )
        idx_min = np.where(dist < self.min_dist)
        idx_max = np.where(dist > self.max_dist)
        bin_entropy_constraint[idx_min] = -1
        bin_entropy_constraint[idx_max] = -1
        
        if display_plots:
            display(bin_entropy_constraint, self.x_bound, self.y_bound, "bin_entropy constraint", self.sampled)

        # include boundary constraint
        for i_row in range(len(bin_entropy_constraint)):
            for j_col in range(len(bin_entropy_constraint[i_row])):
                # Call the function on the current coordinate
                result = boundary_check(self.boundary, [i_row, j_col])
                # Set the value of the current coordinate to the result of the function
                if not result:
                    bin_entropy_constraint[i_row][j_col] = -1
        
        if display_plots:
            display(bin_entropy_constraint, self.x_bound, self.y_bound, "after boundary constraint", self.sampled)
        
        
        # set the locations of sampled points to -1 in bin_entropy_constraint
        for i in range(len(self.sampled)):
            locx = min(int(self.sampled[i][0]), self.size_x - 1) #todo:  is smaple point ever beyond limits?
            locy = min(int(self.sampled[i][1]), self.size_y - 1)
            bin_entropy_constraint[locx][locy] = -1
        
        # if display_plots:
        #     print("remove sampled locations")
        #     display(bin_entropy_constraint)

        if self.mode == 1:  # taking distance into account
            c = 1
            
            ## TEST ##
            total_dist_to_location = np.zeros((self.size_x, self.size_y))
            for sample_location in self.sampled:
                curr = [sample_location[0], sample_location[1]]
                currDist = np.array([curr for i in range(self.x1x2.shape[0])])
                dist_to_location = np.sqrt(
                    (self.x1x2[:, 0] - currDist[:, 0]) ** 2
                    + (self.x1x2[:, 1] - currDist[:, 1]) ** 2
                )
                dist_to_location += c
                dist_to_location = dist_to_location.reshape((self.size_x, self.size_y))
                dist_to_location = np.interp(
                    dist_to_location,
                    (dist_to_location.min(), dist_to_location.max()),
                    (1, 3),
                )
                
                total_dist_to_location+=dist_to_location
            
            
            curr = [curr_x, curr_y]
            currDist = np.array([curr for i in range(self.x1x2.shape[0])])
            dist_to_location = np.sqrt(
                (self.x1x2[:, 0] - currDist[:, 0]) ** 2
                + (self.x1x2[:, 1] - currDist[:, 1]) ** 2
            )
            
            dist_to_location += c
            dist_to_location = dist_to_location.reshape((self.size_x, self.size_y))
            dist_to_location = np.interp(
                dist_to_location,
                (dist_to_location.min(), dist_to_location.max()),
                (1, 3),
            )
            dist_to_location = normalization(dist_to_location)
            if display_plots:
                display(dist_to_location, self.x_bound, self.y_bound, "dist_to_location", self.sampled)
            
            total_dist_to_location = dist_to_location
            # total_dist_to_location = normalization(total_dist_to_location)
            
            if display_plots:
                display(total_dist_to_location, self.x_bound, self.y_bound, "total_dist_to_location", self.sampled)
            ## TEST ##
            
            
            # coefficient = 0.005  # gamma
            coefficient = 0.25  # gamma
            # new_bin = bin_entropy_constraint / (dist_to_location * coefficient)
            new_bin = bin_entropy_constraint - total_dist_to_location * coefficient
            
            self.temp_new_bin = new_bin
            # r2 = np.unravel_index(new_bin.argmax(), bin_entropy_constraint.shape)
            # Random value from all max values
            all_max_indices = np.where(new_bin == np.amax(new_bin))
            max_index = np.random.choice(len(all_max_indices[0]))
            # print("max index: ", max_index)
            # print("all_max_index: ", all_max_indices)
            # print("Chosen Index: ", all_max_indices[0][max_index], all_max_indices[1][max_index])
            r2 = (all_max_indices[0][max_index], all_max_indices[1][max_index])
            # check_distribution(new_bin)
            # print(r2)
            next_x, next_y = r2[0], r2[1]
            next = [next_x, next_y]
            
            if display_plots:
                display(new_bin, self.x_bound, self.y_bound, "new_bin", self.sampled)
                
            if display_plots:
                display(new_bin, self.x_bound, self.y_bound, "new_bin with next point", self.sampled + [next])
            
            # print("dist_to_location")
            # display(dist_to_location)

        elif self.mode == 2:  # greedy approach
            # r1 = np.unravel_index(
            #     bin_entropy_constraint.argmax(), bin_entropy_constraint.shape
            # )
            
            
            next_x, next_y = r1[0], r1[1]
            next = [next_x, next_y]
        

        if display_plots:
            print("---------")
        return next

    def plot(self):
        x_bound = [coord[0] for coord in self.boundary]
        y_bound = [coord[1] for coord in self.boundary]
        x_bound.append(self.boundary[0][0])
        y_bound.append(self.boundary[0][1])

        # switch x and y of self.sampled
        self.sampled = [[i[1], i[0]] for i in self.sampled]
        visualizer(
            self.sampled,
            None,
            self.mu,
            self.bin_entropy,
            x_bound,
            y_bound,
            animate=True,
        )
        plt.show()

    def update_boundary(self, boundary):
        self.boundary = boundary
        self.x_bound = [coord[0] for coord in boundary]
        self.y_bound = [coord[1] for coord in boundary]
        
        if len(boundary):
            self.x_bound.append(boundary[0][0])
            self.y_bound.append(boundary[0][1])
        
    # Runs prediction loop with random values
    def run_prediction_loop_in_sim(self, conv, iterations):
        for i in range(iterations):
            print("----- ",i, " -----")
            
            next_loc = self.predict()
            gps = conv.map2gps(next_loc[0], next_loc[1])
            
            print("Map Offsets : ", next_loc)
            print("GPS Lat,Long: ", gps)
            print("---------------")

            self.update(next_loc[0], next_loc[1], random.random() + 0.1)


