from copy import copy

from manager import * 
import rosnode
import time
from matplotlib import pyplot as plt

from env_utils.algo_constants import *
from env_utils.sim_utils import *
from sklearn.gaussian_process.kernels import RBF
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.preprocessing import StandardScaler
import numpy as np
from boundaryConversion import Conversion

import numpy as np
import scipy as sp
import scipy.ndimage
from scipy.interpolate import griddata

from adaptiveROS import *
import csv
import os
import yaml

def extract_data_from_disk(file_path, element="Cl", filter=False):
    column_number = None
    row_numbers = []
    data_values = []
    gps_locations= []
    odom_locations = []

    with open(file_path, mode='r', newline='') as file:
        reader = csv.reader(file)
        for row_index, row in enumerate(reader):
            if column_number is None:
                for col_index, value in enumerate(row):
                    if value == element:
                        column_number = col_index
                        break
    
    with open(file_path, mode='r', newline='') as file:
        reader = csv.reader(file)
        for row_index, row in enumerate(reader):
            if row[column_number] == element:
                row_numbers.append(row_index + 1)

    with open(file_path, mode='r', newline='') as file:
        reader = csv.reader(file)
        for row_index, row in enumerate(reader):
            if row_index in row_numbers:
                data_values.append(float(row[column_number]))
    
    with open(file_path, mode='r', newline='') as file:
        reader = csv.reader(file)
        first_daily_id = None
        next_daily_id = None
        for row_index, row in enumerate(reader):
            # print(first_daily_id, next_daily_id)
            if first_daily_id is None:
                first_daily_id = row[0]
                gps_locations.append(eval(row[5]))
                odom_locations.append(eval(row[6]))
                next_daily_id = int(first_daily_id) + 1
            else:
                try:
                    if int(row[0]) == next_daily_id:
                        gps_locations.append(eval(row[5]))
                        odom_locations.append(eval(row[6]))
                        next_daily_id = next_daily_id + 1
                except:
                    pass
                    
            
    if filter:
        data_values = data_values[::2]
                
    return data_values, gps_locations, odom_locations

class Real2Sim():
    def __init__(self,
                 data_dir,
                 kernel,
                 sample_multiplier,
                 alpha=0.25,
                 filter_data=False,
                 cells_per_meter=4
                 ):
        
        self.data_dir = data_dir
        self._filter_data = filter_data
        self._cells_per_meter = cells_per_meter
        self.kernel = kernel
        self.sample_multiplier = sample_multiplier
        
        self.load_from_disk()
        
        # Setup Conversion
        self.c = Conversion(self._cells_per_meter)
        self.c.get_zone(self.gps_locations[0][0], self.gps_locations[0][1])
        self.boundary_utm_offset = self.c.boundary_conversion(self.boundary_in_gps)
        
        self.startx, self.starty = self.c.gps2map(self.gps_locations[0][0], 
                                                  self.gps_locations[0][1])
        self.startx_in_grid, self.starty_in_grid = self.c.map2grid(self.startx, self.starty)
        self.width_in_grid = self.c.width * self._cells_per_meter
        self.height_in_grid = self.c.height * self._cells_per_meter

        self.boundary_in_grid = [self.c.map2grid(p[0], p[1]) for p in self.boundary_utm_offset]

        self.locations_in_map = [self.c.gps2map(pos[0], pos[1]) for pos in self.gps_locations]
        self.locations_in_grid = [self.c.map2grid(pos[0], pos[1]) for pos in self.locations_in_map]
                
        # GP Params
        self.size_x=self.width_in_grid
        self.size_y=self.height_in_grid
        self.x1 = np.linspace(0, self.size_x - 1, self.size_x)
        self.x2 = np.linspace(0, self.size_y - 1, self.size_y)
        self.x1x2 = np.array([(a, b) for a in self.x1 for b in self.x2])
        
        self.scaler = StandardScaler()
        self.sampled_val_std = self.scaler.fit_transform(np.array(self.sampled_values).reshape(-1, 1)).ravel()
        # print(f'sampled_val_std: {sampled_val_std}')
        
        self.gp = GaussianProcessRegressor(kernel=self.kernel, 
                                           alpha=0.25, 
                                           n_restarts_optimizer=100)
        

    def load_from_disk(self):
        self._csv_path = os.path.join(self.data_dir, "scan_results_adaptive.csv")
        self._params_path = os.path.join(self.data_dir, "autonomy_params.yaml")
        
        with open('/home/proboticks/catkin_ws/patrick_data/nrec/26-09-2024_16:09:09/autonomy_params.yaml', 'r') as file:
            self._yaml_data = yaml.safe_load(file)

            # Get the list from the YAML data
            self.boundary_in_gps = list(zip(self._yaml_data['boundary_lat'], self._yaml_data['boundary_lon']))
                
        
        self.sampled_values, self.gps_locations, self.odom_locations = extract_data_from_disk(file_path=self._csv_path,
                                                                                              filter=self._filter_data)
        
        self.sampled_values = np.asarray(self.sampled_values) * self.sample_multiplier
        self.sampled_values = self.sampled_values.tolist()
        print(f"Samples (in ppm): {self.sampled_values}")
        print(f"Total Samples   : {len(self.sampled_values)}")

    def fit(self):
        self.gp.fit(self.locations_in_grid, self.sampled_val_std)

        self.mu, self.std_var = self.gp.predict(self.x1x2, return_std=True)
        self.mu = np.reshape(self.mu, (self.size_x, self.size_y))
        self.std_var = np.reshape(self.std_var, (self.size_x, self.size_y))
        self.mu = self.scaler.inverse_transform(self.mu)

    def show(self):
        x_bound = [coord[0] for coord in self.boundary_in_grid]
        y_bound = [coord[1] for coord in self.boundary_in_grid]

        self.bin_entropy = np.abs(normalization(self.mu)) + 3 * self.std_var

        # switch x and y of sampled
        self.sample_locations_plot = [[i[1], i[0]] for i in self.locations_in_grid]
        # visualizer(
        #     self.sample_locations_plot,
        #     None,
        #     self.mu,
        #     self.bin_entropy,
        #     x_bound,
        #     y_bound,
        #     animate=True,
        # )
        
        visualizer_recreate_real(
            sampled=self.sample_locations_plot,
            surface_mu=self.mu,
            x_bound=x_bound,
            y_bound=y_bound,
            predicted_mapsize=(self.size_x, self.size_y),
            save_to_disk=False,
            filename=None
        )
        
        
        plt.show()


       