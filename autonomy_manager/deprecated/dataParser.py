import csv
import os
from adaptiveROS import adaptiveROS
from boundaryConversion import conversion
import re
from copy import copy
from itertools import product
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import scipy as sp
import scipy.ndimage
from scipy.interpolate import griddata
from sklearn import preprocessing
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
from sklearn.gaussian_process.kernels import Matern, WhiteKernel, ConstantKernel
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.colors as colors
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
from environmentGeneration import generateRandomDistribution

def visualizer(sampled, surface_mu):
    fig = plt.figure()
    A_plot = plt.imshow(surface_mu, cmap='YlGn', interpolation='nearest')
    plt.colorbar(A_plot)
    plt.plot(adaptive.x_bound, adaptive.y_bound, linewidth=3, color=(0.733,0,0))

    sampled = np.asarray(sampled, np.dtype('int','int'))
    plt.plot(sampled[:,1], sampled[:,0], '.', color=line_color)
    plt.xlim(0, int(conv.width))
    plt.ylim(0, int(conv.height))
    plt.show()

file_path = '/Users/slothysloth/Dropbox/Mac/Desktop/HEBI Robotics/system/hebi/autonomy_manager/2023-03-30.csv'

data = []
max = 0
min = 0

current_directory = os.getcwd()
print("Current working directory:", current_directory)

# Open the CSV file in read mode
with open(file_path, 'r') as csv_file:
    # Create a CSV reader object
    csv_reader = csv.reader(csv_file)
    
    # Iterate through each row in the CSV file
    row_num = 0
    data = []
    final = []
    for row in csv_reader:
        # if the row number is 0, 4, 8, ...
        if row_num % 4 == 0:
            #get the last element 
            data.append(row[-1])
        elif row_num % 4 == 1:
            row_num += 1
            continue
        elif row_num % 4 == 2:
            #get the 5th element from the end
            data.append(10000 * float(row[-5]))
            if max < float(row[-5]):
                max = float(row[-5]) * 10000
            if min > float(row[-5]):
                min = float(row[-5]) * 10000
        elif row_num % 4 == 3:
            #get the 5th element from the end
            #data.append(row[-5])
            final.append(data)
            data = []
        row_num += 1

#boundary
boundary = [[39.189249, -84.763784], [39.189280, -84.763434], [39.189549,-84.763486], [39.189513, -84.763819]]
conv = Conversion()
conv.get_zone(39.189249, -84.763784)
boundary_utm_offset = conv.boundaryConversion(boundary)
print(conv.width, conv.height)
adaptive = adaptiveROS(conv.width, conv.height, [0,0], len(final))
adaptive.updateBoundary(boundary_utm_offset)
pattern = r"[-+]?\d+\.\d+"

# normalize the data
# for each in final:
#     final[final.index(each)][1] = (each[1] - min) / (max - min)

for each in final:
    numbers = re.findall(pattern, each[0])
    latitude = float(numbers[0])
    longitude = float(numbers[1])
    pos = conv.gps2map(latitude, longitude)
    print(pos[0], pos[1], each[1])
    adaptive.update(pos[0], pos[1], each[1])
    adaptive.predict()
    #print(adaptive.mu)

line_color = (0.733,0,0)
surface_mu = adaptive.mu 
sampled = [[i[0], i[1]] for i in adaptive.sampled]
visualizer(sampled, surface_mu)



manual_loc = [[7,3], [6,9], [5,15], [4,21], [3,27], [9,28], [19,5], [18,11], [17,17], [16,23], [15,28], [23,18], [22,23], [21,29],
          [29,18], [28,24], [27,30]]
#flip x and y of each element in manual_loc
for each in manual_loc:
    each[0], each[1] = each[1], each[0]

###########################################
manual_val = [441,371,478,340,58,72,588,573,683,424,252,649,400,205,547,390,61]
kernel = RBF(2.0)  # covariance function
gp = GaussianProcessRegressor(kernel=kernel)
gp.fit(manual_loc, manual_val)
surface_mu_manual, std_var = gp.predict(adaptive.x1x2, return_std=True)
surface_mu_manual = np.reshape(surface_mu_manual, (adaptive.sizex, adaptive.sizey))
visualizer(manual_loc, surface_mu_manual)

###########################################
#loop through all the points in surface_mu_manual and surface_mu at the same time 
total = adaptive.sizex * adaptive.sizey
count = 0
for i in range(adaptive.sizex):
    for j in range(adaptive.sizey):
        if abs(surface_mu_manual[i][j] - surface_mu[i][j]) < 150:
            count += 1

print(count/total)