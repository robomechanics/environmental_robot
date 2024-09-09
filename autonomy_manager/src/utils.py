import numpy as np
from scipy.stats import wasserstein_distance
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
import matplotlib.pyplot as plt
import math
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

import matplotlib.pyplot as plt
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF
import scipy.stats as st
from numpy.ma.core import absolute
import libpysal
from libpysal import weights
from libpysal.weights import Queen
from esda.moran import Moran
from libpysal.weights import lat2W
from libpysal.weights import Queen
from esda.moran import Moran_Local
import geopandas as gpd
from matplotlib import pyplot as plt
import os
from pathlib import Path
import random
from sklearn.preprocessing import StandardScaler


def is_standardized(arr):
    return np.std(arr) == 1.0

def is_normalized(arr):
    return (np.min(arr) == 0.0 and np.max(arr) == 1.0)

def check_distribution(arr, title=None):
    if title:
        print(f'----- {title} ----' )
    print(f' Normalized  : {is_normalized(arr)}')
    print(f' Standardized: {is_standardized(arr)}')
    print(f' Min         : {np.min(arr)}')
    print(f' Max         : {np.max(arr)}')
    print(f' Mean        : {np.mean(arr)}')
    print('-------------------')
    
def get_os_paths(data_directory):
    cwd_path = Path(os.getcwd())
    pwd_path = cwd_path.parent.absolute()
    data_path = Path.joinpath(pwd_path, data_directory)
    return cwd_path, pwd_path, data_path


marker_color = (0.8, 0, 0)
line_color = (0,0,0.6)
annotation_color = (0,0,0)

def display(env_map, 
            x_bound,
            y_bound,
            title = None,
            sampled = None,
            figsize = (3,3)):
    
    fig, (ax) = plt.subplots(1, 1, figsize=figsize)
        
    plot = ax.imshow(env_map, cmap="YlGn", interpolation='nearest')
    cax = inset_axes(
        ax,
        width="5%",
        height="100%",
        loc="lower left",
        bbox_to_anchor=(1.05, 0.0, 1, 1),
        bbox_transform=ax.transAxes,
        borderpad=0,
    )
    cbar = fig.colorbar(plot, cax=cax)
    ax.invert_yaxis()
    if title:
        ax.set_title(title)
        
    ax.plot(x_bound, y_bound, linewidth=2, color=(0, 0, 0.6))
    if sampled:
        sampled = np.asarray(sampled, np.dtype("int", "int"))
        ax.plot(sampled[:, 1], sampled[:, 0], ".", color=marker_color, markersize=14)
        
        # Annotations
        for i in range(sampled.shape[0]):
            ax.annotate(str(i), (sampled[i][1], sampled[i][0]), textcoords="offset points", xytext=(0,-4), ha='center', fontsize=7, color=annotation_color)
 
    plt.show()

def morans_i(map, threshold = 0.5, print_stats = False):
    binary = (normalization(map) > threshold).astype(int)
    x_coords = np.arange(binary.shape[1])  # Assuming the x coordinates
    y_coords = np.arange(binary.shape[0])  # Assuming the y coordinates
    x, y = np.meshgrid(x_coords, y_coords)
    pixel_gdf = gpd.GeoDataFrame(geometry=gpd.points_from_xy(x.ravel(), y.ravel()))
    pixel_gdf['value'] = binary.ravel()
    # Define spatial weights
    w = Queen.from_dataframe(pixel_gdf, use_index=False)
    moran = Moran(binary, w)
    moran_loc = Moran_Local(pixel_gdf['value'], w)
    areal_extent = np.count_nonzero(binary)/(binary.shape[0]*binary.shape[1])
    
    if print_stats:
            print("Areal Extent   :", areal_extent)
            print("Moran's I      :", moran.I)
            print("p-value        :", moran.p_sim)
            print("Local Moran's I:", moran_loc.Is)
            
    return areal_extent, moran.I, moran.p_sim, moran_loc.Is
 
def normalization(x):
    np_max = np.max(x)
    np_min = np.min(x)
    if np_min != np_max:
        x = (x - np_min) / (np_max - np_min)
    return x

def standardization(x):
    mu = np.mean(x)
    std_dev = np.std(x)
    return (x-mu)/std_dev


def prob(x):
    x = x / np.sum(x)
    return x


def emd(array1, array2):
    a1 = array1.ravel()
    a2 = array2.ravel()
    return wasserstein_distance(a1, a2)


def emd_n(array1, array2):
    a1 = array1.ravel()
    a2 = array2.ravel()
    a1_n = normalization(a1)
    a2_n = normalization(a2)
    return wasserstein_distance(a1_n, a2_n)


def emd_p(array1, array2):
    a1 = array1.ravel()
    a2 = array2.ravel()
    a1_p = prob(a1)
    a2_p = prob(a2)
    return wasserstein_distance(a1_p, a2_p)


def grid(mapsize, env, points, kernel=RBF(2.0)):
    x1 = np.linspace(0, mapsize[0] - 1, mapsize[0])
    x2 = np.linspace(0, mapsize[1] - 1, mapsize[1])
    x1x2 = np.array([(a, b) for a in x1 for b in x2])

    gp = GaussianProcessRegressor(kernel=kernel, alpha= 0, n_restarts_optimizer=50)

    # store each points location in env in an array
    val = []
    for i in range(len(points)):
        val.append(env[points[i][0]][points[i][1]])
    
    scaler = StandardScaler()
    std_val = scaler.fit_transform(np.array(val).reshape(-1, 1))
    
    gp.fit(points, std_val)
    
    surface_mu, std_var = gp.predict(x1x2, return_std=True)
    surface_mu = np.reshape(surface_mu, (mapsize[0], mapsize[1]))
    
    surface_mu = scaler.inverse_transform(surface_mu)
    
    return surface_mu, np.min(val), np.max(val)


def visualizer_recreate(
    env_map,
    sampled,
    surface_mu,
    grid_mu,
    grid_sampled,
    adaptive,
    predicted_mapsize,
    wspace=1,
    save_to_disk=False,
    filename="plot.png"
):
    # display env_map as subplot
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(12, 10))
    
    
    vmin = min(env_map.min(), surface_mu.min(), grid_mu.min())
    vmax = max(env_map.max(), surface_mu.max(), grid_mu.max())
    
    # print(f'Min: {vmin} | Max: {vmax}')

    A_plot1 = ax1.imshow(env_map, cmap="YlGn", vmin=vmin, vmax=vmax, interpolation="nearest")
    # cbar1 = fig.colorbar(A_plot1, ax=ax1)
    cax1 = inset_axes(
        ax1,
        width="5%",
        height="100%",
        loc="lower left",
        bbox_to_anchor=(1.05, 0.0, 1, 1),
        bbox_transform=ax1.transAxes,
        borderpad=0,
    )
    cbar1 = fig.colorbar(A_plot1, cax=cax1)
    ax1.set_title("Ground Truth")
    ax1.set_xlim(0, predicted_mapsize[0])
    ax1.set_ylim(0, predicted_mapsize[1])
    ax1.plot(adaptive.x_bound, adaptive.y_bound, linewidth=2, color=line_color)

    A_plot2 = ax2.imshow(surface_mu, cmap="YlGn", vmin=vmin, vmax=vmax, interpolation="nearest")
    # cbar2 = fig.colorbar(A_plot2, ax=ax2)
    cax2 = inset_axes(
        ax2,
        width="5%",
        height="100%",
        loc="lower left",
        bbox_to_anchor=(1.05, 0.0, 1, 1),
        bbox_transform=ax2.transAxes,
        borderpad=0,
    )
    cbar2 = fig.colorbar(A_plot2, cax=cax2)
    # cbar2.set_ticks(np.linspace(vmin, vmax, 4))
    
    ax2.set_title("Adaptive - Reconstructed")
    ax2.plot(adaptive.x_bound, adaptive.y_bound, linewidth=2, color=line_color)
    sampled = np.asarray(sampled, np.dtype("int", "int"))
    ax2.plot(sampled[:, 1], sampled[:, 0], ".", color=marker_color, markersize=10)
    ax2.set_xlim(0, predicted_mapsize[0])
    ax2.set_ylim(0, predicted_mapsize[1])
    # Annotations
    for i in range(sampled.shape[0]):
        ax2.annotate(str(i), (sampled[i][1], sampled[i][0]), textcoords="offset points", xytext=(0,-3), ha='center', fontsize=6, color=annotation_color)


    # the third graph
    A_plot3 = ax3.imshow(grid_mu, cmap="YlGn", vmin=vmin, vmax=vmax, interpolation="nearest")
    # cbar3 = fig.colorbar(A_plot3, ax=ax3)
    cax3 = inset_axes(
        ax3,
        width="5%",
        height="100%",
        loc="lower left",
        bbox_to_anchor=(1.05, 0.0, 1, 1),
        bbox_transform=ax3.transAxes,
        borderpad=0,
    )
    cbar3 = fig.colorbar(A_plot3, cax=cax3)
    grid_sampled = np.asarray(grid_sampled, np.dtype("int", "int"))
    ax3.plot(grid_sampled[:, 1], grid_sampled[:, 0], ".", color=marker_color, markersize=10)
    ax3.plot(adaptive.x_bound, adaptive.y_bound, linewidth=2, color=line_color)
    ax3.set_title("Grid - Reconstructed")
    ax3.set_xlim(0, predicted_mapsize[0])
    ax3.set_ylim(0, predicted_mapsize[1])
    # Annotations
    # for i in range(grid_sampled.shape[0]):
    #     ax3.annotate(str(i), (grid_sampled[i][1], grid_sampled[i][0]), textcoords="offset points", xytext=(1,1), ha='center', fontsize=5)


    fig.subplots_adjust(wspace=0.5)
    # fig.tight_layout()
    
    if save_to_disk:
        plt.savefig(filename, format='png', dpi=300)
    else:
        plt.show()



def visualizer_recreate_real(
    sampled,
    surface_mu,
    adaptive,
    predicted_mapsize,
    save_to_disk=False,
    filename="plot.png"
):
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))
    
    
    vmin = surface_mu.min()
    vmax = surface_mu.max()
    # print(f'Min: {vmin} | Max: {vmax}')

    A_plot = ax.imshow(surface_mu, cmap="YlGn", vmin=vmin, vmax=vmax, interpolation="nearest")
    # cbar2 = fig.colorbar(A_plot, ax=ax)
    cax = inset_axes(
        ax,
        width="5%",
        height="100%",
        loc="lower left",
        bbox_to_anchor=(1.05, 0.0, 1, 1),
        bbox_transform=ax.transAxes,
        borderpad=0,
    )
    cbar2 = fig.colorbar(A_plot, cax=cax)
    # cbar2.set_ticks(np.linspace(vmin, vmax, 4))
    
    ax.set_title("Adaptive - Reconstructed")
    ax.plot(adaptive.x_bound, adaptive.y_bound, linewidth=2, color=line_color)
    sampled = np.asarray(sampled, np.dtype("int", "int"))
    ax.plot(sampled[:, 1], sampled[:, 0], ".", color=marker_color, markersize=10)
    ax.set_xlim(0, predicted_mapsize[0])
    ax.set_ylim(0, predicted_mapsize[1])
    
    # Annotations
    for i in range(sampled.shape[0]):
        ax.annotate(str(i), (sampled[i][1], sampled[i][0]), 
                    textcoords="offset points", xytext=(0,-3), 
                    ha='center', fontsize=6, color=annotation_color)

    fig.subplots_adjust(wspace=0.5)
    # fig.tight_layout()
    
    if save_to_disk:
        plt.savefig(filename, format='png', dpi=300)
    else:
        plt.show()


def score(env_map, surface_mu, tolerance):
    # check the difference between each cell
    # if the difference is within tolerance, then count it as correct
    total_cells = len(env_map) * len(env_map[0])
    correct_cells = 0
    error = 0
    for i in range(len(env_map)):
        for j in range(len(env_map[0])):
            error += abs(env_map[i][j] - surface_mu[i][j])
            if env_map[i][j]:
                if (abs(env_map[i][j] - surface_mu[i][j]) / env_map[i][j]) < tolerance:
                    correct_cells += 1
    score = correct_cells / total_cells

    return score, error


def generate_points(dimensions, num_points, boundary_padding):
    width = dimensions[0]
    height = dimensions[1]
    points = []
    num_points_per_row = math.ceil(math.sqrt(num_points))
    num_points_per_col = math.ceil(num_points / num_points_per_row)

    spacing_x = width / (num_points_per_row + 1)
    spacing_y = height / (num_points_per_col + 1)

    count = 0
    for i in range(num_points_per_col):
        for j in range(num_points_per_row):
            x = (j + 1) * spacing_x + boundary_padding
            y = (i + 1) * spacing_y + boundary_padding

            if count < num_points:
                points.append((int(x), int(y)))
                count += 1
            else:
                break
    return points


# input the polygon points in clockwise order
def boundary_check(polygon, point):
    polygon = np.array(polygon)
    polygon = [tuple(row) for row in polygon]
    polygon = Polygon(polygon)
    point = Point(tuple(point))
    return point.within(polygon)    


# Ref: https://stackoverflow.com/questions/3173320/text-progress-bar-in-terminal-with-block-characters
# Print iterations progress
def printProgressBar (iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '█', printEnd = "\r"):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)
    # Print New Line on Complete
    if iteration == total: 
        print()

# Numpy Utils
def is_all_zero(arr):
    return not np.any(arr)

def any_nan(arr):
    return np.isnan(arr).any()