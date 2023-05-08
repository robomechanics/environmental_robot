import numpy as np
import scipy as sp
import scipy.ndimage
import matplotlib.pyplot as plt
import time
from matplotlib.colors import LogNorm

def generateRandomDistribution(map_size, density=0.1, heterogeneity=0.5, random_seed=''):

	# Define algorithm parameters
	edge_mask = sp.ndimage.binary_erosion(np.ones(map_size)).astype(int)
	density_tol = 0.02
	heterogeneity_factor = 4

	# Declare seed for randomization, if not specified use a random seed
	if not random_seed:
		np.random.seed()
	else:
		np.random.seed(random_seed)

	# Generate an appropriately sized array of random values [0,1]
	env_map_seed = np.random.rand(map_size[0], map_size[1])

	# Transform map into log space
	"""
	what is the reason for doing this?
	"""
	heterogeneity = np.power(heterogeneity,heterogeneity_factor)
	env_map_seed = np.power(0.1,env_map_seed/(heterogeneity+1e-3))

	# Initialize thresholds on variance and density
	current_density = -1
	min_variance = 0
	max_variance = max(map_size)
	min_threshold = 0
	max_threshold = 1

	# Iterate until the density is within a tolerance of the target
	while (abs(current_density-density) > density_tol):

		# Select variance and threshold as average of the bounds
		variance = np.average((min_variance,max_variance))
		threshold = np.average((min_threshold,max_threshold)) # 0.5

		# Apply a Gaussian filter to the map to smooth things out
		env_map = sp.ndimage.filters.gaussian_filter(env_map_seed, [variance,variance], mode='constant')

		# Normalize so that all values are [0,1]
		if (np.max(env_map)>0):
			env_map = env_map/np.max(env_map)

		# Apply a threshold to get a binary map and calculate the density
		env_map_binary = (env_map>=threshold)
		env_map_binary = env_map_binary & edge_mask
		env_map_binary = sp.ndimage.binary_fill_holes(env_map_binary).astype(int)
		current_density = env_map_binary.sum()/edge_mask.sum()

		# Tighten variance parameter towards target density
		if (current_density < density):
			min_variance = variance
		else:
			max_variance = variance

		# If variance converged incorrectly, adjust thresholding bounds and try again
		if abs(max_variance-min_variance) < 1e-3:
			min_variance = 0
			max_variance = max(map_size)
			if current_density > density:
				min_threshold = threshold
			else:
				max_threshold = threshold

		# If thresholding converged incorrectly, reset bounds, generate new map and try again
		if abs(max_threshold-min_threshold) < 1e-2:
			print('threshold converged incorrectly, try again')
			min_threshold = 0
			max_threshold = 1

			np.random.seed()

			# Generate an appropriately sized array of random values [0,1]
			env_map_seed = np.random.rand(map_size[0], map_size[1])

			# Transform map into log space
			heterogeneity = np.power(heterogeneity,heterogeneity_factor)
			env_map_seed = np.power(0.1,1*env_map_seed/(heterogeneity+1e-3))

	return env_map, env_map_binary

def generateRandomObstacles(map_size, density=0.5, heterogeneity=0.5,random_seed=''):

	# Define algorithm parameters
	edge_mask = sp.ndimage.binary_erosion(np.ones(map_size)).astype(int)
	density_tol = 0.02
	heterogeneity_factor = 4

	# Declare seed for randomization, if not specified use a random seed
	if not random_seed:
		np.random.seed()
	else:
		np.random.seed(random_seed)

	# Generate an appropriately sized array of random values [0,1]
	env_map_seed = np.random.rand(map_size[0], map_size[1])

	# plt.imshow(env_map_seed,cmap='jet', origin='lower', vmin=0, vmax=1)
	# plt.savefig("initial_seed.pdf")

	# Transform map into log space
	original_heterogeneity = heterogeneity
	heterogeneity = np.power(heterogeneity,heterogeneity_factor)
	env_map_seed = np.power(0.1,env_map_seed/(heterogeneity+1e-3))

	# plt.imshow(env_map_seed,cmap='jet', origin='lower', vmin=0, vmax=1)
	# plt.savefig("initial__hetero.pdf")

	# Initialize thresholds on variance and density
	current_density = -1
	min_variance = 0
	max_variance = max(map_size)
	min_threshold = 0
	max_threshold = 1

	# Iterate until the density is within a tolerance of the target
	while (abs(current_density-density) > density_tol):

		# Select variance and threshold as average of the bounds
		variance = np.average((min_variance,max_variance))
		threshold = np.average((min_threshold,max_threshold))

		# Apply a Gaussian filter to the map to smooth things out
		env_map = sp.ndimage.filters.gaussian_filter(env_map_seed, [variance,variance], mode='constant')

		# Normalize so that all values are [0,1]
		if (np.max(env_map)>0):
			env_map = env_map/np.max(env_map)

		# Apply a threshold to get a binary map and calculate the density
		env_map_binary = (env_map>=threshold)
		env_map_binary = env_map_binary & edge_mask
		env_map_binary = sp.ndimage.binary_fill_holes(env_map_binary).astype(int)
		current_density = env_map_binary.sum()/edge_mask.sum()

		# # Debug
		# print('')
		# print('current density = ' + str(current_density) + ', target = ' + str(density))
		# print('variance bounds = ' + str(min_variance) + ' < ' + str(variance) + ' < ' + str(max_variance) )
		# print('threshold bounds = ' + str(min_threshold) + ' < ' + str(threshold) + ' < ' + str(max_threshold) )

		# plt.imshow(env_map_binary,cmap='jet', origin='lower')
		# plt.colorbar()
		# plt.show()

		# Tighten variance parameter towards target density
		if (current_density < density):
			min_variance = variance
		else:
			max_variance = variance

		# If variance converged incorrectly, adjust thresholding bounds and try again
		if abs(max_variance-min_variance) < 1e-3:
			min_variance = 0
			max_variance = max(map_size)
			if current_density > density:
				min_threshold = threshold
			else:
				max_threshold = threshold

		# If thresholding converged incorrectly, reset bounds, generate new map and try again
		if abs(max_threshold-min_threshold) < 1e-2:
			print('threshold converged incorrectly, try again')
			min_threshold = 0
			max_threshold = 1

			np.random.seed()

			# Generate an appropriately sized array of random values [0,1]
			env_map_seed = np.random.rand(map_size[0], map_size[1])

			# Transform map into log space
			heterogeneity = np.power(original_heterogeneity,heterogeneity_factor)
			env_map_seed = np.power(0.1,1*env_map_seed/(heterogeneity+1e-3))


	# plt.imshow(env_map,cmap='jet', origin='lower', vmin=0, vmax=1)
	# plt.savefig("cont__hetero__density.pdf")

	# plt.imshow(env_map_binary,cmap='jet', origin='lower', vmin=0, vmax=1)
	# plt.savefig("binary__hetero__density.pdf")


	return env_map_binary