from copy import copy
import numpy as np
import scipy as sp
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.animation as animation
import rospy
from std_msgs.msg import Header
import cv2
import numpy as np
import io
from cv_bridge import CvBridge

def recreateDistribution(robot_path, distribution_map):

	# Set up the grid for interpolation
	n_rows = np.shape(distribution_map)[0]
	n_cols = np.shape(distribution_map)[1]
	grid_x, grid_y = np.mgrid[:n_rows, :n_cols]

	# Get the observations to use for interpolation
	values = distribution_map[robot_path[:,0], robot_path[:,1]]

	# Perform linear interpolation but fill in values outside convex hull with the nearest observation
	grid_z = griddata(robot_path, values, (grid_x, grid_y), method='linear', fill_value=np.nan)
	grid_z_nearest = griddata(robot_path, values, (grid_x, grid_y), method='nearest', fill_value=0)
	grid_z[np.isnan(grid_z)] = grid_z_nearest[np.isnan(grid_z)]

	return grid_z

def getStatistics(robot_path, sample_points, obstacle_map, distribution_map, hotspot_map):

	# Make sure everything has the right type
	robot_path = np.asarray(robot_path, np.dtype('int','int'))
	obstacle_map = np.asarray(obstacle_map, np.dtype('int','int'))
	distribution_map = np.asarray(distribution_map, np.dtype('float'))
	sample_points = np.array(sample_points)
	hotspot_map = np.asarray(hotspot_map, np.dtype('int','int'))

	# Generate the distribution recreation
	recreated_distribution_map = recreateDistribution(sample_points, distribution_map)

	# Compute the path length
	path_length = 0
	for i in range(np.shape(robot_path)[0]-1):
		path_length += np.sqrt(np.power(robot_path[i+1,0] - robot_path[i,0],2) + np.power(robot_path[i+1,1] - robot_path[i,1],2))

	# Compute the number of samples (checking for unique samples, ideally this shouldn't be necessary)
	sample_points = np.unique(sample_points, axis=0)
	num_samples = np.shape(sample_points)[0]

	# Compute masked versions of hotspot and distribution maps
	distribution_map_masked = distribution_map*(1-obstacle_map)
	recreated_distribution_map_masked = recreated_distribution_map*(1-obstacle_map)
	hotspot_map_masked = hotspot_map & (1-obstacle_map)

	# Compute the RSME of the recreation only over the unblocked area
	# rmse = np.sqrt(np.sum(np.power(recreated_distribution_map - distribution_map,2))/np.size(distribution_map)) # Leave obstacles as is
	rmse = np.sqrt(np.sum(np.power(recreated_distribution_map_masked - distribution_map_masked,2))/np.sum(1-obstacle_map)) # Mask obstacles

	# # Debugging - plot the masked maps
	# plt.imshow(hotspot_map, cmap='jet',origin='lower', vmin=0, vmax=1)
	# plt.subplot(121)
	# plotMap(obstacle_map, distribution_map)

	# plt.subplot(122)
	# plt.imshow(distribution_map_masked, cmap='jet',origin='lower', vmin=0, vmax=1)
	# plt.show()

	# Compute the number of hotspots and label them
	# labeled_hotspots, num_hotspots = sp.ndimage.label(hotspot_map) # Leave obstacles as is
	labelled_hotspots, num_hotspots = sp.ndimage.label(hotspot_map_masked) # Mask obstacles

	# Determine the number of found hotspots by iterating through samples and checking if they reside within hotspot boundaries
	if num_hotspots == 0:
		# This can happen if hotspots are fully masked by obstacles
		num_hotspots_detected = 0
		hotspot_detection_rate = 1
	else:
		found_hotspots = []
		for sample in sample_points:
			indicator = labelled_hotspots[sample[0],sample[1]]
			if (indicator>0) and (indicator not in found_hotspots):
				found_hotspots.append(indicator)

		num_hotspots_detected = len(found_hotspots)
		hotspot_detection_rate = num_hotspots_detected/num_hotspots

	return path_length, num_samples, rmse, hotspot_detection_rate, num_hotspots_detected, num_hotspots


def plotMap(obstacle_map, distribution_map, ax=None, normalize=False):

	# Create mask from obstacle map for colormap
	palette = copy(plt.cm.binary)
	palette.set_over('k',1.0)
	palette.set_bad(alpha=0.0)
	Zm = np.ma.masked_where(obstacle_map <=0.99, obstacle_map)

	# Define common formatting options
	origin_string = 'lower' #where the origin should go
	interp_method = 'nearest' # bilinear to look nice, nearest for clarity
	axis_range = (0,np.shape(obstacle_map)[1],0,np.shape(obstacle_map)[0])
	line_color = (0.733,0,0)

	# Create the figure object if not already created
	if ax is None:
		ax = plt.gca()
	plt.sca(ax)

	# Plot the underlying distribution and overlay the obstacle map
	if normalize:
		cm = plt.imshow(distribution_map,cmap='jet',aspect='equal', origin=origin_string,
			  interpolation=interp_method,vmin=distribution_map.min(),vmax=distribution_map.max())
	else:
		cm = plt.imshow(distribution_map,cmap='jet',aspect='equal', origin=origin_string,
			  interpolation=interp_method,vmin=0,vmax=1)
	#plt.plot(robot_path[:,0], robot_path[:,1], linewidth=2, color=line_color)
	plt.xlabel('x')
	plt.ylabel('y')

	# Include a mask for the obstacles
	plt.imshow(Zm,cmap=palette,aspect='equal', origin=origin_string,vmin=0,vmax=1)

	return cm

def visualizer(robot_path, distribution_map, mu, bin_entropy, x_bound, y_bound, fig=None,animate=False, record=False):
		# Recording videos requires a different backend for matplotlib
	if record:
		matplotlib.use("Agg")

	# Make sure objects are np-arrays
	robot_path = np.asarray(robot_path, np.dtype('int','int'))
	mu = np.asarray(mu, np.dtype('float'))
	bin_entropy = np.asarray(bin_entropy, np.dtype('float'))
	if distribution_map is not None:
		distribution_map = np.asarray(distribution_map, np.dtype('float'))

	sampled = np.asarray(robot_path, np.dtype('int','int'))

	# Create the figure object if not yet present and set the line color
	if fig is None:
		fig = plt.figure(figsize=(17,4))
	line_color = (0.733,0,0)
	ax1 = plt.subplot(131)
	# Plot the underlying distribution and overlay the obstacle map
	if distribution_map is not None:
		
		ax1.plot(sampled[:,1], sampled[:,0], '.', color=line_color)
		plotMap(distribution_map, distribution_map, plt.gca())
		plt.title('Original Distribution')

	# Plot the recreated distribution and overlay the obstacle map
	ax2 = plt.subplot(132)
	# ax2.plot(sampled[:,1], sampled[:,0], '.', color=line_color)
	plotMap(mu, mu, plt.gca())
	plt.title('Recreated Distribution')

	# Animate the path if desired
	if animate:
		# Initialize the line object
		line, = ax2.plot([],[], linewidth=2, color=line_color)

		# Define init and animation functions for animation
		def init():  # only required for blitting to give a clean slate.
		    line.set_data(sampled[:,1], sampled[:,0])
		    return line,

		def animate(i):
		    line.set_data(sampled[:,1],sampled[:,0])  # update the data.
		    return line,

		# Determine the correct interval between frames based on a desired animation duration
		anim_duration = 0.05 # seconds
		frame_rate = (sampled.shape[0]/anim_duration)
		interval = 1/frame_rate*1000 # must be in milliseconds

		# Create the animation object
		ani = animation.FuncAnimation(
		    fig, animate, init_func=init, interval=interval, blit=False, save_count=sampled.shape[0])
	else:
		ax1.plot(robot_path[:,1], robot_path[:,0], linewidth=2, color=line_color)

	#plot in red color
	plt.plot(x_bound, y_bound, linewidth=3, color=(0.733,0,0))
	ax3 = plt.subplot(133)
	ax3.plot(sampled[:,1], sampled[:,0], '.', color=line_color)
	plotMap(bin_entropy, bin_entropy, plt.gca())
	plt.plot(x_bound, y_bound, linewidth=3, color=(0.733,0,0))
	plt.title('Uncertainty')

	plt.show()

def plotPathResult(robot_path, obstacle_map, distribution_map, sample_points='', fig=None,animate=False, record=False):

	# Recording videos requires a different backend for matplotlib
	if record:
		matplotlib.use("Agg")

	# Make sure objects are np-arrays
	robot_path = np.asarray(robot_path, np.dtype('int','int'))
	obstacle_map = np.asarray(obstacle_map, np.dtype('float'))
	distribution_map = np.asarray(distribution_map, np.dtype('float'))

	# If sample_points aren't specified just use the points from robot_path
	if not sample_points:
		print('No sampling points specified, using robot path')
		sample_points = np.asarray(robot_path, np.dtype('int','int'))
	else:
		sample_points = np.asarray(sample_points, np.dtype('int','int'))

	# Calculate the recreated distribution map, either as zeros or from sampling along the path
	recreated_distribution_map = recreateDistribution(sample_points, distribution_map)

	# Create the figure object if not yet present and set the line color
	if fig is None:
		fig = plt.figure(figsize=(17,4))
	line_color = (0.733,0,0)

	# yellow
	dot_color = (1,1,0)

	# Plot the underlying distribution and overlay the obstacle map
	ax1 = plt.subplot(131)
	plotMap(obstacle_map,distribution_map, plt.gca())
	plt.title('Original Distribution')

	# Plot the robot path and sample points
	# plt.plot(robot_path[:,1], robot_path[:,0], linewidth=2, color=line_color)
	#plt.plot(sample_points[:,1], sample_points[:,0], '.', color=line_color)


	# Plot the recreated distribution and overlay the obstacle map
	ax2 = plt.subplot(132)
	ax2.plot(sample_points[:,1], sample_points[:,0], '.', color=line_color)
	print(recreated_distribution_map)
	im = plotMap(obstacle_map,recreated_distribution_map, plt.gca())
	plt.title('Recreated Distribution')

	# Animate the path if desired
	if animate:
		# Initialize the line object
		line, = ax2.plot([],[], linewidth=2, color=line_color)

		# Define init and animation functions for animation
		def init():  # only required for blitting t[o give a clean slate.
			line.set_data(robot_path[:,1], robot_path[:,0])
			return line,

		def animate(i):
			line.set_data(robot_path[0:i,1],robot_path[0:i,0])  # update the data.
			return line,

		# Determine the correct interval between frames based on a desired animation duration
		anim_duration = 5 # seconds
		frame_rate = (robot_path.shape[0]/anim_duration)
		interval = 1/frame_rate*1000 # must be in milliseconds

		# Create the animation object
		ani = animation.FuncAnimation(
			fig, animate, init_func=init, interval=interval, blit=True, save_count=robot_path.shape[0])

		# If specified, record the map animation
		if record:
			writer = animation.FFMpegWriter(
			    fps=frame_rate, metadata=dict(artist='Me'), bitrate=1800)
			ani.save("movie.mp4", writer=writer)

	# Otherwise just show all the points in the path
	else:
		plt.plot(robot_path[:,1], robot_path[:,0], linewidth=2, color=line_color)
	
	ax3 = plt.subplot(133)
	ax3.plot(sample_points[:,1], sample_points[:,0], '.', color=dot_color)
	plotMap(obstacle_map,np.abs(recreated_distribution_map-distribution_map,normalize=True), plt.gca())
	plt.colorbar(im)
	plt.title('Error')

	# Show the plot
	if not record:
		plt.show()


def mu_to_img_msg(mu):
	# Plot the mu
	fig, ax = plt.subplots()  # Creates a single Axes
	plotMap(mu, mu, ax)

    # Remove axes, ticks, and labels
	ax.axis('off')  # Turn off the axis
	plt.subplots_adjust(left=0, right=1, top=1, bottom=0)  # Adjust the layout to minimize whitespace
	plt.margins(0, 0)  # Turn off margins

	# Save the figure to an in-memory buffer
	buf = io.BytesIO()
	plt.savefig(buf, format='png', bbox_inches='tight', pad_inches=0)
	plt.close(fig)  # Close the figure to avoid displaying it
	buf.seek(0)  # Rewind the buffer

	# Read the image from the buffer with OpenCV
	cv_image = cv2.imdecode(np.frombuffer(buf.getvalue(), np.uint8), cv2.IMREAD_COLOR)

	height, width = cv_image.shape[:2]

	# Convert color from BGR to RGB (OpenCV uses BGR by default)
	# cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

	# Convert to ROS Image message
	cv_bridge = CvBridge()
	ros_image = cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
	# Assign header with frame_id
	ros_image.header = Header()
	ros_image.header.stamp = rospy.Time.now()

	return ros_image, (width, height)