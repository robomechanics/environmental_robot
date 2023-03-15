#!/usr/bin/env python3
import matplotlib.pyplot as plt
import csv
import re
import numpy as np
header = []
element = []
concentration = []
error = []

# add pxrf's plot script to lookup path
import os
import rospkg
rospack = rospkg.RosPack()
pxrf_path = rospack.get_path('pxrf')

def generate_plot():
#setable parameters
	threshold = 0.01
	number_of_elements_display_limit = 10

	with open(os.path.join(pxrf_path, '/scripts/chemistry.csv'),'r') as csvfile:
		data = list(csv.reader(csvfile, delimiter = ','))
		for row in range(len(data)):
		    if(len(data[row])>2):
			    if (re.search("^\d{4}-(0[1-9]|1[0-2])-(0[1-9]|[12][0-9]|3[01])", data[row][2])):
				    header.append(data[row][2] + " testID:" + data[row][1])
				    element.append(data[row+1])
				    concentration.append(data[row+2])
				    error.append(data[row+3])
		csvfile.close()

	total_num = len(header)
	last = total_num - 1
	#while (len(element[last]) >= number_of_elements_display_limit):
		#threshold = threshold / 2.0

	for i in range(len(element[last])):
		if(len(element[last]) <= number_of_elements_display_limit):
			break
		#print((np.array(concentration[last]).astype(float))[i])
		#print((np.array(concentration[last]).astype(float))[i] <= threshold)
		if ((np.array(concentration[last]).astype(float))[i] <= threshold):
			element[last][i]= None
			concentration[last][i] = None
			error[last][i] = None
	element[last]= list(filter(None,element[last]))
	concentration[last] = list(filter(None,concentration[last]))
	error[last] = list(filter(None,error[last]))
	fig, ax = plt.subplots(figsize=(16,10))
	theme = plt.get_cmap('hsv')
	ax.set_prop_cycle("color",[theme(1. * i/len(element[last])) for i in range(len(element[last]))])
	patches, texts = plt.pie(np.array(concentration[last]),labels = element[last], startangle = 90, radius = 1.1)
	labels = ['{} - {:.2f} +/- {:.3f}'.format(i,j,k) for i,j,k in zip (np.char.array(element[last]),np.array(concentration[last]).astype(float),np.array(error[last]).astype(float))]
	#sort based on concentration
	patches, labels, _ = zip(*sorted(zip(patches, labels, np.array(concentration[last])), key = lambda x: x[2],reverse = True))
	plt.title(header[last])
	fig.canvas.set_window_title(header[last])
	plt.legend(patches, labels, loc = 'best', bbox_to_anchor = (-0.1, 1.), fontsize = 10)
	plt.tight_layout()
	plt.show()
