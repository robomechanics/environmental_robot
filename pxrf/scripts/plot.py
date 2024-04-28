#!/usr/bin/env python3
import matplotlib.pyplot as plt
import csv
import re
import numpy as np
header = []
element = []
concentration = []
error = []

# Add pxrf's plot script to lookup path
import os
import rospkg
import argparse

rospack = rospkg.RosPack()
pxrf_path = rospack.get_path('pxrf')
print(pxrf_path)

def generate_plot(file_path, threshold = 0.01, number_of_elements_display_limit = 10, result_ctr = -1):
	with open(file_path,'r') as csvfile:
		data = list(csv.reader(csvfile, delimiter = ','))
		for row in range(len(data)):
			if(len(data[row])>2):
				if (re.search("^\d{4}-(0[1-9]|1[0-2])-(0[1-9]|[12][0-9]|3[01])", data[row][2])):
					header.append("Date: " + data[row][2] + " | Test ID:" + data[row][1])
					element.append(data[row+1])
					concentration.append(data[row+2])
					error.append(data[row+3])
		csvfile.close()
	
	if result_ctr < 0:
		result_ctr = max(0, len(header) + result_ctr)

	if result_ctr > (len(header) - 1):
		result_ctr = len(header) - 1

	print(f"Displaying PXRF Result: {result_ctr}/{len(header)} from file: {result_ctr}")
 
	#while (len(element[result_ctr]) >= number_of_elements_display_limit):
		#threshold = threshold / 2.0

	for i in range(len(element[result_ctr])):
		if(len(element[result_ctr]) <= number_of_elements_display_limit):
			break
		#print((np.array(concentration[result_ctr]).astype(float))[i])
		#print((np.array(concentration[result_ctr]).astype(float))[i] <= threshold)
		if ((np.array(concentration[result_ctr]).astype(float))[i] <= threshold):
			element[result_ctr][i]= None
			concentration[result_ctr][i] = None
			error[result_ctr][i] = None
   
	element[result_ctr]= list(filter(None,element[result_ctr]))
	concentration[result_ctr] = list(filter(None,concentration[result_ctr]))
	error[result_ctr] = list(filter(None,error[result_ctr]))
	fig, ax = plt.subplots(figsize=(16,10))
	theme = plt.get_cmap('hsv')
	ax.set_prop_cycle("color",[theme(1. * i/len(element[result_ctr])) for i in range(len(element[result_ctr]))])
	patches, texts = plt.pie(np.array(concentration[result_ctr]),labels = element[result_ctr], startangle = 90, radius = 1.1)
	labels = ['{} - {:.2f} +/- {:.3f}'.format(i,j,k) for i,j,k in zip (np.char.array(element[result_ctr]),np.array(concentration[result_ctr]).astype(float),np.array(error[result_ctr]).astype(float))]
	
 	# Sort based on concentration
	patches, labels, _ = zip(*sorted(zip(patches, labels, np.array(concentration[result_ctr])), key = lambda x: x[2],reverse = True))
	plt.title(header[result_ctr])
	try:
		fig.canvas.set_window_title(header[result_ctr])
	except:
		fig.canvas.setWindowTitle(header[result_ctr])	
	plt.legend(patches, labels, loc = 'best', bbox_to_anchor = (-0.1, 1.), fontsize = 10)
	plt.tight_layout()
	plt.show()

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Plot Elemental Concentration from CSV')
	parser.add_argument('-f', '--file', type=str, default=pxrf_path + '/scripts/chemistry.csv')
	parser.add_argument('-t', '--threshold', type=int, default=0.01)
	parser.add_argument('-d', '--display', type=int, default=10)
	parser.add_argument('-i', '--id', type=int, default=-1)
	args = parser.parse_args()

	generate_plot(file_path=args.file, 
                  threshold = args.threshold, 
                  number_of_elements_display_limit = args.display, 
                  result_ctr = args.id)