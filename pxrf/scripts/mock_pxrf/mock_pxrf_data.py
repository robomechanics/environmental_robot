#!/usr/bin/env python3

#Generates fake PXRF data for use in simulation and testing purposes
#Uploads this data into mock_chemistry.csv for general use
#Incorporated into a rosservice and that can be launched and run 
#Mimics the data in chemistry.csv

import csv
import random
import datetime

longitude = -101.992708
latitude = 31.845601

class Mock_pxrf:
    mock_scans = []
    def __init__(self, longitude, latitude):
        self.longitude = longitude
        self.latitude = latitude
        self.current_mock_scans_made = 0
        self.mock_pxrf_data_filename = 'mock_chemistry.csv'
        self.concentrations = []
        self.scan = []

    pass

#gets the fake concentrations of various elements randomly
#random mock concentration parsing is roughly based on the ranges in mock_soil_concentration_ranges.txt
def get_mock_concentrations(filename):
    with open(filename, 'r') as file:
        mock_concentration_ranges = file.read()

    concentrations = [] 
    for line in mock_concentration_ranges.splitlines(): 
        words = line.split()
        if words[1] != 'Trace': #if the element exists in a typical soil sample
            _, lower_range, upper_range = words[0][:-1], words[1][:-1], words[3][:-1]
            #element concentration is based on a range between the values
            element_concentration = random.uniform(float(lower_range), float(upper_range))
        else:
            element_concentration = 0.0
        concentrations.append(element_concentration)

    #normalize elements to all sum to 100
    total = sum(concentrations)
    concentrations = [((concentrations[i]/total)*100) for i in range(len(concentrations))]
    return concentrations


#helper function to increase readability, expects a .txt file with a semicolon
#separating the string 'total_mock_scans_made' with the actual amount of mock scans made
#returns an int containing the total amount of mock scans made
def read_and_update_total_mock_scans_made(filepath):
    with open(filepath, 'r') as file: 
        total_mock_scans = file.read()
    total_mock_scans = int(total_mock_scans.split(':')[1])
    total_mock_scans += 1
    with open(filepath, 'w') as file: #rewrite to the file
        file.write(f'total_mock_scans_made:{total_mock_scans}')
    return total_mock_scans

def generate_element_distributions():
    measured_elements = ['Mg','Al','Si','P','S','Cl','Ca','Ti','V','Cr','Mn',
                         'Fe','Co','Ni','Cu','Zn','As','Se','Rb','Sr','Y','Zr',
                         'Nb','Mo','Ag','Cd','Sn','Sb','Ba','La','Ce','Pr','Nd',
                         'W','Hg','Pb','Bi','Th','U','LE']
    file_path = 'C:/Users/redkr/Desktop/environmental_robot/pxrf/scripts/mock_pxrf_data/mock_scans_made.txt'
    mock_scan = Mock_pxrf(longitude, latitude)
    mock_scan.current_mock_scans_made += 1
    total_mock_scans_made = read_and_update_total_mock_scans_made('total_mock_scans_made.txt')

    

    #add first line to scan [scans made in the current session, total scans made, 
    #date time,  longitude of scan latitude of scan]
    mock_scan.scan.append([mock_scan.current_mock_scans_made, 
                           total_mock_scans_made, 
                           f'{datetime.date.today()} {datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]}',
                           f'Lon: {mock_scan.longitude} Lat: {mock_scan.longitude}'])
    #add second line [list of measured elements]
    mock_scan.scan.append(measured_elements)
    #add third line [list of element concentrations] choosing of values could be improved
    mock_scan.scan.append(get_mock_concentrations('mock_soil_concentration_ranges.txt'))
    #add fourth line [list of predicted element error range] currently modelled after a basic exponential distribution
    #could be improved
    error_average = 1 / 10**3
    mock_scan.scan.append([random.expovariate(1/error_average) for i in range(len(measured_elements))])

    #write the scan to our mock_csv file
    with open(mock_scan.mock_pxrf_data_filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(mock_scan.scan)


if __name__ == '__main__':
    generate_element_distributions()
