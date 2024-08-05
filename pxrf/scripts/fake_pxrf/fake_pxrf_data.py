#!/usr/bin/env python3


#TO-DO change the sessionID to dailyID

#Generates fake PXRF data for use in simulation and testing purposes
#Uploads this data into fake_chemistry.csv for general use
#Incorporated into a rosservice and that can be launched and run 
#Mimics the data in chemistry.csv
import csv
import time
import rospy
import statistics
from pxrf.msg import PxrfMsg
from pxrf.srv import fake_pxrf_response #this line might be off
import sys

# add pxrf's plot script to lookup path
import os
import rospkg
rospack = rospkg.RosPack()
pxrf_path = rospack.get_path('pxrf')
sys.path.insert(0, os.path.abspath(os.path.join(pxrf_path, "scripts")))

import random
import datetime

longitude = -101.992708
latitude = 31.845601
    
class Fake_pxrf:
    fakeScans = []
    def __init__(self, longitude, latitude):
        self.longitude = longitude
        self.latitude = latitude
        self.fakeScansMadeInSesssion = 0
        self.fileContainingFakeData = 'fake_chemistry.csv'
        self.concentrations = []
        self.scan = []
        rospy.init_node('fake_pxrf_data', anonymous=True)
        rospy.Service('/generate_fake_pxrf_data', fake_pxrf_response, self.generateElementDistributions)
        rospy.spin()
    
    def generate_fake_pxrf_data(self):
        measuredElements = ['Mg','Al','Si','P','S','Cl','Ca','Ti','V','Cr','Mn',
                            'Fe','Co','Ni','Cu','Zn','As','Se','Rb','Sr','Y','Zr',
                            'Nb','Mo','Ag','Cd','Sn','Sb','Ba','La','Ce','Pr','Nd',
                            'W','Hg','Pb','Bi','Th','U','LE']

        self.numFakeScansInSesssion += 1
        numFakeScans = getAndUpdateNumFakeScans('numFakeScans.txt')

        self.scan.append([self.fakeScansMadeInSesssion, 
                            numFakeScans, 
                            f'{datetime.date.today()} {datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]}',
                            f'Lon: {self.longitude} Lat: {self.longitude}'])

        self.scan.append(measuredElements)

        self.scan.append(generateFakeConcentrations('fakeSoilConcentrationRanges.txt'))

        #add fourth line [list of predicted element error range] currently modelled after a basic exponential distribution
        #could be improved
        averageError = 1 / 10**3
        self.scan.append([random.expovariate(1/averageError) for i in range(len(measuredElements))])
        self.fakeScans.append(self.scan)

        with open(self.fileContainingFakeData, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(self.scan)

#line contains a string with the lower and upper range of values
def generateIndividualFakeConcentration(line):
    words = line.split()
    if words[1] == 'Trace':
        return 0.0
   
    _, lowerRange, upperRange = words[0][:-1], words[1][:-1], words[3][:-1]
    return random.uniform(float(lowerRange), float(upperRange))

#random fake concentration parsing is roughly based on the ranges in fake_soil_concentration_ranges.txt
def generateFakeConcentrations(filename):
    with open(filename, 'r') as file:
        fakeConcentrationRanges = file.read()

    concentrations = [] 
    for line in fakeConcentrationRanges.splitlines(): 
        elementConcentration = generateIndividualFakeConcentration(line)
        concentrations.append(elementConcentration)

    total = sum(concentrations)
    concentrations = [((concentrations[i]/total)*100) for i in range(len(concentrations))]
    return concentrations

def getAndUpdateNumFakeScans(filepath):
    with open(filepath, 'r') as file: 
        totalFakeScans = file.read()
    totalFakeScans = int(totalFakeScans.split(':')[1])
    totalFakeScans += 1
    with open(filepath, 'w') as file: #rewrite to the file
        file.write(f'numFakeSScandMade:{totalFakeScans}')
    return totalFakeScans


if __name__ == '__main__':
    Fake_pxrf(Fake_pxrf(longitude, latitude))
