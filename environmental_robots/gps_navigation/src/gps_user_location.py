#!/usr/bin/python3
import sys
import os
import re
import csv

# add pxrf's plot script to lookup path
import rospkg
rospack = rospkg.RosPack()
work_dir_path = os.path.join(rospack.get_path('gps_navigation'), 'src')
sys.path.insert(0, work_dir_path)

def read_location():
    print("please ensure that the robot is connected to the wifi")
    data = []
    num = 0
    data_file = os.path.join(work_dir_path, 'locations.csv')
    with open(data_file, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter = ',')
        for row in reader:
            if row:
                num += 1
                data.append(row)
        print("The current map is " + data[-1][0] + "\n")
    while(True):
        select1 = input(" \n Select the following options: \n 1.change map \n 2.new map \n 3.continue \n")
    
        if select1 == '1':
            print("Select from the following map: \n")
            for i in range(len(data)):
                print(str(i+1) + ". " + data[i][0])
            select2 = input()
            if not (int(select2) >= 1 and int(select2) <= num):
                print("invalid number")
            else:
                return data[int(select2) - 1]
        elif select1 == '2':
            filename = input("enter the map name: ")
            lat = input("enter the latitude: ")
            lon = input("enter the longitude: ")
            zoom = input("enter the zoom - recommend 20: ")
            width = input("enter the width - recommend 10: ")
            height = input("enter the height - recommend 10: ")
            new_location = [filename + "," + lat + "," + lon + "," + zoom + "," + width + "," + height]
            with open(data_file, 'a') as writefile:
                writer = csv.writer(writefile)
                writer.writerow([filename, lat, lon, zoom, width, height])
            print("new location added successfully")
            return [filename, lat, lon, zoom, width, height]
        elif select1 == '3':
            print("launching the application")
            return data[-1]
