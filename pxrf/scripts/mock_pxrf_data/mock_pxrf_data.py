#!/usr/bin/env python3
#Generates fake PXRF data for use in simulation and testing purposes
#Uploads this data into mock_chemistry.csv for general use
#Incorporated into a rosservice and that can be launched and run 
#Mimics the following data
#2,2452,2022-06-07 10:04:44.275,Lon: -101.992708 Lat: 31.845601 (samples made in this session, total samples made, date, time, longitude, latitude)
# Mg,Al,Si,P,S,Cl,Ca,Ti,V,Cr,Mn,Fe,Co,Ni,Cu,Zn,As,Se,Rb,Sr,Y,Zr,Nb,Mo,Ag,Cd,Sn,Sb,Ba,La,Ce,Pr,Nd,W,Hg,Pb,Bi,Th,U,LE (element names)
# 0.0,2.3517966270446777,10.530171394348145,0.05086073279380798,1.7672011852264404,7.827661037445068,6.603203773498535,0.2628879249095917,0.0,0.009499354287981987,0.03051569126546383,1.9894896745681763,0.0,0.0056869215331971645,0.002670843154191971,0.012234821915626526,0.0008006208809092641,0.0012089033843949437,0.006755197420716286,0.04971793293952942,0.002259962959215045,0.013003647327423096,0.0,0.0008309381664730608,0.0,0.0,0.0,0.0,0.07388132810592651,0.0,0.030733661726117134,0.040218520909547806,0.10111629217863083,0.0,0.0,0.0,0.0,0.0,0.0,68.235595703125
# 0.4875452220439911,0.05585402622818947,0.05077558010816574,0.004670963156968355,0.011038396507501602,0.03061031736433506,0.02376331016421318,0.019384682178497314,0.009579327888786793,0.002657824894413352,0.002613348187878728,0.01208829041570425,0.004380448721349239,0.0005788506823591888,0.0005221538012847304,0.0005214585107751191,0.00016089610289782286,0.00013569628936238587,0.00016863287601154298,0.00038874344318173826,0.00017140920681413263,0.0002559410349931568,0.00033045231248252094,0.00022671688930131495,0.0,0.0015391879715025425,0.0019484689692035317,0.0026624519377946854,0.0035139594692736864,0.15329799056053162,0.0063327099196612835,0.0117589570581913,0.016991663724184036,0.001160125364549458,0.0007296575349755585,0.00032950897002592683,0.0015301114181056619,0.000754082459025085,0.0005102034192532301,0.07385095208883286

import csv
import random

class Mock_Pxrf:
    pass

current_mock_scans_made = 0
# Specify the path to your CSV file
file_path = 'C:/Users/redkr/Desktop/environmental_robot/pxrf/scripts/chemistry.csv'

# Open the file and create a CSV reader object
with open(file_path, mode='r', newline='') as file:
    csv_reader = csv.reader(file)
    
    # Read the header (first row)
    header = next(csv_reader)
    print(f'Header: {header}')
    
    # Read the rest of the rows
    for row in csv_reader:
        #print(row)
        try:
            row = [float(row[i]) for i in range(len(row))]
            print(sum(row), len(row))
        except:
            print('Stasr')

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
    mock_chemistry_data = list()
    measured_elements =    ['Mg','Al','Si','P','S','Cl','Ca','Ti','V','Cr','Mn',
                           'Fe','Co','Ni','Cu','Zn','As','Se','Rb','Sr','Y','Zr',
                           'Nb','Mo','Ag','Cd','Sn','Sb','Ba','La','Ce','Pr','Nd',
                           'W','Hg','Pb','Bi','Th','U','LE']
    file_path = 'C:/Users/redkr/Desktop/environmental_robot/pxrf/scripts/mock_pxrf_data/mock_scans_made.txt'
    current_mock_scans_made += 1
    total_mock_scans = read_and_update_total_mock_scans_made('total_mock_scans_made.txt')

    

    
    mock_chemistry_data.append([current_mock_scans_made, total_mock_scans_made, 'longitude', 'latitude'])
    mock_chemistry_data.append(measured_elements)

    with open('mock_soil_concentration_ranges.txt', 'r') as file:
        mock_concentration_ranges = file.read()

    for line in mock_concentration_ranges.splitlines():
        words = line.split()
        if words[1] != 'Trace':
            element, lower_range, upper_range = line[0][:-1], line[1][:-1], line[3][:-1]
            #element concentration is based on a range between the values yippee
        else:
            element_concentration = 0.0

            pass
            #much less happens since that element hardly exists
        




    print(len(measured_elements))
print(sum([0.0,1.1739778518676758,5.72076940536499,0.0,0.5838117003440857,13.76088809967041,2.8384900093078613,0.13247539103031158,0.0,0.021830417215824127,0.013545817695558071,0.9414430856704712,0.0,0.002256013685837388,0.0,0.06453360617160797,0.0,0.0005830583395436406,0.003852271940559149,0.03676415607333183,0.0016546008409932256,0.0121618015691638,0.0,0.0013790978118777275,0.0,0.0,0.0,0.0,0.12551197409629822,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.004186565987765789,0.0,0.0,74.55988311767578]))
print(sum([0.4875452220439911,0.05585402622818947,0.05077558010816574,0.004670963156968355,0.011038396507501602,0.03061031736433506,0.02376331016421318,0.019384682178497314,0.009579327888786793,0.002657824894413352,0.002613348187878728,0.01208829041570425,0.004380448721349239,0.0005788506823591888,0.0005221538012847304,0.0005214585107751191,0.00016089610289782286,0.00013569628936238587,0.00016863287601154298,0.00038874344318173826,0.00017140920681413263,0.0002559410349931568,0.00033045231248252094,0.00022671688930131495,0.0,0.0015391879715025425,0.0019484689692035317,0.0026624519377946854,0.0035139594692736864,0.15329799056053162,0.0063327099196612835,0.0117589570581913,0.016991663724184036,0.001160125364549458,0.0007296575349755585,0.00032950897002592683,0.0015301114181056619,0.000754082459025085,0.0005102034192532301,0.07385095208883286]))
generate_element_distributions()