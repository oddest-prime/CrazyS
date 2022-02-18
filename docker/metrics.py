#!/usr/bin/python3

import csv
from os.path import exists
import numpy as np

SWARM_DISABLED=            0
SWARM_DECLARATIVE_SIMPLE=  1
SWARM_REYNOLDS=            2
SWARM_REYNOLDS_LIMITED=    4
SWARM_REYNOLDS_VELOCITY=   8
SWARM_GRADIENT=            16
SWARM_GRADIENT_ENUM=       32
SWARM_PHASE_ESTABLISHED=   64
SWARM_LANDING=             32768

MIN_dist_min=99999
MAX_dist_center=0
MIN_obstacle_dist_min=99999

points_per_second = 200 # number of datapoints per second, for plotting

ARRAY_time = []
ARRAY_dist_min = []
ARRAY_dist_center = []
ARRAY_obstacle_dist_min = []

for i in range(0, 50):
    filename = "Metrics{}.csv".format(i)
    if exists(filename):
        print('filename ', filename)
        with open(filename, newline='') as csvfile:
            csv_data = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in csv_data:
                #print(', '.join(row))
                timeStampSec, timeStampNsec, enable_swarm, dist_min, dist_center, obstacle_dist_min, droneCount, neighbourhood_cnt, _ = row;
                timeStamp=float(timeStampSec)+float(timeStampNsec)/1000000000.0
                dist_min = float(dist_min)
                dist_center = float(dist_center)
                obstacle_dist_min = float(obstacle_dist_min)
                index = int(timeStamp*points_per_second)
                
                if int(enable_swarm) & SWARM_PHASE_ESTABLISHED: # metrics are only relevant after swarm is fully established
                
                    while len(ARRAY_time) < index+1:
                        ARRAY_time.append(0)
                    ARRAY_time[index] = timeStamp
                    while len(ARRAY_dist_min) < index+1:
                        ARRAY_dist_min.append(99999)
                    ARRAY_dist_min[index] = min(ARRAY_dist_min[index], dist_min)
                    while len(ARRAY_dist_center) < index+1:
                        ARRAY_dist_center.append(0)
                    ARRAY_dist_center[index] = max(ARRAY_dist_center[index], dist_center)
                    while len(ARRAY_obstacle_dist_min) < index+1:
                        ARRAY_obstacle_dist_min.append(99999)
                    ARRAY_obstacle_dist_min[index] = min(ARRAY_obstacle_dist_min[index], obstacle_dist_min)
                    
                    if dist_min < MIN_dist_min:
                        MIN_dist_min = dist_min
                    if dist_center > MAX_dist_center:
                        MAX_dist_center = dist_center
                    if obstacle_dist_min < MIN_obstacle_dist_min:
                        MIN_obstacle_dist_min = obstacle_dist_min
                
print("MIN_dist_min: ", MIN_dist_min)
print("MAX_dist_center: ", MAX_dist_center)
print("MIN_obstacle_dist_min: ", MIN_obstacle_dist_min)

f = open('CombinedMetrics.csv', 'w')
writer = csv.writer(f)
index = 0
while index < len(ARRAY_time):
    if ARRAY_time[index] > 0.1:
        writer.writerow([ARRAY_time[index], ARRAY_dist_min[index], ARRAY_dist_center[index], ARRAY_obstacle_dist_min[index]])
    index = index + 1
f.close()

