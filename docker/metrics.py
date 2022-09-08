#!/usr/bin/python3

import csv
from os.path import exists
import numpy as np
import argparse
import pathlib
import os
import statistics

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
MAX_dist_target=0

points_per_second = 100 # number of datapoints per second, for plotting

ARRAY_time = []
ARRAY_dist_min = []
ARRAY_dist_center = []
ARRAY_obstacle_dist_min = []
ARRAY_dist_target = []

first_index = -1

def aggregate_logs(dirname = "."):
    global SWARM_PHASE_ESTABLISHED

    global MIN_dist_min
    global MAX_dist_center
    global MIN_obstacle_dist_min
    global MAX_dist_target

    global points_per_second
    global first_index

    global ARRAY_time
    global ARRAY_dist_min
    global ARRAY_dist_center
    global ARRAY_obstacle_dist_min
    global ARRAY_dist_target

    for i in range(0, 50):
        filename = "{}/Metrics{}.csv".format(dirname, i)
        if exists(filename):
            print('filename=', filename)
            with open(filename, newline='') as csvfile:
                csv_data = csv.reader(csvfile, delimiter=',', quotechar='|')
                for row in csv_data:
                    #print(', '.join(row))
                    timeStampSec, timeStampNsec, enable_swarm, dist_min, dist_center, obstacle_dist_min, dist_target, droneCount, neighbourhood_cnt, _ = row;
                    timeStamp=float(timeStampSec)+float(timeStampNsec)/1000000000.0
                    dist_min = float(dist_min)
                    dist_center = float(dist_center)
                    obstacle_dist_min = float(obstacle_dist_min)
                    dist_target = float(dist_target)
                    if first_index < 0:
                        first_index = int(timeStamp*points_per_second)
                    index = int(timeStamp*points_per_second) - first_index

                    if int(enable_swarm) & SWARM_GRADIENT_ENUM: # metrics are only relevant after swarm is fully established

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
                        while len(ARRAY_dist_target) < index+1:
                            ARRAY_dist_target.append(0)
                        ARRAY_dist_target[index] = max(ARRAY_dist_target[index], dist_target)

                        if dist_min < MIN_dist_min:
                            MIN_dist_min = dist_min
                        if dist_center > MAX_dist_center:
                            MAX_dist_center = dist_center
                        if obstacle_dist_min < MIN_obstacle_dist_min:
                            MIN_obstacle_dist_min = obstacle_dist_min
                        if dist_center > MAX_dist_target:
                            MAX_dist_target = dist_target

def aggregate_write_out(dirname = ".", filename_prefix = "", filename_suffix = ""):
    print("MIN_dist_min: ", MIN_dist_min)
    print("MAX_dist_center: ", MAX_dist_center)
    print("MIN_obstacle_dist_min: ", MIN_obstacle_dist_min)
    print("MAX_dist_target: ", MAX_dist_target)

    filename_out = "{}/{}CombinedMetrics{}.csv".format(dirname, filename_prefix, filename_suffix)
    print('filename_out=', filename_out)
    f = open(filename_out, 'w')
    writer = csv.writer(f)
    index = 0
    while index < len(ARRAY_time):
        if ARRAY_time[index] > 8:
#            writer.writerow([ARRAY_time[index], min([ARRAY_dist_min[index-1], ARRAY_dist_min[index]]), max([ARRAY_dist_center[index-1], ARRAY_dist_center[index]]), min(ARRAY_obstacle_dist_min[index-1], ARRAY_obstacle_dist_min[index]), ARRAY_dist_target[index]])
#            writer.writerow([ARRAY_time[index], ARRAY_dist_min[index], ARRAY_dist_center[index], ARRAY_obstacle_dist_min[index], ARRAY_dist_target[index]])
            writer.writerow([ARRAY_time[index], statistics.median([ARRAY_dist_min[index-2], ARRAY_dist_min[index-1], ARRAY_dist_min[index]]), statistics.median([ARRAY_dist_center[index-4], ARRAY_dist_center[index-3], ARRAY_dist_center[index-2], ARRAY_dist_center[index-1], ARRAY_dist_center[index]]), statistics.median([ARRAY_obstacle_dist_min[index-2], ARRAY_obstacle_dist_min[index-1], ARRAY_obstacle_dist_min[index]]), ARRAY_dist_target[index]])

        index = index + 1
    f.close()



parser = argparse.ArgumentParser(description='Aggregate logs for plotting.')
parser.add_argument('path', metavar='path', nargs='*', help='path filters') # pathlib.Path

args = parser.parse_args()
if args.path == []:
    print("no path specified, using current directory.")
    aggregate_logs()
    aggregate_write_out()
else:
    print("path: ", args.path)
    for i in os.listdir(".."):
        if os.path.isdir(os.path.join("..",i)):
            matching = True
            for matcher in args.path:
                if i.find(matcher) == -1:
                    matching = False

            if matching:
                print("matching directory:", i, args.path);
                aggregate_logs("../{}".format(i))
                aggregate_write_out("..", "data_", "_filtered-by_{}".format('_'.join(args.path)))
            else:
                print("other directory:", i, args.path);
exit(0)
