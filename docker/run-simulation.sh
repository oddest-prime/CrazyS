#!/bin/bash

cd /crazyflie_ws
source /opt/ros/melodic/setup.bash
source /crazyflie_ws/devel/setup.bash

catkin build

roslaunch rotors_gazebo crazyflie2_hovering_swarm.launch gui:=false

/crazyflie_ws/src/crazys/docker/generate-video.sh
