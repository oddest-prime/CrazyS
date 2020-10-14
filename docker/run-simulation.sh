#!/bin/bash

Xvfb -shmem -screen 0 1280x1024x24 &
export DISPLAY=:0

cd /crazyflie_ws
source /opt/ros/melodic/setup.bash
source /crazyflie_ws/devel/setup.bash

catkin build

roslaunch rotors_gazebo crazyflie2_hovering_swarm.launch gui:=false

kill `pidof Xvfb`

/crazyflie_ws/src/crazys/docker/generate-video.sh
