#!/bin/bash

Xvfb -shmem -screen 0 1280x1024x24 &
export DISPLAY=:0

cd /crazyflie_ws
source /opt/ros/melodic/setup.bash
source /crazyflie_ws/devel/setup.bash

catkin build

TIMEOUT=300

roslaunch rotors_gazebo crazyflie2_hovering_swarm.launch gui:=false &

sleep 1
iter=1
R_PID=`pgrep roslaunch`
for i in `seq 1 $TIMEOUT`
do
    kill -0 $R_PID 2> /dev/null || break
    echo "  [[ global sim timeout counter: $iter / $TIMEOUT ]]  "
    iter=$((iter+1))
    sleep 1
done;
echo $iter > /tmp/run.seconds
echo ""
echo "##  killing roslaunch";
kill $R_PID 2> /dev/null
sleep 5

kill `pidof Xvfb`

/crazyflie_ws/src/crazys/docker/generate-video.sh
