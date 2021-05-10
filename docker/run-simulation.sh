#!/bin/bash

TIMEOUT=2500

Xvfb -shmem -screen 0 1280x1024x24 &
export DISPLAY=:0

hash="uu"
if [ $# -ge 1 ]
then
  hash="$1"
fi;
date_hash="`date +%Y-%m-%d_%H-%M-%S`_${hash}"
echo "========================================================="
echo "git hash: ${hash}"
echo "date and hash: ${date_hash}"
echo "========================================================="

mkdir -p /crazyflie_ws/src/crazys/log_output

cd /crazyflie_ws
source /opt/ros/melodic/setup.bash
source /crazyflie_ws/devel/setup.bash

catkin clean --yes
catkin build

echo "========================================================="
echo "build done."
echo "========================================================="

# roslaunch rotors_gazebo crazyflie2_hovering_swarm.launch gui:=false & # old version
# roslaunch rotors_gazebo crazyflie2_swarm15.launch gui:=false & # big swarm with 15 quadcopters
# roslaunch rotors_gazebo crazyflie2_swarm.launch gui:=false & # small swarm with 4 quadcopters

date_hash_info="${date_hash}_mpc1"
echo "========================================================="
echo "running ${date_hash_info}"
echo "========================================================="

roslaunch rotors_gazebo crazyflie2_swarm15.launch gui:=false swarm_mode:=mpc1 & # big swarm with 15 quadcopters

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

/crazyflie_ws/src/crazys/docker/generate-video.sh "${date_hash_info}"
mv /crazyflie_ws/src/crazys/log_output /crazyflie_ws/src/crazys/log_${date_hash_info}



date_hash_info="${date_hash}_reynolds"
echo "========================================================="
echo "running ${date_hash_info}"
echo "========================================================="

roslaunch rotors_gazebo crazyflie2_swarm15.launch gui:=false swarm_mode:=reynolds & # big swarm with 15 quadcopters

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

/crazyflie_ws/src/crazys/docker/generate-video.sh "${date_hash_info}"
mv /crazyflie_ws/src/crazys/log_output /crazyflie_ws/src/crazys/log_${date_hash_info}
