#!/bin/bash

TIMEOUT=2500
# TIMEOUT=1500 # debug, faster runs...

Xvfb -shmem -screen 0 1280x1024x24 &
export DISPLAY=:0

launch_file="crazyflie2_swarm15.launch"  # big swarm with 15 quadcopters

hash="uu"
mode="mm"
params="pp"
if [ $# -ge 3 ]
then
  hash="$1"
  mode="$2"
  params="$3"
fi;
date_hash="`date +%Y-%m-%d_%H-%M-%S`_${hash}"
date_hash_mode="${date_hash}_${mode}_${params}"
echo "========================================================="
echo "git hash: ${hash}"
echo "swarm mode: ${mode}"
echo "swarm params: ${params}"
echo "date, hash and mode: ${date_hash_mode}"
echo "launch file: ${launch_file}"
echo "========================================================="

echo "++ drone SWARM simulation ++"  > /tmp/video.info
echo " "                                         >> /tmp/video.info
echo "=========================================================" >> /tmp/video.info
echo "git hash: ${hash}"                                         >> /tmp/video.info
echo "swarm mode: ${mode}"                                       >> /tmp/video.info
echo "swarm params: ${params}"                                   >> /tmp/video.info
echo "date, hash and mode: ${date_hash_mode}"                    >> /tmp/video.info
echo "launch file: ${launch_file}"                               >> /tmp/video.info
echo "=========================================================" >> /tmp/video.info
echo "params file content:" >> /tmp/video.info
echo "---------------------------------------------------------" >> /tmp/video.info
cat /crazyflie_ws/src/crazys/rotors_gazebo/resource/crazyflie2_${params}.yaml  >> /tmp/video.info

cd /crazyflie_ws
source /opt/ros/melodic/setup.bash
source /crazyflie_ws/devel/setup.bash

catkin clean --yes
catkin build

echo "========================================================="
echo "build done."
echo "========================================================="

mkdir -p /tmp/log_output
# roslaunch rotors_gazebo crazyflie2_swarm2.launch gui:=false swarm_mode:=${mode} & # simplified szenario with 2 quadcopters
# roslaunch rotors_gazebo crazyflie2_swarm4.launch gui:=false swarm_mode:=${mode} & # small swarm with 4 quadcopters
# roslaunch rotors_gazebo crazyflie2_swarm15.launch gui:=false swarm_mode:=${mode} swarm_params:=${params} & # big swarm with 15 quadcopters

roslaunch rotors_gazebo ${launch_file} gui:=false swarm_mode:=${mode} swarm_params:=${params} &

sleep 1
iter=1
R_PID=`pgrep roslaunch`
for i in `seq 1 $TIMEOUT`
do
    kill -0 $R_PID 2> /dev/null || break
    echo "  [[ ${date_hash_mode} - global sim timeout counter: $iter / $TIMEOUT ]]  "
    iter=$((iter+1))
    sleep 1
done;
echo $iter > /tmp/run.seconds
echo ""
echo "##  killing roslaunch";
kill $R_PID 2> /dev/null
sleep 5
kill `pidof Xvfb`

echo "=========================================================" >> /tmp/video.info
echo "run time: ${iter} seconds (max: $TIMEOUT)" >> /tmp/video.info
echo "=========================================================" >> /tmp/video.info

/crazyflie_ws/src/crazys/docker/generate-video.sh "${date_hash_mode}"
mv /tmp/log_output /crazyflie_ws/src/crazys/log_${date_hash_mode}
