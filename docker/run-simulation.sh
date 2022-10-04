#!/bin/bash

TIMEOUT=5000
#TIMEOUT=3000
# TIMEOUT=500 # debug, faster runs...

Xvfb -shmem -screen 0 1280x1024x24 &
export DISPLAY=:0

hash="uu"
mode="ll"
mode="mm"
params="pp"
obstacleScenario="os"
pathScenario="ps"
extra="ee"
if [ $# -ge 3 ]
then
  hash="$1"
  launch="$2"
  mode="$3"
  params="$4"
  obstacleScenario="$5"
  pathScenario="$6"
  extra="$7"
fi;
date_hash="`date +%Y-%m-%d_%H-%M-%S`_${hash}"
date_hash_mode="${date_hash}_${launch}_${mode}_${params}_obs${obstacleScenario}_path${pathScenario}_${extra}"
# launch_file="crazyflie2_swarm15.launch"  # big swarm with 15 quadcopters
launch_file="crazyflie2_${launch}.launch"  # big swarm with 15 quadcopters

echo "========================================================="
echo "git hash: ${hash}"
echo "swarm mode: ${mode}"
echo "swarm params: ${params}"
echo "obstacle scenario: ${obstacleScenario}"
echo "path scenario: ${pathScenario}"
echo "extra filename: ${extra}"
echo "date, hash and mode: ${date_hash_mode}"
echo "launch file: ${launch_file}"
echo "========================================================="

echo "++ drone SWARM simulation ++"  > /tmp/video.info
echo " "                                         >> /tmp/video.info
echo "=========================================================" >> /tmp/video.info
echo "git hash: ${hash}"                                         >> /tmp/video.info
echo "swarm mode: ${mode}"                                       >> /tmp/video.info
echo "swarm params: ${params}"                                   >> /tmp/video.info
echo "obstacle scenario: ${obstacleScenario}"                    >> /tmp/video.info
echo "path scenario: ${pathScenario}"                            >> /tmp/video.info
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

source /crazyflie_ws/devel/setup.bash

echo "========================================================="
echo "build done."
echo "========================================================="

mkdir -p /tmp/log_output
# roslaunch rotors_gazebo crazyflie2_swarm2.launch gui:=false swarm_mode:=${mode} & # simplified szenario with 2 quadcopters
# roslaunch rotors_gazebo crazyflie2_swarm4.launch gui:=false swarm_mode:=${mode} & # small swarm with 4 quadcopters
# roslaunch rotors_gazebo crazyflie2_swarm15.launch gui:=false swarm_mode:=${mode} swarm_params:=${params} & # big swarm with 15 quadcopters

roslaunch rotors_gazebo ${launch_file} gui:=false swarm_mode:=${mode} swarm_params:=${params} obstacleScenario:=${obstacleScenario} pathScenario:=${pathScenario} &

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

cp /crazyflie_ws/src/crazys/docker/Makefile /tmp/log_output/Makefile
cp /crazyflie_ws/src/crazys/docker/metrics.py /tmp/log_output/metrics.py
chmod +x /tmp/log_output/metrics.py

cd /tmp/log_output/
make DistSimMetricsMerged.csv
make StateMerged.csv
make MetricsMerged.png 2> Metrics.txt
make StateMerged.png 2> States.txt
make all-metrics
make all-distance
make all-state
make all-trace

mkdir -p /tmp/plot_output/
mv /tmp/log_output/*.png /tmp/plot_output/.

echo "metrics file content:" >> /tmp/video.info
echo "---------------------------------------------------------" >> /tmp/video.info
grep "metric:" Metrics.txt >> /tmp/video.info
#echo "---------------------------------------------------------" >> /tmp/video.info
#echo "states file content:" >> /tmp/video.info
#echo "---------------------------------------------------------" >> /tmp/video.info
#grep "metric:" States.txt >> /tmp/video.info
echo "=========================================================" >> /tmp/video.info

# save metrics in database

echo "insert into runs values('${date_hash_mode}', '${hash}', '${mode}', '${params}', '${obstacleScenario}', '${pathScenario}', '${extra}', '${launch_file}');" | sqlite3 /crazyflie_ws/src/crazys/simulations.database

SAVEIFS=$IFS
IFS=$( echo -e "\n\b")
for line in `grep "metric:" /tmp/log_output/Metrics.txt`
do
    field_name=`echo $line | cut -d " " -f 2-5`
    field_val=`echo $line | cut -d " " -f 6`
    echo "insert into metrics values('${date_hash_mode}', '${field_name}', '${field_val}');" | sqlite3 /crazyflie_ws/src/crazys/simulations.database
done

linecnt=0
for line in `cat /crazyflie_ws/src/crazys/rotors_gazebo/resource/crazyflie2_${params}.yaml | cut -d "#" -f 1 | jq 'keys[] as $k | [(.[$k] | to_entries[] | [$k, .key, .value] )] | flatten[]'`
do
    linecnt=$(($linecnt + 1))
    if [ $linecnt -eq 1 ]
    then
        line1=`echo $line | cut -d "\"" -f 2`
    fi;
    if [ $linecnt -eq 2 ]
    then
        line2=`echo $line | cut -d "\"" -f 2`
    fi;
    if [ $linecnt -eq 3 ]
    then
        line3=`echo $line | cut -d "\"" -f 2`
        linecnt=0

        field_name="$line1.$line2"
        field_val="$line3"
        echo "insert into params values('${date_hash_mode}', '${field_name}', '${field_val}');" | sqlite3 /crazyflie_ws/src/crazys/simulations.database
    fi;
done;
IFS=$SAVEIFS

cp /tmp/video.info /tmp/log_output/simulation.info
/crazyflie_ws/src/crazys/docker/generate-video.sh "${date_hash_mode}"
mv /tmp/log_output /crazyflie_ws/src/crazys/log_${date_hash_mode}
mv /tmp/plot_output /crazyflie_ws/src/crazys/plot_${date_hash_mode}
