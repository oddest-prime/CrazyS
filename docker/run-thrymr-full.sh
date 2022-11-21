#!/bin/bash

RUN_START=`date`
RUN_START_INT=`date +%s`
rm -f /tmp/stop
cd ~/SWARM/crazys

# test different separation weights
for i in a
do
#  for j in 5 7 10 12 15 20 30 50 70 100 150 200 250 300 500 700 1000 2000 3000 # dyn_sep
#  for j in 600 700 850 1000 1200 1500 1800 2200 # dyn_sep
#  for j in 1 2 5 7 10 12 15 17 20 25 30 50 120 150 # dyn_thr
#  for j in 70 100 150 250 350 500 700 1000 1500 2500 3500 # dyn_tar
#  for j in 5 10 20 50 100 200 250 500 1000 # dyn_sca
#  for j in 0 5 10 20 35 50 75 100 150 200 300 500 # dyn_cal
#  for j in 5 7 10 15 20 35 50 70 100 # dyn_eps
#  for j in 0 1 2 3 5 7 10 15 20 35 50 # dyn_nse
for j in 2 5 7 8 10 12 15 20 35 50 70 100 150 200 # dyn_eps
  do
    #dyn_nse=`echo "scale=2;$j / 100" | bc | awk '{printf "%.2f", $0}'`
    dyn_nse="0.1"
    dyn_eps=`echo "scale=2;$j / 100" | bc | awk '{printf "%.2f", $0}'`
    #dyn_eps="0.07"
    dyn_sep="1000"
    dyn_thr="0.15" # "0.12"
    dyn_tar="200" #"250" # "150"
    #dyn_sca=`echo "scale=2;$j / 100" | bc | awk '{printf "%.2f", $0}'`
    dyn_sca="1"
    dyn_cal="5"
    echo "i = $i, j = $j, dyn_nse = $dyn_nse, dyn_eps = $dyn_eps, dyn_sep = $dyn_sep, dyn_thr = $dyn_thr, dyn_tar = $dyn_tar, dyn_sca = $dyn_sca, dyn_cal = $dyn_cal"

    cp rotors_gazebo/resource/crazyflie2_mpc1_placeholder.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_SEP__/$dyn_sep/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_THR__/$dyn_thr/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_TAR__/$dyn_tar/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_SCA__/$dyn_sca/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_CAL__/$dyn_cal/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_EPS__/$dyn_eps/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_NSE__/$dyn_nse/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml

    extratext="dyn_eps${dyn_eps}"
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 dist mpc1_dyn_a 0 5 "${extratext}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 distGT mpc1_dyn_a 0 5 "${extratext}" &
    sleep 150 # delay compilation by 150 seconds in second two docker containers
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 dist mpc1_dyn_a 0 5 "${extratext}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 distGT mpc1_dyn_a 0 5 "${extratext}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist2 dist mpc1_dyn_a 0 5 "${extratext}" &
    wait

    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml

    test -e /tmp/stop && exit 7 # stop if flag is present
  done
done

RUN_END=`date`
RUN_END_INT=`date +%s`
DURATION=`echo "scale=2;(${RUN_END_INT} - ${RUN_START_INT}) / (60*60)" | bc | awk '{printf "%.2f", $0}'`
echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
echo "% RUN_START:  ${RUN_START}"
echo "% RUN_END:    ${RUN_END}"
echo "% DURATION:   ${DURATION} hours"
echo "%   run-thrymr-full.sh done."
echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
exit 0








# full simulation run, Sept 2022
#for params in "spc_global_A" "spc_local_A" "spc_global_B" "spc_local_B"
for ndrones in "30" "15" "9" "4"
do
  for obss in "0" "5" "6" "7"
  do
    pa="1"
    if [ "$obss" = "0" ]
    then
      pa="2"
    fi;
    if [ "$obss" = "7" ]
    then
      pa="3"
    fi;
    echo "running: ndrones=$ndrones obss=$obss pa=$pa"

    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm$ndrones gradient spc_local_A $obss $pa "full" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm$ndrones gradient spc_local_B $obss $pa "full" &
    sleep 150 # delay compilation by 150 seconds in second two docker containers
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm$ndrones gradenum spc_local_A $obss $pa "full" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm$ndrones gradenum spc_local_B $obss $pa "full" &
    wait

    test -e /tmp/stop && exit 7 # stop if flag is present
  done
done

RUN_END=`date`
echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
echo "% RUN_START: ${RUN_START}"
echo "% RUN_END:   ${RUN_END}"
echo "%   run-thrymr-full.sh done."
echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"












exit 0; # ###################################

# full simulation run, February 2022
#for params in "spc_global_A" "spc_local_A" "spc_global_B" "spc_local_B"
for params in "spc_local_A" "spc_local_B"
do
  for obstacles in "" "_obstacle2" "_obstacle3"
  do
    echo "running: params=$params obstacles=$obstacles"

    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm30$obstacles gradient $params "full" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm30$obstacles gradenum $params "full" &
    sleep 150 # delay compilation by 150 seconds in second two docker containers
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm15$obstacles gradient $params "full" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm15$obstacles gradenum $params "full" &
    wait

    test -e /tmp/stop && exit 7 # stop if flag is present

    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9$obstacles gradient $params "full" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9$obstacles gradenum $params "full" &
    sleep 150 # delay compilation by 150 seconds in second two docker containers
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4$obstacles gradient $params "full" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4$obstacles gradenum $params "full" &
    wait

    test -e /tmp/stop && exit 7 # stop if flag is present
  done
done

RUN_END=`date`
RUN_END_INT=`date +%s`
echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
echo "% RUN_START: ${RUN_START}"
echo "% RUN_END:   ${RUN_END}"
echo "%   run-thrymr-full.sh done."
echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
