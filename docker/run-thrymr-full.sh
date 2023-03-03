#!/bin/bash

RUN_START=`date`
RUN_START_INT=`date +%s`
rm -f /tmp/stop
cd ~/SWARM/crazys

DOCKER_PIDS=()
function wait_until_max_procs_running {
  NEWPID=$!

  MAX_RUNNING=6
  if [ $# -eq 1 ]
  then
    MAX_RUNNING=$1
  fi;

  DOCKER_PIDS+=($NEWPID)
  echo "New instance started with pid $NEWPID. Wait 5 sec."
  sleep 5

  while : ; do
    NPROC_RUNNING=0
    for value in "${DOCKER_PIDS[@]}"
    do
        if kill -0 $value 2> /dev/null
        then
          #echo "pid $value still running"
          NPROC_RUNNING=$(($NPROC_RUNNING + 1))
        #else
        #  echo "pid $value already done"
        fi;
    done
    if pidof cmake > /dev/null 2> /dev/null
    then
      echo "$NPROC_RUNNING processed running of $MAX_RUNNING max. cmake running: should wait 10 sec."
      sleep 10;
    elif [ $NPROC_RUNNING -ge $MAX_RUNNING ]
    then
      echo "$NPROC_RUNNING processed running of $MAX_RUNNING max: should wait 10 sec."
      sleep 10;
    else
      echo "$NPROC_RUNNING processed running of $MAX_RUNNING max: ok, proceed."
      break;
    fi;
  done;

  if test -e /tmp/stop
  then
    RUN_END=`date`
    RUN_END_INT=`date +%s`
    DURATION=`echo "scale=2;(${RUN_END_INT} - ${RUN_START_INT}) / (60*60)" | bc | awk '{printf "%.2f", $0}'`
    echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
    echo "%   aborted by /tmp/stop - not starting any new instances."
    echo "% RUN_START:  ${RUN_START}"
    echo "% RUN_END:    ${RUN_END}"
    echo "% DURATION:   ${DURATION} hours"
    echo "%   run-thrymr-full.sh done."
    echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
    exit 7 # stop if flag is present
  fi;
}

rm -f rotors_gazebo/resource/crazyflie2_mpc1_dyn_*.yaml
# test different separation weights
for i in a b c d e
do
#  for j in 5 7 10 12 15 20 30 50 70 100 150 200 250 300 500 700 1000 2000 3000 # dyn_sep
#  for j in 600 700 850 1000 1200 1500 1800 2200 # dyn_sep
#  for j in 1 2 5 7 10 12 15 17 20 25 30 50 120 150 # dyn_thr
#  for j in 70 100 150 250 350 500 700 1000 1500 2500 3500 # dyn_tar
#  for j in 5 10 20 50 100 200 250 500 1000 # dyn_sca
#  for j in 0 2 5 7 10 15 22 30 50 75 110 200 # dyn_cal
#  for j in 5 7 10 15 20 35 50 70 100 # dyn_eps
#  for j in 0 1 2 3 5 7 10 15 20 35 50 # dyn_nse
#  for j in 2 5 7 8 10 12 15 20 35 50 70 100 150 200 # dyn_eps
#  for j in 6 12 # dyn_nmm
#  for j in 200 250 300 500 700 1000 2000 3000 # dyn_tar
#  for j in 100 150 200 300 500 1000 # dyn_sca
#  for j in 248 249 250 251 252 # dyn_tar
#  for j in 10 20 30 50 62 75 87 105 120 140 200 # dyn_ese
#  for j in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14
#  for j in 10 15 17 20 23 25 27 30 33 35 40 45 50 990
#  for j in 5 6 7 8 9 10 12 15 20 # dyn_eps
#  for j in 0 5 10 20 40 80 160 320 640 1280  # dyn_hgh
#  for j in 1000 100 50 25 20 15 12 10 7 5 2  # dyn_hzd
#for j in 0 5 10 20 50 100 # dyn_nse
for j in 10 # dyn_nse
  do
    #yamlname=`pwgen -n 4 1`
    yamlname=`printf "%05d%s" $j $i`

    dyn_nse=`echo "scale=2;$j / 100" | bc | awk '{printf "%.2f", $0}'`
    #dyn_nse="0.1"
    #dyn_eps=`echo "scale=2;$j / 100" | bc | awk '{printf "%.2f", $0}'`
    dyn_eps="0.05" # "0.1" # "0.05"
    dyn_nmm="6" # 6
    dyn_sep="350" # 350
    dyn_thr="0.15" # "0.12"
    dyn_tar="250" # "200" # "250" # "150"
    #dyn_sca=`echo "scale=2;$j / 100" | bc | awk '{printf "%.2f", $0}'`
    dyn_sca="2"
    dyn_cal="0" # 30, 5
    #dyn_ese=`echo "scale=2;$j / 100" | bc | awk '{printf "%.2f", $0}'`
    dyn_ese="0.87"
    #dyn_nhd=`echo "scale=1;$j / 10" | bc | awk '{printf "%.1f", $0}'`
    dyn_nhd="2.5"
    dyn_hgh="40"
    dyn_hzd="17"

    echo "i = $i, j = $j, dyn_nse = $dyn_nse, dyn_eps = $dyn_eps, dyn_nmm = $dyn_nmm, dyn_sep = $dyn_sep, dyn_thr = $dyn_thr, dyn_tar = $dyn_tar, dyn_sca = $dyn_sca, dyn_cal = $dyn_cal, dyn_ese = $dyn_ese, dyn_nhd = $dyn_nhd, dyn_hgh = $dyn_hgh, dyn_hzd = $dyn_hzd"

    cp rotors_gazebo/resource/crazyflie2_mpc1_placeholder.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
    sed -i "s/__DYN_NSE__/$dyn_nse/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
    sed -i "s/__DYN_EPS__/$dyn_eps/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
    sed -i "s/__DYN_NMM__/$dyn_nmm/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
    sed -i "s/__DYN_SEP__/$dyn_sep/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
    sed -i "s/__DYN_THR__/$dyn_thr/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
    sed -i "s/__DYN_TAR__/$dyn_tar/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
    sed -i "s/__DYN_SCA__/$dyn_sca/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
    sed -i "s/__DYN_CAL__/$dyn_cal/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
    sed -i "s/__DYN_ESE__/$dyn_ese/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
    sed -i "s/__DYN_NHD__/$dyn_nhd/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
    sed -i "s/__DYN_HGH__/$dyn_hgh/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
    sed -i "s/__DYN_HZD__/$dyn_hzd/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml

    #extratext="dyn_nmm${dyn_nmm}"
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist2_rover dist mpc1_dyn_${yamlname} 0 6 "${extratext}" &
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9_rover dist mpc1_dyn_${yamlname} 0 6 "${extratext}" &

    extratext="dyn_nse${dyn_nse}"

    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 dist mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 distGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 elev mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 elevGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running

    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 dist mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 dist mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 dist mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist2 dist mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running

    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 elev mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 elev mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 elev mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist2 elev mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running

    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist4_hw dist mpc1_dyn_${yamlname} 0 8 "${extratext}" &
    wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist4_hw distGT mpc1_dyn_${yamlname} 0 8 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist4_hw elev mpc1_dyn_${yamlname} 0 8 "${extratext}" &
    #wait_until_max_procs_running

  done
done
wait_until_max_procs_running 1
rm -f rotors_gazebo/resource/crazyflie2_mpc1_dyn_*.yaml

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


if [ "$j" = "15" ]; then
  dyn_sep="950"
fi;
if [ "$j" = "20" ]; then
  dyn_sep="900"
fi;
if [ "$j" = "30" ]; then
  dyn_sep="800"
fi;
if [ "$j" = "50" ]; then
  dyn_sep="610"
fi;
if [ "$j" = "62" ]; then
  dyn_sep="510"
fi;
if [ "$j" = "75" ]; then
  dyn_sep="420"
fi;
if [ "$j" = "87" ]; then
  dyn_sep="350"
fi;
if [ "$j" = "105" ]; then
  dyn_sep="255"
fi;
if [ "$j" = "120" ]; then
  dyn_sep="200"
fi;
if [ "$j" = "140" ]; then
  dyn_sep="125"
fi;
if [ "$j" = "200" ]; then
  dyn_sep="25"
fi;
