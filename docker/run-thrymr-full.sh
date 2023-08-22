#!/bin/bash

RUN_START=`date`
RUN_START_INT=`date +%s`
rm -f /tmp/stop
cd ~/SWARM/crazys

DOCKER_PIDS=()
function wait_until_max_procs_running {
  NEWPID=$!

  MAX_RUNNING=8
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
for i in a
do
#  for j in 5 7 10 12 15 20 30 50 70 100 150 200 250 300 500 700 1000 2000 3000 # dyn_sep
#  for j in 600 700 850 1000 1200 1500 1800 2200 # dyn_sep
#  for j in 5 10 15 20 25 30 40 50 60 75 100 # dyn_thr
#  for j in 70 100 150 250 350 500 700 1000 1500 2500 3500 # dyn_tar
#  for j in 5 10 20 50 100 200 250 500 1000 # dyn_sca
#  for j in 0 2 5 7 10 15 22 30 50 75 110 200 # dyn_cal
#  for j in 5 7 10 15 20 35 50 70 100 # dyn_eps
#  for j in 0 1 2 3 5 7 10 15 20 35 50 # dyn_nse
#  for j in 2 5 7 8 10 12 15 20 35 50 70 100 150 200 # dyn_eps
#  for j in 6 12 # dyn_nmm
#  for j in 250 500 1000 2000 5000 6000 10000 # dyn_tar
#  for j in 100 150 200 300 500 1000 # dyn_sca
#  for j in 248 249 250 251 252 # dyn_tar
#  for j in 50 75 87 92 100 110 120 140 200 # dyn_ese
#  for j in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14
#  for j in 10 15 17 20 23 25 27 30 33 35 40 45 50 990
#  for j in 5 6 7 8 9 10 12 15 20 # dyn_eps
#  for j in 0 5 10 20 40 80 160 320 640 1280  # dyn_hgh
#  for j in 1000 100 50 25 20 15 12 10 7 5 2  # dyn_hzd
#  for j in 0 2 3 5 6 7 8 10 12 15 20 25 30 # dyn_nse
#  for j in 100 120 150 180 # dyn_sep
#  for j in 15 # dyn_thr
#  for j in 150 170 190 210 230 250 # dyn_sca
#  for j in 150 200 250 300 400 500 999 # dyn_nhd
#  for j in 2 5 7 10 20 30  # dyn_iir
for j in 1 # fake loop
#for j in 20 0 2 5 10 15 # dyn_nse
do
    #for c in 0 2 3 # dyn_col
    #for c in 0 1 # dyn_col
    for c in 1 # dyn_sup
    do
    yamlname=`printf "%05d%s%s" $j $i $c`
    jscaled=`echo "scale=2;$j / 100" | bc | awk '{printf "%.2f", $0}'`

    dyn_nse="0.0" # 0.05 0.1
    #dyn_eps="$jscaled"
    dyn_eps="0.07" # "0.1" # "0.05" "0.15"
    dyn_nmm="6" # 6 # 3
    dyn_sep="100" # 350 # "100" "120"
    dyn_tar="6000" # "200" # "250" # "150" # 250
    dyn_hgh="800"
    dyn_cal="0" # 30, 5
    #dyn_sca="$jscaled"
    dyn_sca="1.5" # "2"
    dyn_thr="0.55" # "0.15" # "0.12" # "0.55"
    #dyn_thr="$jscaled"
    #dyn_ese="$jscaled"
    dyn_ese="1.35" # "0.87" # "1.10"
    #dyn_nhd="$jscaled"
    dyn_nhd="3.0" # "999.99"
    dyn_hzd="17"
    dyn_col="0" # "3" $c
    dyn_sup="1"
    dyn_iir="0.3"

    echo "i = $i, j = $j, dyn_nse = $dyn_nse, dyn_col = $dyn_col, dyn_eps = $dyn_eps, dyn_nmm = $dyn_nmm, dyn_sep = $dyn_sep, dyn_thr = $dyn_thr, dyn_tar = $dyn_tar, dyn_sca = $dyn_sca, dyn_cal = $dyn_cal, dyn_ese = $dyn_ese, dyn_nhd = $dyn_nhd, dyn_hgh = $dyn_hgh, dyn_hzd = $dyn_hzd"

    cp rotors_gazebo/resource/crazyflie2_mpc1_placeholder.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
    #cp rotors_gazebo/resource/crazyflie2_mpc1_params2d.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
    sed -i "s/__DYN_NSE__/$dyn_nse/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
    sed -i "s/__DYN_COL__/$dyn_col/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
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
    sed -i "s/__DYN_IIR__/$dyn_iir/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
    sed -i "s/__DYN_SUP__/$dyn_sup/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml

    #extratext="dyn_nmm${dyn_nmm}"
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist2_rover dist mpc1_dyn_${yamlname} 0 6 "${extratext}" &
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9_rover dist mpc1_dyn_${yamlname} 0 6 "${extratext}" &

    #extratext="dyn_sup${dyn_sup}dyn_iir${dyn_iir}dyn_nse${dyn_nse}"
    extratext="dyn_j${j}"

#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` join15 dist mpc1_dyn_${yamlname} 0 9 "${extratext}" &
#    wait_until_max_procs_running

    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 dist mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 distGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 elev mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 elevGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running

    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 cyclic mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    wait_until_max_procs_running
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 cyclic mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    wait_until_max_procs_running
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 cyclic mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    wait_until_max_procs_running
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist2 cyclic mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    wait_until_max_procs_running

    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 dist mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    wait_until_max_procs_running
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 dist mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    wait_until_max_procs_running
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 dist mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    wait_until_max_procs_running
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist2 dist mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    wait_until_max_procs_running


#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 distGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
#    wait_until_max_procs_running
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 distGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
#    wait_until_max_procs_running
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 distGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
#    wait_until_max_procs_running
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist2 distGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
#    wait_until_max_procs_running

#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 elev mpc1_dyn_${yamlname} 0 5 "${extratext}" &
#    wait_until_max_procs_running
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 elev mpc1_dyn_${yamlname} 0 5 "${extratext}" &
#    wait_until_max_procs_running
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 elev mpc1_dyn_${yamlname} 0 5 "${extratext}" &
#    wait_until_max_procs_running
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist2 elev mpc1_dyn_${yamlname} 0 5 "${extratext}" &
#    wait_until_max_procs_running

#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 elevGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
#    wait_until_max_procs_running
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 elevGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
#    wait_until_max_procs_running
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 elevGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
#    wait_until_max_procs_running
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist2 elevGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
#    wait_until_max_procs_running

    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5_chain chain mpc1_dyn_${yamlname} 0 8 "${extratext}" &
    #wait_until_max_procs_running

    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist4_hw dist mpc1_dyn_${yamlname} 0 8 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist4_hw distGT mpc1_dyn_${yamlname} 0 8 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist4_hw elev mpc1_dyn_${yamlname} 0 8 "${extratext}" &
    #wait_until_max_procs_running

  done
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
