#!/bin/bash

RUN_START=`date`
rm -f /tmp/stop

cd ~/SWARM/crazys

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
echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
echo "% RUN_START: ${RUN_START}"
echo "% RUN_END:   ${RUN_END}"
echo "%   run-thrymr-full.sh done."
echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
