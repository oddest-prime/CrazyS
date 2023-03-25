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
    echo "%   run-thrymr.sh done."
    echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
    exit 7 # stop if flag is present
  fi;
}

for i in a b c d e
do
    for j in 300 99999 # dyn_nhd
    do
        for k in 150 170 # dyn_sca
        do
        yamlname=`printf "%05d%s%s" $j $i $k`
        dyn_nhd=`echo "scale=2;$j / 100" | bc | awk '{printf "%.2f", $0}'`
        dyn_sca=`echo "scale=2;$k / 100" | bc | awk '{printf "%.2f", $0}'`

        cp rotors_gazebo/resource/crazyflie2_mpc1_params2a.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
        sed -i "s/__DYN_NHD__/$dyn_nhd/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml
        sed -i "s/__DYN_SCA__/$dyn_sca/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_${yamlname}.yaml

        extratext="dyn_nhd${dyn_nhd}dyn_sca${dyn_sca}fin"

    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 dist mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 distGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 elev mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 elevGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    #wait_until_max_procs_running

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

    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 elev mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    wait_until_max_procs_running
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 elev mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    wait_until_max_procs_running
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 elev mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    wait_until_max_procs_running
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist2 elev mpc1_dyn_${yamlname} 0 5 "${extratext}" &
    wait_until_max_procs_running

#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 elevGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
#    wait_until_max_procs_running
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 elevGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
#    wait_until_max_procs_running
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 elevGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
#    wait_until_max_procs_running
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist2 elevGT mpc1_dyn_${yamlname} 0 5 "${extratext}" &
#    wait_until_max_procs_running
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
echo "%   run-thrymr.sh done."
echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
exit 0






# ####################################################################################








#                                                                                                                         hash                         launch      mode    params        obs path extra
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist2_chain chainGT mpc1_params2c 0 9 "chain1" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist2_chain chainGT mpc1_params2c 0 9 "chain2" &
sleep 150 # delay compilation by 150 seconds in second two docker containers
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist2_chain chainGT mpc1_params2c 0 9 "chain3" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist2_chain chainGT mpc1_params2c 0 9 "chain4" &
sleep 150 # delay compilation by 150 seconds in second two docker containers
wait
test -e /tmp/stop && exit 7 # stop if flag is present

exit 0; # ###################################



#                                                                                                                         hash                         launch mode   params        obs path extra
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5_box4 elev mpc1_params2b 0 7 "stabilize1" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5_box4 elev mpc1_params2b 0 7 "stabilize2" &
sleep 150 # delay compilation by 150 seconds in second two docker containers
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5_box4 elevGT mpc1_params2b 0 7 "stabilize1" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5_box4 elevGT mpc1_params2b 0 7 "stabilize2" &
sleep 150 # delay compilation by 150 seconds in second two docker containers
wait
test -e /tmp/stop && exit 7 # stop if flag is present

exit 0; # ###################################


docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 dist mpc1_params2b 0 4 "eel" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 distGT mpc1_params2b 0 4 "eel" &
sleep 150 # delay compilation by 150 seconds in second two docker containers
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 dist mpc1_params2b 0 4 "eel" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 distGT mpc1_params2b 0 4 "eel" &
sleep 150 # delay compilation by 150 seconds in second two docker containers
wait
test -e /tmp/stop && exit 7 # stop if flag is present

docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 dist mpc1_params2c 0 4 "eel" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 distGT mpc1_params2c 0 4 "eel" &
sleep 150 # delay compilation by 150 seconds in second two docker containers
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 dist mpc1_params2c 0 4 "eel" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 distGT mpc1_params2c 0 4 "eel" &
sleep 150 # delay compilation by 150 seconds in second two docker containers
wait
test -e /tmp/stop && exit 7 # stop if flag is present

echo -e "\n`date` - done.\n"; exit 0; # --------------- END EXECUTION




#                                                                                                                         hash                         launch mode params       extra
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 mpc1 mpc1_params2b "run1" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 mpc1 mpc1_params2b "run2" &
sleep 150 # delay compilation by 150 seconds in second two docker containers
#                                                                                                                         hash                         launch mode params       extra
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 mpc1 mpc1_params2b "run3" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 mpc1 mpc1_params2b "run4" &
wait

echo -e "\n`date` - done.\n"; exit 0; # --------------- END EXECUTION

#                                                                                                                         hash                         launch mode params       extra
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 mpc1 mpc1_params2b "run5" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 mpc1 mpc1_params2b "run6" &
sleep 150 # delay compilation by 150 seconds in second two docker containers
#                                                                                                                         hash                         launch mode params       extra
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 mpc1 mpc1_params2b "run7" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 mpc1 mpc1_params2b "run8" &
wait

echo -e "\n`date` - done.\n"; exit 0; # --------------- END EXECUTION


docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 mpc1 mpc1_params2 "rel_dist" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 mpc1 mpc1_params2 "rel_dist" &
sleep 150 # delay compilation by 150 seconds in second two docker containers
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 mpc1 mpc1_params2 "rel_dist" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist1 mpc1 mpc1_params2 "rel_dist" &
wait

docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist15 mpc1 mpc1_params2b "rel_dist" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist9 mpc1 mpc1_params2b "rel_dist" &
sleep 150 # delay compilation by 150 seconds in second two docker containers
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist5 mpc1 mpc1_params2b "rel_dist" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` dist1 mpc1 mpc1_params2b "rel_dist" &
wait

echo -e "\n`date` - done.\n"; exit 0; # --------------- END EXECUTION






# check gradient with enumeration based version
for j in 2 3 4 5
do
  docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9_obstacle2 gradenum mpc1_params$j "obs9_params$j" &
  docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm15_obstacle2 gradenum mpc1_params$j "obs15_params$j" &
  sleep 150 # delay compilation by 150 seconds in second two docker containers
  docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2_obstacle2 gradenum mpc1_params$j "obs2_params$j" &
  docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4_obstacle2 gradenum mpc1_params$j "obs4_params$j" &
  wait

  test -e /tmp/stop && exit 7 # stop if flag is present
done

# check gradient with enumeration based version
for j in 2 3 4 5
do
  docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 gradenum mpc1_params$j "free9_params$j" &
  docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm15 gradenum mpc1_params$j "free15_params$j" &
  sleep 150 # delay compilation by 150 seconds in second two docker containers
  docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 gradenum mpc1_params$j "free2_params$j" &
  docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4 gradenum mpc1_params$j "free4_params$j" &
  wait

  test -e /tmp/stop && exit 7 # stop if flag is present
done

# check gradient based version with vector distance
for j in 2 3 4 5
do
  docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm15_obstacle2 gradient mpc1_params$j "obs15_params$j" &
  docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9_obstacle2 gradient mpc1_params$j "obs9_params$j" &
  sleep 150 # delay compilation by 150 seconds in second two docker containers
  docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2_obstacle2 gradient mpc1_params$j "obs2_params$j" &
  docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4_obstacle2 gradient mpc1_params$j "obs4_params$j" &
  wait

  test -e /tmp/stop && exit 7 # stop if flag is present
done

# check gradient based version with vector distance
for j in 2 3 4 5
do
  docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm15 gradient mpc1_params$j "free15_params$j" &
  docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 gradient mpc1_params$j "free9_params$j" &
  sleep 150 # delay compilation by 150 seconds in second two docker containers
  docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 gradient mpc1_params$j "free2_params$j" &
  docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4 gradient mpc1_params$j "free4_params$j" &
  wait

  test -e /tmp/stop && exit 7 # stop if flag is present
done


exit 0; # ###################################

# this was the big simulation batch per 2021-06-29...
for i in 2 # dyn_n
do
  for j in 2 4 6 8 10 12 14 16 18 20 # dyn_eps
  do
    dyn_n="$i"
    dyn_eps=`echo "scale=2;$j * 2.0 / 100 / $i" | bc | awk '{printf "%.2f", $0}'`
    echo "i = $i, j = $j, dyn_n = $dyn_n, dyn_eps = $dyn_eps"

    cp rotors_gazebo/resource/crazyflie2_mpc1_placeholder.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_N__/$dyn_n/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_EPS__/$dyn_eps/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml

    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_dyn_a "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm15 mpc1 mpc1_dyn_a "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}" &
    sleep 150 # delay compilation by 150 seconds in second two docker containers
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 mpc1 mpc1_dyn_a "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4 mpc1 mpc1_dyn_a "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}" &
    wait

    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
  done
done

exit 0;










# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` mpc1
# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reynolds
# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reylimited

# compare 2-drone szenario
# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` mpc1 swarm_params
# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reynolds swarm_params
# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reylimited swarm_params
# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reyvelocity swarm_params

# compare MPC1 with different params (swarm of 15)
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` mpc1 mpc1_params1
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` mpc1 mpc1_params2
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` mpc1 mpc1_params3
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` mpc1 mpc1_params4
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` mpc1 mpc1_params5
#wait
exit 0;

#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reynolds reynolds_params1 &
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reynolds reynolds_params2 &
#wait
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reylimited reynolds_params1 &
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reyvelocity reynolds_params1 &
#wait

# run 6 different parameter for 9 drone swarm
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_params1 &
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_params2 &
#wait
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_params3 &
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_params4 &
#wait
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_params5 &
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_params6 &
#wait

# check difference for centroid version
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4 mpc1 mpc1_params1 "centroid4" &
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_params1 "centroid9" &


# simulation batch per 2021-07-08 for different scaling of gradient controller
for i in 2 # dyn_n
do
#  for j in `seq 1 15` # dyn_eps
  for j in 8 15 # dyn_eps
  do
    dyn_n="$i"
    dyn_eps_a="0.06"
    dyn_eps_b="0.08"
    dyn_eps_c="0.12"
    dyn_eps_d="0.15"
    dyn_scale=`echo "scale=2;$j * 3 / 1000 / $i" | bc | awk '{printf "%.2f", $0}'`
    echo "i = $i, j = $j, dyn_n = $dyn_n, dyn_eps_a = $dyn_eps_a, dyn_eps_b = $dyn_eps_b, dyn_scale = $dyn_scale"

    cp rotors_gazebo/resource/crazyflie2_mpc1_placeholder.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_EPS__/$dyn_eps_a/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_SCALE__/$dyn_scale/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml

    cp rotors_gazebo/resource/crazyflie2_mpc1_placeholder.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_b.yaml
    sed -i "s/__DYN_EPS__/$dyn_eps_b/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_b.yaml
    sed -i "s/__DYN_SCALE__/$dyn_scale/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_b.yaml

    cp rotors_gazebo/resource/crazyflie2_mpc1_placeholder.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_c.yaml
    sed -i "s/__DYN_EPS__/$dyn_eps_c/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_c.yaml
    sed -i "s/__DYN_SCALE__/$dyn_scale/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_c.yaml

    cp rotors_gazebo/resource/crazyflie2_mpc1_placeholder.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_d.yaml
    sed -i "s/__DYN_EPS__/$dyn_eps_c/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_d.yaml
    sed -i "s/__DYN_SCALE__/$dyn_scale/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_d.yaml

#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 mpc1 mpc1_dyn_a "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}_sep-${sep_a}" &
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 mpc1 mpc1_dyn_b "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}_sep-${sep_b}" &

    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 gradient mpc1_dyn_a "gradient9_dyn_eps-${dyn_eps_a}_scale-${dyn_scale}" &
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 gradient mpc1_dyn_b "gradient9_dyn_eps-${dyn_eps_b}_scale-${dyn_scale}" &
    #sleep 150 # delay compilation by 150 seconds in second two docker containers
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4 gradient mpc1_dyn_a "gradient4_dyn_eps-${dyn_eps_a}_scale-${dyn_scale}" &
    #docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4 gradient mpc1_dyn_b "gradient4_dyn_eps-${dyn_eps_b}_scale-${dyn_scale}" &
    #sleep 150 # delay compilation by 150 seconds in second two docker containers


    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 gradient mpc1_dyn_a "gradient2_dyn_eps-${dyn_eps_a}_scale-${dyn_scale}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 gradient mpc1_dyn_b "gradient2_dyn_eps-${dyn_eps_b}_scale-${dyn_scale}" &
    sleep 120 # delay compilation by 120 seconds in second two docker containers
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 gradient mpc1_dyn_c "gradient2_dyn_eps-${dyn_eps_c}_scale-${dyn_scale}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 gradient mpc1_dyn_d "gradient2_dyn_eps-${dyn_eps_d}_scale-${dyn_scale}" &
    wait

    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4 gradient mpc1_dyn_a "gradient4_dyn_eps-${dyn_eps_a}_scale-${dyn_scale}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4 gradient mpc1_dyn_b "gradient4_dyn_eps-${dyn_eps_b}_scale-${dyn_scale}" &
    sleep 120 # delay compilation by 120 seconds in second two docker containers
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4 gradient mpc1_dyn_c "gradient4_dyn_eps-${dyn_eps_c}_scale-${dyn_scale}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4 gradient mpc1_dyn_d "gradient4_dyn_eps-${dyn_eps_d}_scale-${dyn_scale}" &
    wait

    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 gradient mpc1_dyn_a "gradient9_dyn_eps-${dyn_eps_a}_scale-${dyn_scale}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 gradient mpc1_dyn_b "gradient9_dyn_eps-${dyn_eps_b}_scale-${dyn_scale}" &
    sleep 120 # delay compilation by 120 seconds in second two docker containers
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 gradient mpc1_dyn_c "gradient9_dyn_eps-${dyn_eps_c}_scale-${dyn_scale}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 gradient mpc1_dyn_d "gradient9_dyn_eps-${dyn_eps_d}_scale-${dyn_scale}" &
    wait

    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm15 gradient mpc1_dyn_a "gradient15_dyn_eps-${dyn_eps_a}_scale-${dyn_scale}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm15 gradient mpc1_dyn_b "gradient15_dyn_eps-${dyn_eps_b}_scale-${dyn_scale}" &
    sleep 120 # delay compilation by 120 seconds in second two docker containers
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm15 gradient mpc1_dyn_c "gradient15_dyn_eps-${dyn_eps_c}_scale-${dyn_scale}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm15 gradient mpc1_dyn_d "gradient15_dyn_eps-${dyn_eps_d}_scale-${dyn_scale}" &
    wait

    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_b.yaml
    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_c.yaml
    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_d.yaml
  done
done


exit 0;
# simulation batch per 2021-07-05 for different seperation and target weights (centroid mode)
for i in 2 # dyn_n
do
  for j in `seq 1 20` # dyn_eps
  do
    dyn_n="$i"
    dyn_eps=`echo "scale=2;$j * 2.0 / 100 / $i" | bc | awk '{printf "%.2f", $0}'`
    sep_a="6.0"
    sep_b="12.0"
    tar_a="50.0"
    tar_b="100.0"
    echo "i = $i, j = $j, dyn_n = $dyn_n, dyn_eps = $dyn_eps, sep_a = $sep_a, sep_b = $sep_b, tar_a = $tar_a, tar_b = $tar_b"

    cp rotors_gazebo/resource/crazyflie2_mpc1_placeholder.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_N__/$dyn_n/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_EPS__/$dyn_eps/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_SEP__/$sep_a/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_TAR__/$tar_a/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml

    cp rotors_gazebo/resource/crazyflie2_mpc1_placeholder.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_b.yaml
    sed -i "s/__DYN_N__/$dyn_n/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_b.yaml
    sed -i "s/__DYN_EPS__/$dyn_eps/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_b.yaml
    sed -i "s/__DYN_SEP__/$sep_b/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_b.yaml
    sed -i "s/__DYN_TAR__/$tar_a/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_b.yaml

    cp rotors_gazebo/resource/crazyflie2_mpc1_placeholder.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_c.yaml
    sed -i "s/__DYN_N__/$dyn_n/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_c.yaml
    sed -i "s/__DYN_EPS__/$dyn_eps/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_c.yaml
    sed -i "s/__DYN_SEP__/$sep_a/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_c.yaml
    sed -i "s/__DYN_TAR__/$tar_b/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_c.yaml

    cp rotors_gazebo/resource/crazyflie2_mpc1_placeholder.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_d.yaml
    sed -i "s/__DYN_N__/$dyn_n/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_d.yaml
    sed -i "s/__DYN_EPS__/$dyn_eps/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_d.yaml
    sed -i "s/__DYN_SEP__/$sep_b/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_d.yaml
    sed -i "s/__DYN_TAR__/$tar_b/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_d.yaml

#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 mpc1 mpc1_dyn_a "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}_sep-${sep_a}" &
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 mpc1 mpc1_dyn_b "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}_sep-${sep_b}" &

    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_dyn_a "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}_sep-${sep_a}_tar-${tar_a}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_dyn_b "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}_sep-${sep_b}_tar-${tar_a}" &
    sleep 30 # delay compilation by 30 seconds in second two docker containers
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_dyn_c "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}_sep-${sep_a}_tar-${tar_b}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_dyn_d "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}_sep-${sep_b}_tar-${tar_b}" &
    wait

    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_b.yaml
    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_c.yaml
    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_d.yaml
  done
done

exit 0;
# this was the big simulation batch per 2021-06-29...
for i in `seq 1 4` # dyn_n
do
  for j in `seq 1 20` # dyn_eps
  do
    dyn_n="$i"
    dyn_eps=`echo "scale=2;$j * 2.0 / 100 / $i" | bc | awk '{printf "%.2f", $0}'`
    echo "i = $i, j = $j, dyn_n = $dyn_n, dyn_eps = $dyn_eps"

    cp rotors_gazebo/resource/crazyflie2_mpc1_placeholder.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_N__/$dyn_n/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_EPS__/$dyn_eps/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml

    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 mpc1 mpc1_dyn_a "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4 mpc1 mpc1_dyn_a "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}" &
    wait

    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_dyn_a "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm15 mpc1 mpc1_dyn_a "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}" &
    wait

    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
  done
done
