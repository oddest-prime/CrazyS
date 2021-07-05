#!/bin/bash

cd ~/SWARM/crazys
# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` mpc1
# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reynolds
# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reylimited

# compare 2-drone szenario
# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` mpc1 swarm_params
# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reynolds swarm_params
# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reylimited swarm_params
# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reyvelocity swarm_params

# compare MPC1 with different params (swarm of 15)
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` mpc1 mpc1_params1 &
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` mpc1 mpc1_params2 &
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` mpc1 mpc1_params3
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` mpc1 mpc1_params4
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` mpc1 mpc1_params5
#wait

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

docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4 mpc1 mpc1_params1 "centroid4" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_params1 "centroid9" &

exit 0;
# simulation batch per 2021-07-01 for different seperation weights
for i in 2 # dyn_n
do
  for j in `seq 1 20` # dyn_eps
  do
    dyn_n="$i"
    dyn_eps=`echo "scale=2;$j * 2.0 / 100 / $i" | bc | awk '{printf "%.2f", $0}'`
    sep_a="6.0"
    sep_b="12.0"
    echo "i = $i, j = $j, dyn_n = $dyn_n, dyn_eps = $dyn_eps, sep_a = $sep_a, sep_b = $sep_b"

    cp rotors_gazebo/resource/crazyflie2_mpc1_placeholder.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_N__/$dyn_n/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_EPS__/$dyn_eps/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    sed -i "s/__DYN_SEP__/$sep_a/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml

    cp rotors_gazebo/resource/crazyflie2_mpc1_placeholder.yaml rotors_gazebo/resource/crazyflie2_mpc1_dyn_b.yaml
    sed -i "s/__DYN_N__/$dyn_n/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_b.yaml
    sed -i "s/__DYN_EPS__/$dyn_eps/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_b.yaml
    sed -i "s/__DYN_SEP__/$sep_b/g" rotors_gazebo/resource/crazyflie2_mpc1_dyn_b.yaml

    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 mpc1 mpc1_dyn_a "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}_sep-${sep_a}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 mpc1 mpc1_dyn_b "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}_sep-${sep_b}" &

    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_dyn_a "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}_sep-${sep_a}" &
    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_dyn_b "dyn-n_${dyn_n}_dyn_eps-${dyn_eps}_sep-${sep_b}" &
    wait

    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_b.yaml
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
