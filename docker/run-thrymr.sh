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

# check difference for centroid version
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4 mpc1 mpc1_params1 "centroid4" &
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 mpc1 mpc1_params1 "centroid9" &


# simulation batch per 2021-07-08 for different scaling of gradient controller
for i in 2 # dyn_n
do
#  for j in `seq 1 15` # dyn_eps
  for j in 8 # dyn_eps
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
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 gradient mpc1_dyn_b "gradient2_dyn_eps-${dyn_eps_b}_scale-${dyn_scale}" &
#    sleep 120 # delay compilation by 150 seconds in second two docker containers
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 gradient mpc1_dyn_c "gradient2_dyn_eps-${dyn_eps_c}_scale-${dyn_scale}" &
#    docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 gradient mpc1_dyn_d "gradient2_dyn_eps-${dyn_eps_d}_scale-${dyn_scale}" &
#    wait

    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_a.yaml
    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_b.yaml
    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_c.yaml
    rm -rf rotors_gazebo/resource/crazyflie2_mpc1_dyn_d.yaml
  done
done


exit 0;
# check gradient based version
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 gradient mpc1_params1 "gradient2_params1" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4 gradient mpc1_params1 "gradient4_params1" &
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 gradient mpc1_params1 "gradient9_params1" &
wait
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm2 gradient mpc1_params2 "gradient2_params2" &
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm4 gradient mpc1_params2 "gradient4_params2" &
#docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` swarm9 gradient mpc1_params2 "gradient9_params2" &
#wait




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
