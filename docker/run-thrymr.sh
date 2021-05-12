#!/bin/bash

cd ~/SWARM/crazys
# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` mpc1
# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reynolds
# docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reylimited

# compare 2-drone szenario
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` mpc1
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reynolds
docker run --rm --volume ~/SWARM/crazys:/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD` reylimited
