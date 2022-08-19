#!/bin/bash

cd /home/andreas/SWARM/crazys/docker
HW_LOG_DIR="/home/andreas/SWARM/telesto_logs/current"
MAKEFILE_SRC="/home/andreas/SWARM/crazys/docker/Makefile"
PARAMS_SRC="/home/andreas/SWARM/crazys/rotors_gazebo/resource/crazyflie2_mpc1_params1.yaml"

pushd $HW_LOG_DIR
if test -e State0.csv
then
  startdate=`head -n 1 State0.csv | cut -d "," -f 1`
  echo "-- File State0.csv exists, starts at $startdate."
  mkdir -p ../$startdate

  mv *.csv ../$startdate/.
#  cp -arv *.csv ../$startdate/.

  pushd $HW_LOG_DIR/../$startdate
  cp -arv $MAKEFILE_SRC .
  cp -arv $PARAMS_SRC .
  make all-distance
  make all-metrics
  make all-state
  make all-trace
  cat *.yaml
  popd
else
  echo "-- File State0.csv does not exist."
fi;
popd
