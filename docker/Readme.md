# Docker container for crazys swarm simulation

## Build container
    cd docker
	docker build . --tag crazys-focal-noetic

## Run container
_(make sure to change to main directory of this repository first)_

	docker run --rm -it crazys crazys-focal-noetic

### Run container with GUI on host machine
_(make sure to change to main directory of this repository first)_

	docker run --rm -it --device /dev/dri/ --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "$(pwd)":/crazyflie_ws/src/CrazyS crazys-focal-noetic

## Build
For initial build use the following alias:

	crazys_init

Subseqent builds done with catkin:

    catkin build

## Start simulation examples

### Hovering
	roslaunch rotors_gazebo crazyflie2_hovering_example.launch

### Simple path of waypoints
	roslaunch rotors_gazebo crazyflie2_waypoint_example.launch

## Notes
    rostopic pub /crazyflie2/command/trajectory trajectory_msgs/MultiDOFJointTrajectory '{points: [transforms:[rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}, translation: {x: 1.0, y: 0.0, z: 1.0}]]}'
