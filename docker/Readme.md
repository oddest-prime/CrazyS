# Docker container for crazys swarm simulation

## Build container
    cd docker
	docker build . --tag crazys_focal_noetic

## Run container
	docker run --rm -it crazys crazys_focal_noetic

### Run container with GUI on host machine
	docker run --rm -it --device /dev/dri/ --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "$(pwd)":/crazyflie_ws/src/CrazyS crazys_focal_noetic

## Build
For initial build use the following alias:

	crazys_init

Subseqent builds done with catkin:

    catkin build

## Start hovering simulation example
	roslaunch rotors_gazebo crazyflie2_hovering_example.launch

  roslaunch rotors_gazebo crazyflie2_waypoint_example.launch

## Notes
    rostopic pub /crazyflie2/command/trajectory trajectory_msgs/MultiDOFJointTrajectory '{points: [transforms:[rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}, translation: {x: 1.0, y: 0.0, z: 1.0}]]}'
