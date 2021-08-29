# Docker container for crazys swarm simulation

## Build container
	docker build . --tag crazys

## Run container without GUI and render video
	docker run --rm --volume "$(pwd)":/crazyflie_ws/src/crazys crazys /crazyflie_ws/src/crazys/docker/run-simulation.sh `git rev-parse --short HEAD`

## Run container
	docker run -it crazys

## Run container with GUI on host machine
	docker run -it --device /dev/dri/ --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "$(pwd)":/crazyflie_ws/src/crazys crazys

	cd ~/SWARM && docker run -it --device /dev/dri/ --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --device=/dev/input/js0 --device /dev/bus/usb/ --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "$(pwd)/crazys":/crazyflie_ws/src/crazys -v "$(pwd)/lps-ros":/crazyflie_ws/src/lps-ros -v "$(pwd)/crazyflie_ros":/crazyflie_ws/src/crazyflie_ros crazys

### Start simulation with GUI
	roslaunch rotors_gazebo crazyflie2_swarm2.launch gui:=true swarm_params:=mpc1_params2
	roslaunch rotors_gazebo crazyflie2_swarm2.launch gui:=true swarm_mode:=mpc1 swarm_params:=mpc1_params2
	roslaunch rotors_gazebo crazyflie2_swarm4.launch gui:=true
	roslaunch rotors_gazebo crazyflie2_swarm9.launch gui:=true swarm_params:=mpc1_params2

  roslaunch rotors_gazebo crazyflie2_swarm2.launch gui:=true swarm_mode:=gradient swarm_params:=mpc1_params1

### Start with hardware
  roslaunch crazyflie_demo teleop_xbox360.launch

/dev/bus/usb

## Other notes
	export containerId=$(docker ps -l -q)
	xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId`

	roslaunch rotors_gazebo crazyflie2_hovering_example.launch gui:=false

### Record step-response of inner loop controller
	roslaunch rotors_gazebo crazyflie2_step_response.launch gui:=true

	rm -f *.csv && rm /tmp/cam* -rf && roslaunch rotors_gazebo crazyflie2_hovering_two.launch

	ffmpeg -r 30 -pattern_type glob -i "/tmp/cam1/default_camera1_link_*.jpg" -c:v libx264 my_cam1.mp4
	ffmpeg -r 30 -pattern_type glob -i "/tmp/cam2/default_camera2_link_*.jpg" -c:v libx264 my_cam2.mp4
	ffmpeg -r 30 -pattern_type glob -i "/tmp/cam3/default_camera3_link_*.jpg" -c:v libx264 my_cam3.mp4
	ffmpeg -r 30 -pattern_type glob -i "/tmp/cam4/default_camera4_link_*.jpg" -c:v libx264 my_cam4.mp4

	ffmpeg -i my_cam1.mp4 -i my_cam3.mp4 -filter_complex hstack my_out1.mp4
	ffmpeg -i my_cam2.mp4 -i my_cam4.mp4 -filter_complex hstack my_out2.mp4
	ffmpeg -i my_out1.mp4 -i my_out2.mp4 -filter_complex vstack my_out.mp4
