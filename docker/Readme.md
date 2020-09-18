	docker build . --tag crazys

	docker run -it crazys
	
	docker run -it --device /dev/dri/ --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" crazys
	
	export containerId=$(docker ps -l -q)
	xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId`
	
	roslaunch rotors_gazebo crazyflie2_hovering_example.launch
	
