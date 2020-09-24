	docker build . --tag crazys

	docker run -it crazys
	
	docker run -it --device /dev/dri/ --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "$(pwd)":/crazyflie_ws/src/crazys crazys
	
	export containerId=$(docker ps -l -q)
	xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId`
	
	roslaunch rotors_gazebo crazyflie2_hovering_example.launch
	
	ffmpeg -r 30 -pattern_type glob -i "/tmp/cam1/default_camera_link_*.jpg" -c:v libx264 my_camera.mp4

