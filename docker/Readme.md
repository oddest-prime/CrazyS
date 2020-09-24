	docker build . --tag crazys

	docker run -it crazys
	
	docker run -it --device /dev/dri/ --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "$(pwd)":/crazyflie_ws/src/crazys crazys
	
	export containerId=$(docker ps -l -q)
	xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId`
	
	roslaunch rotors_gazebo crazyflie2_hovering_example.launch
	
	ffmpeg -r 30 -pattern_type glob -i "/tmp/cam1/default_camera1_link_*.jpg" -c:v libx264 my_cam1.mp4
	ffmpeg -r 30 -pattern_type glob -i "/tmp/cam2/default_camera2_link_*.jpg" -c:v libx264 my_cam2.mp4
	
	ffmpeg -r 30 -pattern_type glob -i "/tmp/cam1/default_camera1_link_*.jpg" -i "/tmp/cam2/default_camera2_link_*.jpg" -filter_complex '[0:v]pad=iw*2:ih[int];[int][1:v]overlay=W/2:0[vid]' -map [vid] -c:v libx264 my_cam_x.mp4
	
	ffmpeg -i my_cam1.mp4 -i my_cam2.mp4 -filter_complex hstack output.mp4
