#!/bin/bash

ffmpeg -r 15 -pattern_type glob -i "/tmp/cam1/default_camera1_link_*.jpg" -c:v libx264 /tmp/my_cam1.mp4
ffmpeg -r 15 -pattern_type glob -i "/tmp/cam2/default_camera2_link_*.jpg" -c:v libx264 /tmp/my_cam2.mp4
ffmpeg -r 15 -pattern_type glob -i "/tmp/cam3/default_camera3_link_*.jpg" -c:v libx264 /tmp/my_cam3.mp4
ffmpeg -r 15 -pattern_type glob -i "/tmp/cam4/default_camera4_link_*.jpg" -c:v libx264 /tmp/my_cam4.mp4

ffmpeg -i /tmp/my_cam1.mp4 -i /tmp/my_cam3.mp4 -filter_complex hstack /tmp/my_out1.mp4
ffmpeg -i /tmp/my_cam2.mp4 -i /tmp/my_cam4.mp4 -filter_complex hstack /tmp/my_out2.mp4
ffmpeg -i /tmp/my_out2.mp4 -i /tmp/my_out1.mp4 -filter_complex vstack /tmp/my_out.mp4

mv /tmp/my_out.mp4 /crazyflie_ws/src/crazys/video_`date +%Y-%m-%d_%H-%M-%S`.mp4

rm -f /tmp/my_*.mp4
rm -rf /tmp/cam*
