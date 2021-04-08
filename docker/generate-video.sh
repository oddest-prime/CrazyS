#!/bin/bash

hash="$1"

ffmpeg -r 15 -pattern_type glob -i "/tmp/cam1/default_camera1_link_*.jpg" -c:v libx264 /tmp/my_cam1.mp4
ffmpeg -r 15 -pattern_type glob -i "/tmp/cam2/default_camera2_link_*.jpg" -c:v libx264 /tmp/my_cam2.mp4
ffmpeg -r 15 -pattern_type glob -i "/tmp/cam3/default_camera3_link_*.jpg" -c:v libx264 /tmp/my_cam3.mp4
ffmpeg -r 15 -pattern_type glob -i "/tmp/cam4/default_camera4_link_*.jpg" -c:v libx264 /tmp/my_cam4.mp4

ffmpeg -i /tmp/my_cam1.mp4 -i /tmp/my_cam3.mp4 -filter_complex "[0:v]crop=1719:1080, pad=1720:1080:0:0:black[tmp0]; [1:v]crop=1719:1080, pad=1720:1080:1:0:black[tmp1]; [tmp0][tmp1]hstack[v] " -map [v] -y /tmp/my_out1.mp4
#ffmpeg -i /tmp/my_cam1.mp4 -i /tmp/my_cam3.mp4 -filter_complex hstack /tmp/my_out1.mp4
ffmpeg -i /tmp/my_cam2.mp4 -i /tmp/my_cam4.mp4 -filter_complex "[0:v]crop=1719:1080, pad=1720:1080:0:0:black[tmp0]; [1:v]crop=1719:1080, pad=1720:1080:1:0:black[tmp1]; [tmp0][tmp1]hstack[v] " -map [v] -y /tmp/my_out2.mp4
#ffmpeg -i /tmp/my_cam2.mp4 -i /tmp/my_cam4.mp4 -filter_complex hstack /tmp/my_out2.mp4
ffmpeg -i /tmp/my_out2.mp4 -i /tmp/my_out1.mp4 -filter_complex "[0:v]crop=3440:1079, pad=3440:1080:0:0:black[tmp0]; [1:v]crop=3440:1079, pad=3440:1080:1:0:black[tmp1]; [tmp0][tmp1]vstack[v] " -map [v] -y /tmp/my_out.mp4
#ffmpeg -i /tmp/my_out2.mp4 -i /tmp/my_out1.mp4 -filter_complex vstack /tmp/my_out.mp4

mv /tmp/my_out.mp4 /crazyflie_ws/src/crazys/video_`date +%Y-%m-%d_%H-%M-%S`_${hash}.mp4
#mv /tmp/my_cam4.mp4 /crazyflie_ws/src/crazys/video_`date +%Y-%m-%d_%H-%M-%S`_cam4.mp4

rm -f /tmp/my_*.mp4
rm -rf /tmp/cam*
