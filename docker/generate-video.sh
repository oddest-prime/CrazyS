#!/bin/bash

date_hash="$1"

rm /tmp/info.png -rf
convert -page 3440x2160+150+100 -pointsize 50 -font Courier text:/tmp/video.info /tmp/info.png

ffmpeg -r 15 -pattern_type glob -i "/tmp/cam1/default_camera1_link_*.jpg" -c:v libx264 /tmp/my_cam1.mp4
ffmpeg -r 15 -pattern_type glob -i "/tmp/cam2/default_camera2_link_*.jpg" -c:v libx264 /tmp/my_cam2.mp4
ffmpeg -r 15 -pattern_type glob -i "/tmp/cam3/default_camera3_link_*.jpg" -c:v libx264 /tmp/my_cam3.mp4
ffmpeg -r 15 -pattern_type glob -i "/tmp/cam4/default_camera4_link_*.jpg" -c:v libx264 /tmp/my_cam4.mp4

#ffmpeg -i /tmp/my_cam1.mp4 -i /tmp/my_cam3.mp4 -filter_complex "[0:v]crop=1719:1080, pad=1720:1080:0:0:black[tmp0]; [1:v]crop=1719:1080, pad=1720:1080:1:0:black[tmp1]; [tmp0][tmp1]hstack[v] " -map [v] -y /tmp/my_out1.mp4
#ffmpeg -i /tmp/my_cam2.mp4 -i /tmp/my_cam4.mp4 -filter_complex "[0:v]crop=1719:1080, pad=1720:1080:0:0:black[tmp0]; [1:v]crop=1719:1080, pad=1720:1080:1:0:black[tmp1]; [tmp0][tmp1]hstack[v] " -map [v] -y /tmp/my_out2.mp4
#ffmpeg -i /tmp/my_out2.mp4 -i /tmp/my_out1.mp4 -filter_complex "[0:v]crop=3440:1079, pad=3440:1080:0:0:black[tmp0]; [1:v]crop=3440:1079, pad=3440:1080:1:0:black[tmp1]; [tmp0][tmp1]vstack[v] " -map [v] -y /tmp/my_out.mp4

ffmpeg \
-i /tmp/my_cam1.mp4 \
-i /tmp/my_cam3.mp4 \
-i /tmp/my_cam2.mp4 \
-i /tmp/my_cam4.mp4 \
-loop 1 -t 3 -i /tmp/info.png \
-filter_complex "\
[4:v]fade=t=out:st=4:d=0.1[c0]; \
[0:v]crop=1718:1080, pad=1720:1080:0:0:black[tmp0]; \
[2:v]crop=1718:1080, pad=1720:1080:0:0:black[tmp2]; \
[tmp0][1:v]hstack[v0]; \
[tmp2][3:v]hstack[v1]; \
[v0]crop=3440:1078, pad=3440:1080:0:0:black[vv0]; \
[vv0][v1]vstack[vv]; \
[c0][vv]concat=n=2:v=1:a=0,format=yuv420p[v]" \
 -map [v] -y /tmp/my_out.mp4

mv /tmp/my_out.mp4 /crazyflie_ws/src/crazys/video_${date_hash}.mp4
#mv /tmp/my_cam4.mp4 /crazyflie_ws/src/crazys/video_`date +%Y-%m-%d_%H-%M-%S`_cam4.mp4

rm -f /tmp/my_*.mp4
#rm -rf /tmp/cam*/*.jpg
