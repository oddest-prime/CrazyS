#!/bin/bash

date_hash="$1"

rm /tmp/info.png -rf
convert -page 3440x2160+150+100 -pointsize 50 -font Courier text:/tmp/video.info /tmp/info.png
if test -e /tmp/info-0.png
then
  rm /tmp/info-*.png -rf
  convert -page 3440x2160+150+100 -pointsize 40 -font Courier text:/tmp/video.info /tmp/info.png
  if test -e /tmp/info-0.png
  then
    rm /tmp/info-*.png -rf
    convert -page 3440x2160+150+100 -pointsize 30 -font Courier text:/tmp/video.info /tmp/info.png
    if test -e /tmp/info-0.png
    then
      rm /tmp/info-*.png -rf
      convert -page 3440x2160+150+100 -pointsize 25 -font Courier text:/tmp/video.info /tmp/info.png
      if test -e /tmp/info-0.png
      then
        rm /tmp/info-*.png -rf
        convert -page 3440x2160+150+100 -pointsize 20 -font Courier text:/tmp/video.info /tmp/info.png
        if test -e /tmp/info-0.png
        then
          rm /tmp/info-*.png -rf
          convert -page 3440x2160+150+100 -pointsize 15 -font Courier text:/tmp/video.info /tmp/info.png
        fi
      fi
    fi
  fi
fi

ffmpeg -r 15 -pattern_type glob -i "/tmp/cam1/default_camera1_link_*.jpg" -c:v libx264 /tmp/my_cam1.mp4 #      side view from X-axis
ffmpeg -r 15 -pattern_type glob -i "/tmp/cam2/default_camera2_link_*.jpg" -c:v libx264 /tmp/my_cam2.mp4 #      top view at center
ffmpeg -r 15 -pattern_type glob -i "/tmp/cam3/default_camera3_link_*.jpg" -c:v libx264 /tmp/my_cam3.mp4 #      side view from Y-axis
ffmpeg -r 15 -pattern_type glob -i "/tmp/cam4/default_camera4_link_*.jpg" -c:v libx264 /tmp/my_cam4.mp4 #      inclined diagonal view
test -e /tmp/cam5 && ffmpeg -r 15 -pattern_type glob -i "/tmp/cam5/default_camera5_link_*.jpg" -c:v libx264 /tmp/my_cam5.mp4 || cp /tmp/my_cam4.mp4 /tmp/my_cam5.mp4 #      top view at obstacle

#ffmpeg -i /tmp/my_cam1.mp4 -i /tmp/my_cam3.mp4 -filter_complex "[0:v]crop=1719:1080, pad=1720:1080:0:0:black[tmp0]; [1:v]crop=1719:1080, pad=1720:1080:1:0:black[tmp1]; [tmp0][tmp1]hstack[v] " -map [v] -y /tmp/my_out1.mp4
#ffmpeg -i /tmp/my_cam2.mp4 -i /tmp/my_cam4.mp4 -filter_complex "[0:v]crop=1719:1080, pad=1720:1080:0:0:black[tmp0]; [1:v]crop=1719:1080, pad=1720:1080:1:0:black[tmp1]; [tmp0][tmp1]hstack[v] " -map [v] -y /tmp/my_out2.mp4
#ffmpeg -i /tmp/my_out2.mp4 -i /tmp/my_out1.mp4 -filter_complex "[0:v]crop=3440:1079, pad=3440:1080:0:0:black[tmp0]; [1:v]crop=3440:1079, pad=3440:1080:1:0:black[tmp1]; [tmp0][tmp1]vstack[v] " -map [v] -y /tmp/my_out.mp4

# /tmp/my_cam1.mp4 - links oben
# /tmp/my_cam5.mp4 - rechts oben
# /tmp/my_cam2.mp4 - links unten
# /tmp/my_cam4.mp4 - rechts unten

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

#mv /tmp/my_cam1.mp4 /crazyflie_ws/src/crazys/video_${date_hash}_cam1.mp4
#mv /tmp/my_cam2.mp4 /crazyflie_ws/src/crazys/video_${date_hash}_cam2.mp4
#mv /tmp/my_cam3.mp4 /crazyflie_ws/src/crazys/video_${date_hash}_cam3.mp4
#mv /tmp/my_cam4.mp4 /crazyflie_ws/src/crazys/video_${date_hash}_cam4.mp4

rm -f /tmp/my_*.mp4
#rm -rf /tmp/cam*/*.jpg
