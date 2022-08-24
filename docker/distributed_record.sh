#!/bin/bash

dpart=`date +%Y-%m-%d_%H-%M-%S`
fpart=`echo "${dpart}_${1}"`

ssh andreas@rpi1 "killall ffmpeg -s USR1"
ssh andreas@rpi2 "killall ffmpeg -s USR1"
ssh andreas@rpi1 "rm -f ./out.mjpeg.mkv"
ssh andreas@rpi2 "rm -f ./out.mjpeg.mkv"

echo "starting recording at rpi1."
ssh andreas@rpi1 "ffmpeg -y -f v4l2 -framerate 30 -video_size 960x540 -input_format mjpeg -i /dev/video0 -c copy out.mjpeg.mkv 2> /dev/null" &
echo "started recording at rpi1."

echo "starting recording at rpi2."
ssh andreas@rpi2 "ffmpeg -y -f v4l2 -framerate 30 -video_size 960x540 -input_format mjpeg -i /dev/video0 -c copy out.mjpeg.mkv 2> /dev/null" &
echo "started recording at rpi2."

echo "reading input (enter something and press ENTER to stop recording)."
read -r input
echo "done reading input."

#sleep 10

ssh andreas@rpi1 "killall ffmpeg -s USR1"
ssh andreas@rpi2 "killall ffmpeg -s USR1"

scp andreas@rpi1:./out.mjpeg.mkv /home/andreas/SWARM/telesto_logs/videos/recording_${fpart}_rpi1cam.mkv
ssh andreas@rpi1 "rm -f ./out.mjpeg.mkv"

scp andreas@rpi2:./out.mjpeg.mkv /home/andreas/SWARM/telesto_logs/videos/recording_${fpart}_rpi2cam.mkv
ssh andreas@rpi2 "rm -f ./out.mjpeg.mkv"
