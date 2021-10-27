#!/bin/bash


set -e


cd ~/robotcar_ws/script
echo -e "[INFO] ALL 8 cameras started !"
echo -e "[INFO] Please RUN rqt OR rqt_image_view to see the camera images."
./camera_enable_ros.sh 

exit 0
