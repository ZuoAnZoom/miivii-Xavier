#!/bin/bash

set -e

fps_A=$(cat ~/robotcar_ws/src/miivii_gmsl_camera/miivii_gmsl_ros/launch/1_node_with_4_cameras.launch | grep fps)
fps_B=$(cat ~/robotcar_ws/src/miivii_gmsl_camera/miivii_gmsl_ros/launch/2_node_with_4_cameras.launch | grep fps)

echo -e "[INFO] Group_A cameras info are below:"
echo -e "[INFO]   Camera 1: front Left"
echo -e "[INFO]   Camera 2: front Right"
echo -e "[INFO]   Camera 3: front Fish eye"
echo -e "[INFO]   Camera 4: back  Fish eye"
echo -e "[INFO]$fps_A"
echo -e "[INFO] Group_B cameras info are below:"
echo -e "[INFO]$fps_B"
echo -e "[INFO] Please MAKE SURE you don't need to change the camera FPS !"


cd ~/workspace
echo -e "[INFO] cameras image will saved at $PWD"

echo -e "[INFO] Which Group of cameras do you want to Record? Please input [A/B/AB]"
read group
if [[ "$group" == "A" ]]; then
	echo -e "[INFO] Will record Group A. Camera 1-4."
	rosbag record /miivii_gmsl_ros_A/camera1 /miivii_gmsl_ros_A/camera2 /miivii_gmsl_ros_A/camera3 /miivii_gmsl_ros_A/camera4
elif [[ "$group" == "B" ]]; then
	echo -e "[INFO] Will record Group B. Camera 5-8."
	rosbag record /miivii_gmsl_ros_B/camera1 /miivii_gmsl_ros_B/camera2 /miivii_gmsl_ros_B/camera3 /miivii_gmsl_ros_B/camera4
elif [[ "$group" == "AB" ]]; then
	echo -e "[INFO] Will record All 8 cameras."
	rosbag record /miivii_gmsl_ros_A/camera1 /miivii_gmsl_ros_A/camera2 /miivii_gmsl_ros_A/camera3 /miivii_gmsl_ros_A/camera4 /miivii_gmsl_ros_B/camera1 /miivii_gmsl_ros_B/camera2 /miivii_gmsl_ros_B/camera3 /miivii_gmsl_ros_B/camera4
else 
	echo -e "[EROR] Not [A/B/AB], record stopped, no rosbag saved." 
fi


exit 0
