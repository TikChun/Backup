#!/bin/bash

source /opt/ros/noetic/setup.bash

roslaunch realsense2_camera rs_camera.launch --wait &
 
command1_pid=$!

rosbag record /camera/image_raw -o exp.bag

kill command1_pid