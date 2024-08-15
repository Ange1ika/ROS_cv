#!/bin/bash

echo "Run ros_bridge"

source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
 

export ROS_MASTER_URI=http://localhost:11311

# Запустите rosbridge
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
