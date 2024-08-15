#!/bin/bash


echo "Run ros2 setup"
source /opt/ros/foxy/setup.bash

echo "Run rviz2"
ros2 run rviz2 rviz2 &
sleep 1

ros2 bag play /resources/data/rosbag2_2024_08_13-19_13_31_0.db3



