#!/bin/bash

echo "Run ros2 setup"
source /opt/ros/foxy/setup.bash

echo "Run rviz2"
ros2 run rviz2 rviz2 &
sleep 1

ros2 bag play /resources/data/rosbag2_26.db3

for node in $(ros2 node list); do
  ros2 param set $node use_sim_time true
done
