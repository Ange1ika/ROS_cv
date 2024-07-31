#!/bin/bash

python3 ../docker_ros_bridge/scripts/run_ros_bridge.py --topics /realsense_gripper/color/image_raw/compressed /zed_node/left/image_rect_color/compressed /segmentation/realsense /segmentation/zed_node /tracking_vis/realsense /tracking_vis/zed_node /object_point_cloud_vis --topic-types 'sensor_msgs/msg/CompressedImage' 'sensor_msgs/msg/CompressedImage' 'tidy_bot_cv_msgs/msg/Objects' 'tidy_bot_cv_msgs/msg/Objects' 'sensor_msgs/msg/Image' 'sensor_msgs/msg/Image' 'sensor_msgs/msg/PointCloud2'
