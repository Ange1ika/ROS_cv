#!/bin/bash

source /opt/ros/noetic/setup.bash
source /sources/catkin_ws/devel/setup.bash

# Bag 1
#export FIXED_FRAME=base_link

#export FIXED_FRAME=realsense_gripper_link
export TARGET_BAG=сoutput.bag
#export RVIZ_CONF=/resources/data/testbag_full.rviz  

export IMAGE_TOPIC=/camera/camera/color/image_raw/compressed
export DEPTH_TOPIC=/camera/camera/depth/image_rect_raw/compressedDepth
export INFO_TOPIC=/camera/camera/color/camera_info
export DEPTH_INFO=/camera/camera/depth/camera_info

#camera_color_optical_frame
#camera_depth_optical_frame
#head_link

export RVIZ_CONF=/resources/data/rviz_full_conf_2.rviz
#export DEPTH_TOPIC=/realsense_gripper/aligned_depth_to_color/image_raw  
#export IMAGE_TOPIC=/realsense_gripper/color/image_raw/compressed
#export IMAGE_TOPIC=/realsense_gripper/color/image_raw
export DEPTH_TO_POINT_CLOUD_ALLOW_PYTHON_IMPLEMENTATION=1


#export INFO_TOPIC=/cam2/zed_node_1/left/camera_info
#export IMAGE_TOPIC=/cam2/zed_node_1/left/image_rect_color/compressed  #топик на ros2
#export DEPTH_TOPIC=/cam2/zed_node_1/depth/depth_registered
#export DEPTH_INFO=/cam2/zed_node_1/depth/camera_info



#export INFO_TOPIC=/camera2/camera2/depth/camera_info
#export IMAGE_TOPIC=/camera2/camera2/color/image_raw/compressed  #топик на ros2
#export DEPTH_TOPIC=/camera2/camera2/depth/image_rect_raw
#export DEPTH_INFO=/camera2/camera2/depth/camera_info


#export FIXED_FRAME=camera2_color_optical_frame # ros2

echo "Run roscore"
roscore &
sleep 1

#echo "Run bag play"
#rosbag play $TARGET_BAG -i
#rosbad play -l /resources/data/bags.txt
#sleep 1

#echo "Run aruco_node.py"
#python /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/aruco_detection_service.py &
#sleep 1

#echo "Run point_cloud_listener.py"
#python /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/point_cloud_listener.py &
#sleep 1

#rosbag play $TARGET_BAG -r 0.5 -s 5 --pause
# rosbag play $TARGET_BAG -r 0.5 -s 5 -u 5
exec /bin/bash

