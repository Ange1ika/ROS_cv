#!/bin/bash

source /opt/ros/noetic/setup.bash
source /sources/catkin_ws/devel/setup.bash

# Bag 1
#export FIXED_FRAME=base_link
export FIXED_FRAME=camera2_color_optical_frame # ros2
#export FIXED_FRAME=realsense_gripper_link
export TARGET_BAG=сoutput.bag
#export RVIZ_CONF=/resources/data/testbag_full.rviz  
export DEPTH_INFO=/camera2/camera2/depth/camera_info
export RVIZ_CONF=/resources/data/rviz_full_conf_2.rviz
#export DEPTH_TOPIC=/realsense_gripper/aligned_depth_to_color/image_raw  

export DEPTH_TOPIC=/camera2/camera2/depth/image_rect_raw

# /camera2/camera2/color/image_raw/compressedDepth  #топик на ros2
#export DEPTH_TOPIC=/realsense_gripper/aligned_depth_to_color/image_raw
export IMAGE_TOPIC=/camera2/camera2/color/image_raw/compressed  #топик на ros2
export INFO_TOPIC=/camera2/camera2/depth/camera_info
#export IMAGE_TOPIC=/realsense_gripper/color/image_raw/compressed


#export IMAGE_TOPIC=/realsense_gripper/color/image_raw
export DEPTH_TO_POINT_CLOUD_ALLOW_PYTHON_IMPLEMENTATION=1

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





echo "Run bot_sort_node.py"
python /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/bot_sort_node.py -vis &
sleep 1

echo "Run object_point_cloud_extraction_node.py"
python /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/object_point_cloud_extraction_node.py -vis &
sleep 1


echo "Run object_pose_estimation_node.py" 
python /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/object_pose_estimation_node.py -vis &
sleep 1

echo "Run tracker_3d_node.py"
python /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/tracker_3d_node.py -vis &
sleep 1



#echo "Run aruco_node.py"
#python /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/aruco_node.py &
#sleep 1

#echo "Run aruco_seg_fusion"
#python /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/combine_data.py &
#sleep 1


# Ожидание публикации изображения с метками ArUco
#while :
#do
#    if rostopic list -v | grep -q "/aruco_image \[sensor_msgs/Image\] 1 publisher"; then
#        echo "Found /aruco_image with 1 publisher"
#        break
#    else
#        echo "Waiting for /aruco_image..."
#        sleep 1
#    fi
#done


#echo "Run rviz"
#rosrun rviz rviz -d $RVIZ_CONF &
#sleep 1


python /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/text_query_generation_server.py &
while :
do
    if rostopic list -v | grep -q "/segmentation_labels \[husky_tidy_bot_cv/Categories\] 1 publisher"; then
        echo "Found /segmentation_labels with 1 publisher"
        break
    else
        echo "Waiting for /segmentation_labels..."
        sleep 1
    fi
done


python /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/openseed_node.py -vis &
while :
do
    if rostopic list -v | grep -q "/segmentation_vis \[sensor_msgs/Image\] 1 publisher"; then
        echo "Found /segmentation_vis with 1 publisher"
        break
    else
        echo "Waiting for /segmentation_vis..."
        sleep 1
    fi
done

#rosbag play $TARGET_BAG -r 0.5 -s 5 --pause
# rosbag play $TARGET_BAG -r 0.5 -s 5 -u 5
exec /bin/bash

