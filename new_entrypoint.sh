#!/bin/bash

source /opt/ros/noetic/setup.bash
source /sources/catkin_ws/devel/setup.bash

export FIXED_FRAME=base_link
export TARGET_BAG=/resources/data/output.bag
export RVIZ_CONF=/resources/data/testbag_full.rviz
export DEPTH_TOPIC=/realsense_gripper/aligned_depth_to_color/image_raw
export IMAGE_TOPIC=/realsense_gripper/color/image_raw/compressed
export DEPTH_TO_POINT_CLOUD_ALLOW_PYTHON_IMPLEMENTATION=1

echo "Run roscore"
roscore &
sleep 1

echo "Run bag play"
rosbag play $TARGET_BAG -i &
sleep 1

echo "Run bot_sort_node.py"
python /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/bot_sort_node.py -vis &
sleep 1

echo "Run tracker_3d_node.py"
python /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/tracker_3d_node.py -vis &
sleep 1

echo "Run object_point_cloud_extraction_node.py"
python /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/object_point_cloud_extraction_node.py -vis &
sleep 1

echo "Run object_pose_estimation_node.py"
python /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/object_pose_estimation_node.py -vis &
sleep 1

echo "Run data_aggregator_node.py"
python /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/data_aggregator_node.py &
sleep 1

echo "Run rviz"
rosrun rviz rviz -d $RVIZ_CONF &
sleep 1

python /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/text_query_generation_server.py &
sleep 1

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

# Function to call services
call_services() {
    OBJECT_ID=$1
    rosservice call /object_point_cloud_extraction/set_object_id $OBJECT_ID
    rosservice call /object_pose_estimation/get_object_pose $OBJECT_ID
}

# Monitor tracked objects and call services for new ones
monitor_tracked_objects() {
    PREVIOUS_IDS=()
    while :
    do
        CURRENT_IDS=$(rostopic echo -n 1 /tracked_objects_3d_vis | grep "tracking_id" | awk '{print $2}')
        for ID in $CURRENT_IDS; do
            if [[ ! " ${PREVIOUS_IDS[@]} " =~ " ${ID} " ]]; then
                echo "New object detected with ID: $ID"
                call_services $ID
                PREVIOUS_IDS+=($ID)
            fi
        done
        sleep 1
    done
}

monitor_tracked_objects &

rosbag play $TARGET_BAG -r 0.5 -s 5

exec /bin/bash

