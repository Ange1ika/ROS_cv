cmake_minimum_required(VERSION 3.5)
project(husky_tidy_bot_cv)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Message and service generation
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectArucoMarker.srv"
  "srv/Reset.srv"
  "srv/SetObjectId.srv"
  "srv/GetObjectPointCloud.srv"
  "srv/GetObjectPose.srv"
  "msg/Box.msg"
  "msg/Roi.msg"
  "msg/Mask.msg"
  "msg/Objects.msg"
  "msg/Objects3d.msg"
  "msg/ObjectPointCloud.msg"
  "msg/Categories.msg"
  "msg/ObjectPose.msg"
  "msg/Marker.msg"
  "msg/Detection.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
)

# Install Python nodes
ament_python_install_package(scripts)


# Install scripts
install(PROGRAMS
  scripts/aruco_detection_service.py
  scripts/aruco_node.py
  scripts/bot_sort.py
  scripts/bot_sort_node.py
  scripts/byte_track.py
  scripts/combine_data.py
  scripts/conversions.py
  scripts/data_aggregator.py
  scripts/fixed_cats.py
  scripts/object_point_cloud_extraction.py
  scripts/object_point_cloud_extraction_node.py
  scripts/object_pose_estimation.py
  scripts/object_pose_estimation_node.py
  scripts/openseed_model.py
  scripts/openseed_node.py
  scripts/plot_logs.py
  scripts/text_query_generation_client.py
  scripts/text_query_generation_server.py
  scripts/tracker_3d.py
  scripts/tracker_3d_node.py
  scripts/visualize_objects_point_cloud.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
#)

ament_package()
