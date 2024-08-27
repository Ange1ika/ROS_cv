cmake_minimum_required(VERSION 3.5)
project(husky_tidy_bot_cv)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Для поддержки сообщений и сервисов
find_package(rosidl_default_generators REQUIRED)

# Указываем зависимые пакеты
set(msg_deps
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
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
  "srv/Reset.srv"
  "srv/SetObjectId.srv"
  "srv/GetObjectPointCloud.srv"
  "srv/GetObjectPose.srv"
  "srv/DetectArucoMarker.srv"
  DEPENDENCIES ${msg_deps}
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

