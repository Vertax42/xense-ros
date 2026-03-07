cmake_minimum_required(VERSION 3.8)
project(xense_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ForceVector.msg"
  "msg/ForceArray.msg"
  "msg/MarkerArray2D.msg"
  "msg/MarkerArray3D.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
