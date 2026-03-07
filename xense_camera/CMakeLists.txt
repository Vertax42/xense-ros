cmake_minimum_required(VERSION 3.8)
project(xense_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Make the bundled cmake/ directory visible so find_package(xensesdk) resolves
# to our xensesdkConfig.cmake wrapper (the SDK ships only Targets, no Config).
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(xense_msgs REQUIRED)

# libxensesdk public dependencies (must be found before xensesdk targets are imported)
find_package(spdlog REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc video imgcodecs)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

find_package(xensesdk REQUIRED)

add_executable(xense_camera_node
  src/xense_camera_node_main.cpp
  src/xense_camera_node.cpp
  src/ros_utils.cpp
)

target_include_directories(xense_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include
)

target_compile_features(xense_camera_node PUBLIC cxx_std_17)

ament_target_dependencies(xense_camera_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf2_ros
  xense_msgs
)

target_link_libraries(xense_camera_node
  xense::xensesdk
)

install(TARGETS xense_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
