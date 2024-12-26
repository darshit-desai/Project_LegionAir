cmake_minimum_required(VERSION 3.5)
project(tflite_prop_detection)

# Default to C++17 if not specified
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Check if C++ version is less than 17 and set it to 14 if true
if(CMAKE_CXX_STANDARD LESS 17)
  message(WARNING "C++ standard less than 17 detected, setting to C++14")
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(voxl_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED)

# Include directories
include_directories(
  ${ament_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(tflite_prop_detection_cpp src/tflite_prop_detection_cpp_node.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(tflite_prop_detection_cpp
  rclcpp
  sensor_msgs
  voxl_msgs
  image_transport
  pcl_conversions
  pcl_msgs
  PCL
)

# Install targets and scripts for ROS package
install(TARGETS
  tflite_prop_detection_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
