cmake_minimum_required(VERSION 3.5)
project(kalman_filter_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(kalman_filter_node src/kalman_filter_node.cpp)

ament_target_dependencies(
  kalman_filter_node
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  kalman_filter_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

