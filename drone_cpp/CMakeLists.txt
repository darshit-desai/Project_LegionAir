cmake_minimum_required(VERSION 3.8)
project(drone_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drone_swarm_msgs REQUIRED)

#--------------------------------
# C++
#--------------------------------
# add include directories
install(DIRECTORY
  include
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

include_directories(include)

# set executables
add_executable(drone_node_cpp src/drone_node.cpp)

# add dependencies
ament_target_dependencies(drone_node_cpp rclcpp std_msgs geometry_msgs drone_swarm_msgs)

# install executables to lib/${PROJECT_NAME}
install(TARGETS
  drone_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

