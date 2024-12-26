cmake_minimum_required(VERSION 3.5)
project(your_tf_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(tf_publisher_cpp src/tf_publisher_cpp_node.cpp)
ament_target_dependencies(tf_publisher_cpp rclcpp tf2_ros)

## Specify libraries to link a library or executable target against
target_link_libraries(tf_publisher_cpp
  ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS
  tf_publisher_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
