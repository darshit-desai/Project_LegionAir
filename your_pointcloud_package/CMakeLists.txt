cmake_minimum_required(VERSION 3.5)
project(your_pointcloud_package)

if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-Wstack-usage=2800000)  # Set the desired stack size (e.g., 16384 KB)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

# Uncomment if you have any Python scripts to install
# ament_python_install_package(${PROJECT_NAME})

# Declare the C++ executable
add_executable(pointcloud_transformer src/pc_transform.cpp)
ament_target_dependencies(pointcloud_transformer
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  PCL
  Eigen3
  pcl_conversions
  pcl_msgs
)

# Specify include directories
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Specify libraries to link against
target_link_libraries(pointcloud_transformer
  ${PCL_LIBRARIES}
)

install(TARGETS
  pointcloud_transformer
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install package.xml
ament_package()
