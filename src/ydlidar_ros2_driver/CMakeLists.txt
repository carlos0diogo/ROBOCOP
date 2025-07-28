cmake_minimum_required(VERSION 3.5)
project(ydlidar_ros2_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

# Placeholder executables (reais precisariam do SDK do YDLiDAR)
add_executable(ydlidar_ros2_driver_node src/placeholder_node.cpp)
ament_target_dependencies(ydlidar_ros2_driver_node
  rclcpp
  sensor_msgs
  std_srvs
)

# Install
install(TARGETS
  ydlidar_ros2_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
