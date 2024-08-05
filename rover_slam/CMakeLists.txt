cmake_minimum_required(VERSION 3.8)
project(rover_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Create executable
add_executable(rover_slam src/rover_slam.cpp src/rover_slam_node.cpp)
ament_target_dependencies(rover_slam rclcpp sensor_msgs cv_bridge OpenCV)
target_include_directories(rover_slam PRIVATE include/rover_slam)

# Install targets
install(TARGETS rover_slam
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
