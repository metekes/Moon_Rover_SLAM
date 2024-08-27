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
find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)

include_directories(${g2o_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})

# Create executable
add_library(icp_graph_lib src/icp_graph.cpp)
target_link_libraries(icp_graph_lib g2o::core ${Eigen3_LIBS})
target_include_directories(icp_graph_lib PRIVATE include/rover_slam)

# Create executable
add_executable(rover_slam src/rover_slam.cpp src/rover_slam_node.cpp)
ament_target_dependencies(rover_slam rclcpp sensor_msgs cv_bridge OpenCV message_filters)
target_include_directories(rover_slam PRIVATE include/rover_slam)
target_link_libraries(rover_slam icp_graph_lib g2o::core g2o_types_slam3d g2o_types_sba)

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
