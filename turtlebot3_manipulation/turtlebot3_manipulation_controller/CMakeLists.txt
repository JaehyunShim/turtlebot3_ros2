################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_manipulation_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ar_track_alvar_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy) REQUIRED)
find_package(smach REQUIRED)
find_package(smach_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(open_manipulator_msgs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "ar_track_alvar_msgs"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "rclpy"
  "smach"
  "smach_ros"
  "std_msgs"
  "trajectory_msgs"
  "open_manipulator_msgs"
)

set(EXEC_NAME "turtlebot3_manipulation_controller")

add_executable(${EXEC_NAME} src/gazebo_bridge.cpp)
ament_target_dependencies(${EXEC_NAME} ${dependencies})

################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(ar_track_alvar_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(smach)
ament_export_dependencies(smach_ros)
ament_export_dependencies(std_msgs)
ament_export_dependencies(trajectory_msgs)
ament_export_dependencies(open_manipulator_msgs)
ament_package()
