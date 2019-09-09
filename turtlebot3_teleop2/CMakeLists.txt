################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_teleop2)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "rclcpp"
  "sensor_msgs"
)

set(JOYSTICK_EXEC_NAME "open_manipulator_x_teleop_joystick")

add_executable(${JOYSTICK_EXEC_NAME} src/open_manipulator_x_teleop_joystick.cpp)
ament_target_dependencies(${JOYSTICK_EXEC_NAME} ${dependencies})

################################################################################
# Install
################################################################################
install(TARGETS ${JOYSTICK_EXEC_NAME} 
  DESTINATION lib/${PROJECT_NAME} 
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_package()
