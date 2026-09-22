cmake_minimum_required(VERSION 3.8)
project(lotusim_cpp_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#============================================================================
# Dependencies
#============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(lotusim_msgs REQUIRED)
find_package(std_msgs REQUIRED)

#============================================================================
# Executable
#============================================================================
add_executable(spawn_ships src/spawn_ships.cpp)
add_executable(controlling_ships src/controlling_ships.cpp)
add_executable(detection_radar src/detection_radar.cpp)

#============================================================================
# Library
#============================================================================
ament_target_dependencies(spawn_ships
  rclcpp
  rclcpp_action
  geographic_msgs
  lotusim_msgs
)

ament_target_dependencies(controlling_ships
  rclcpp
  rclcpp_action
  geographic_msgs
  lotusim_msgs
  std_msgs
)

ament_target_dependencies(detection_radar
  rclcpp
  rclcpp_action
  geographic_msgs
  lotusim_msgs
)

#============================================================================
# Installation
#============================================================================
install(TARGETS
  spawn_ships
  controlling_ships
  detection_radar
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
