cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(light_actuator VERSION 0.0.1 LANGUAGES CXX)

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror=format-security -Wpedantic)
endif()

#============================================================================
# Find dependencies
#============================================================================
# ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lotusim_common REQUIRED)

# Gazebo packages
find_package(sdformat14 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED COMPONENTS log)
find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
find_package(gz-common5 REQUIRED)

#============================================================================
# Create library
#============================================================================
add_library(light_actuator SHARED
  src/light_actuator.cpp
)

target_include_directories(light_actuator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(light_actuator
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  gz-common5::gz-common5
)

ament_target_dependencies(light_actuator
  rclcpp
  std_msgs
  lotusim_common
)

#============================================================================
# Installation
#============================================================================
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN "*.h"
)

install(
  TARGETS light_actuator
  EXPORT export_light_actuator
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#============================================================================
# Export
#============================================================================
ament_export_targets(export_light_actuator HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  lotusim_common
  gz-sim8
  gz-plugin2
  gz-msgs10
  gz-transport13
  sdformat14
  gz-common5
)

ament_package()
