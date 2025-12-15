cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(waypoint_follower_plugin VERSION 0.0.1 LANGUAGES CXX)

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror=format-security -Wpedantic)
endif()

if(LOTUS_TIDY)
  set(CMAKE_CXX_CLANG_TIDY
    "clang-tidy;-checks=-*,clang-analyzer-*,modernize-*,performance-*,hicpp-*,concurrency-*,bugprone-*,portability-*,cppcoreguidelines-*;-header-filter=.*")
endif()

#============================================================================
# Debug and profiling options
#============================================================================
if(DEBUG)
  message(STATUS "Building ${PROJECT_NAME} with debug options ${DEBUG}")
  add_compile_options(-g -Og)
  find_package(backward_ros)
  add_definitions(-DDEBUG=${DEBUG})
endif()

if(BUILD_PROFILING)
  add_compile_options(-pg -g -Og)
endif()

#============================================================================
# Find dependencies
#============================================================================
# ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(lotusim_common REQUIRED)
find_package(lotusim_msgs REQUIRED)

# Gazebo packages
find_package(sdformat14 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED COMPONENTS log)
find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
find_package(gz-common5 REQUIRED COMPONENTS profiler events av)

#============================================================================
# Create library
#============================================================================
add_library(waypoint_plugin SHARED
  src/waypoint_follower.cpp
)

target_include_directories(waypoint_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(waypoint_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

ament_target_dependencies(waypoint_plugin
  rclcpp
  geographic_msgs
  lotusim_msgs
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
  TARGETS waypoint_plugin
  EXPORT export_waypoint_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#============================================================================
# Export
#============================================================================
ament_export_targets(export_waypoint_plugin HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  geographic_msgs
  lotusim_msgs
  lotusim_common
  gz-sim8
  gz-plugin2
  gz-msgs10
  gz-transport13
  sdformat14
  gz-common5
)

ament_package()