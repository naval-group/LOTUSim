cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
project(radar_sensor VERSION 0.0.1 LANGUAGES CXX)

#============================================================================
# Configure the project
#============================================================================
include(GNUInstallDirs)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror=format-security -Wpedantic)
endif()

if(LOTUS_TIDY)
  set(CMAKE_CXX_CLANG_TIDY "clang-tidy;-checks=-*,-header-filter=.*, clang-analyzer-*,modernize-*,performance-*,hicpp-*,concurrency-*,bugprone-*,portability-*,cppcoreguidelines-*")
endif()

#============================================================================
# Set project-specific options
#============================================================================
if(DEBUG)
  message("Building ${PROJECT_NAME} with debug options ${DEBUG}")
  add_compile_options(-g -Og)
  find_package(backward_ros)
  add_definitions(-DDEBUG=${DEBUG})
endif()

if(BUILD_PROFILING)
  add_compile_options(-pg -g -Og)
endif()

if(NOT DEFINED BUILD_SHARED_LIBS)
  option(BUILD_SHARED_LIBS "Build dynamically-linked binaries" ON)
endif()

#============================================================================
# Search for dependencies
#============================================================================
# ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lotusim_common REQUIRED)
find_package(lotusim_sensor_base REQUIRED)
find_package(lotusim_msgs REQUIRED)
find_package(lotusim_sensor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Gazebo packages — exact same pattern as entity_manager
find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

#============================================================================
# Create library
#============================================================================
add_library(radar_sensor
  SHARED
  src/radar_sensor.cpp
)

target_include_directories(radar_sensor
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# target_link_libraries BEFORE ament_target_dependencies — same as entity_manager
target_link_libraries(radar_sensor
  PUBLIC
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
)

ament_target_dependencies(radar_sensor
  PUBLIC
  rclcpp
  lotusim_common
  lotusim_sensor_base
  lotusim_msgs
  lotusim_sensor_msgs
  sensor_msgs
)

#============================================================================
# Install
#============================================================================
install(
  TARGETS radar_sensor
  EXPORT  export_radar_sensor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN "*.h"
  PATTERN ".~" EXCLUDE
)

#============================================================================
# Export
#============================================================================
ament_export_targets(export_radar_sensor HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  lotusim_common
  lotusim_sensor_base
  lotusim_msgs
  lotusim_sensor_msgs
  sensor_msgs
  gz-sim8
  gz-msgs10
  gz-transport13
  gz-plugin2
)
ament_export_include_directories(include)
ament_package()
gz_create_packages()