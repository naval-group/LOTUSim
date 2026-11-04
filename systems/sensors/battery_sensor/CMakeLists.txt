cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
project(battery_sensor VERSION 0.0.1 LANGUAGES CXX)

#============================================================================
# Configure the project
#============================================================================
include(GNUInstallDirs)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror=format-security -Wpedantic)
endif()

if(NOT DEFINED BUILD_SHARED_LIBS)
  option(BUILD_SHARED_LIBS "Build dynamically-linked binaries" ON)
endif()

#============================================================================
# Dependencies
#============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lotusim_common REQUIRED)
find_package(lotusim_sensor_base REQUIRED)
find_package(lotusim_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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

gz_find_package(gz-common5 REQUIRED)

#============================================================================
# Library
#============================================================================
add_library(battery_sensor SHARED src/battery_sensor.cpp)

target_include_directories(battery_sensor
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(battery_sensor
  PUBLIC
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-common5::gz-common5
)

ament_target_dependencies(battery_sensor
  PUBLIC
  rclcpp
  lotusim_common
  lotusim_sensor_base
  lotusim_msgs
  sensor_msgs
)

#============================================================================
# Install
#============================================================================
install(
  TARGETS battery_sensor
  EXPORT  export_battery_sensor
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
ament_export_targets(export_battery_sensor HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  lotusim_common
  lotusim_sensor_base
  lotusim_msgs
  sensor_msgs
  gz-sim8
  gz-msgs10
  gz-transport13
  gz-plugin2
  gz-common5
)
ament_export_include_directories(include)
ament_package()
gz_create_packages()
