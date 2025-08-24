cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(lotusim_sensor_plugin VERSION 0.0.1 LANGUAGES CXX)

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
# For debugging
if(DEBUG)
  message("Building ${PROJECT_NAME} with debug options ${DEBUG}")
  add_compile_options(-g -Og)
  find_package(backward_ros)
  # Used to define the logger level
  add_definitions(-DDEBUG=${DEBUG} )
endif()

# For profiling
if(BUILD_PROFILING)
  add_compile_options(-pg -g -Og)
endif()

if(NOT DEFINED BUILD_SHARED_LIBS)
  option(BUILD_SHARED_LIBS "Build dynamically-linked binaries" ON)
endif()

#============================================================================
# Search for project-specific dependencies
#============================================================================
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lotusim_common REQUIRED)
find_package(lotusim_sensor_base REQUIRED)
find_package(subsea_pressure_sensor REQUIRED)
find_package(ais_sensor REQUIRED)
find_package(imu_sensor REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(gz-sensors8 REQUIRED)

#============================================================================
# Create and install project files
#============================================================================

add_library(lotusim_sensor_plugin
SHARED
src/lotusim_sensor_plugin.cpp
)
target_include_directories(lotusim_sensor_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR} include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(lotusim_sensor_plugin
  gz-rendering8::gz-rendering8
  gz-sensors8::lidar
  gz-sensors8::gpu_lidar
  gz-sensors8::rendering   
)
ament_target_dependencies(lotusim_sensor_plugin 
  ament_cmake
  lotusim_common
  lotusim_sensor_base
  subsea_pressure_sensor
  ais_sensor
  imu_sensor
)
ament_export_include_directories(include)
ament_export_dependencies(
  ament_cmake
  lotusim_common
  lotusim_sensor_base
  subsea_pressure_sensor
  ais_sensor
  imu_sensor
)
ament_export_targets(
  export_lotusim_sensor_plugin
)
ament_export_libraries(
  lotusim_sensor_plugin
)
#============================================================================
# Install the files for this package
#============================================================================
install(
  TARGETS lotusim_sensor_plugin
  EXPORT  export_lotusim_sensor_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN "*.h"
)
ament_package()
gz_create_packages()
 