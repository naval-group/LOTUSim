cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(ais_sensor VERSION 0.0.1 LANGUAGES CXX)

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

# Code coverage
# if(BUILD_TESTING)
#     add_compile_options(-fprofile-arcs -ftest-coverage -g -Og)
# endif()

#============================================================================
# Search for project-specific dependencies
#============================================================================
find_package(lotusim_sensor_base REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(lotusim_common REQUIRED)
find_package(Eigen3 REQUIRED)

#============================================================================
# Create and install project files
#============================================================================
add_library(ais_sensor 
  SHARED   
  src/ais_sensor.cpp
)
target_include_directories(ais_sensor
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR} include>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(ais_sensor 
  PUBLIC  
  ament_cmake
  lotusim_common
  lotusim_sensor_base
  lotusim_msgs
  lotusim_sensor_msgs
)
ament_export_targets(
  export_ais_sensor
)
ament_export_dependencies(
  rclcpp
  ament_cmake
  lotusim_sensor_base
  lotusim_msgs
  lotusim_sensor_msgs
)
ament_export_include_directories(include)
ament_export_libraries(
  ais_sensor
)

#============================================================================
# Install the files for this package
#============================================================================
install(
  TARGETS ais_sensor
  EXPORT  export_ais_sensor
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
  PATTERN ".~"
  EXCLUDE
)

ament_package()
gz_create_packages()



