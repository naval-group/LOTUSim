cmake_minimum_required(VERSION 3.8)

project(power_subsystem)

#============================================================================
# Configure the project
#============================================================================
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lotusim_common REQUIRED)
find_package(rclcpp REQUIRED)

# Gazebo packages
find_package(gz-cmake3 REQUIRED)
find_package(sdformat14 REQUIRED)
find_package(gz-sim8 REQUIRED)
gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
gz_find_package(gz-common5 REQUIRED COMPONENTS profiler events av)

#============================================================================
# Create and install project files
#============================================================================
add_library(power_subsystem
SHARED
src/power_manager.cpp
src/power_manager_instance.cpp
)
target_include_directories(power_subsystem
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(power_subsystem
  PUBLIC
    gz-sim8::gz-sim8
    gz-common5::gz-common5
    sdformat14::sdformat14
    gz-plugin2::gz-plugin2
    gz-plugin2::register
)

ament_target_dependencies(power_subsystem 
  PUBLIC
  rclcpp
  lotusim_common
  std_msgs
)

#============================================================================
# Install the files for this package
#============================================================================
install(
  TARGETS power_subsystem
  EXPORT  export_power_subsystem
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

#============================================================================
# Export
#============================================================================
ament_export_targets(export_power_subsystem HAS_LIBRARY_TARGET)
ament_export_dependencies(
  gz-cmake3
  ament_cmake
  rclcpp
  lotusim_common
  gz-sim8
  gz-plugin2
)

ament_export_targets(
  export_power_subsystem
)
ament_export_libraries(
  power_subsystem
)
ament_export_include_directories(include)

ament_package()
