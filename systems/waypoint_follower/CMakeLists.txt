cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(waypoint_follower_plugin VERSION 0.0.1 LANGUAGES CXX)

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
  set(CMAKE_CXX_CLANG_TIDY
      "clang-tidy;-checks=-*,-header-filter=.*,clang-analyzer-*,modernize-*,performance-*,hicpp-*,concurrency-*,bugprone-*,portability-*,cppcoreguidelines-*")
endif()

# Debug and profiling options
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
# Find dependencies
#============================================================================
find_package(gz-cmake3 REQUIRED)
gz_configure_project(VERSION_SUFFIX)

set(protobuf_MODULE_COMPATIBLE TRUE)

gz_find_package(sdformat14 REQUIRED)
gz_find_package(gz-sim8 REQUIRED)
gz_find_package(gz-msgs10 REQUIRED)
gz_find_package(gz-transport13 REQUIRED COMPONENTS log)
gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
gz_find_package(gz-common5 REQUIRED COMPONENTS profiler events av)

find_package(ament_cmake REQUIRED)
find_package(lotusim_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geographic_msgs REQUIRED)

#============================================================================
# Include and link directories
#============================================================================
include_directories(
  include
  ${CMAKE_INSTALL_FULL_INCLUDEDIR}
)

link_directories(
  ${CMAKE_INSTALL_FULL_LIBDIR}
)

#============================================================================
# Create library
#============================================================================
add_library(waypoint_plugin SHARED src/WaypointFollowerPlugin.cpp)

target_include_directories(waypoint_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR} include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(waypoint_plugin
  ${Boost_SYSTEM_LIBRARY}
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  lotusim_common
)

ament_target_dependencies(waypoint_plugin 
  rclcpp
  ament_cmake
  lotusim_common
  geographic_msgs
)

#============================================================================
# Install library and headers
#============================================================================
install(
  TARGETS waypoint_plugin
  EXPORT export_waypoint_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

#============================================================================
# Export
#============================================================================
ament_export_targets(export_waypoint_plugin)
ament_export_dependencies(ament_cmake rclcpp lotusim_common)
ament_export_include_directories(include)
ament_export_libraries(waypoint_plugin)

ament_package()
gz_create_packages()
