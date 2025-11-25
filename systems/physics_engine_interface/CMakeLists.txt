cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(physics_engine_interface VERSION 0.0.1 LANGUAGES CXX)

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

if(DEBUG)
  message(STATUS "Building ${PROJECT_NAME} with debug options")
  add_compile_options(-g -Og)
  find_package(backward_ros)
  add_definitions(-DDEBUG=${DEBUG})
endif()

if(BUILD_PROFILING)
  add_compile_options(-pg -g -Og)
endif()

#============================================================================
# Dependencies
#============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lotusim_common REQUIRED)
find_package(lotusim_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(websocketpp REQUIRED)

find_package(gz-cmake3 REQUIRED)

gz_find_package(sdformat14 REQUIRED)
find_package(gz-sim8 REQUIRED)
gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)

#============================================================================
# Library
#============================================================================
add_library(physics_interface_plugin SHARED
src/physics_interface_plugin.cpp
  src/xdyn_websocket.cpp
  src/ros2_interface.cpp
)

target_include_directories(physics_interface_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(physics_interface_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  websocketpp::websocketpp
  Boost::system
  yaml-cpp
)

ament_target_dependencies(physics_interface_plugin
  rclcpp
  geometry_msgs
  lotusim_common
  lotusim_msgs
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
  TARGETS physics_interface_plugin
  EXPORT export_physics_interface_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#============================================================================
# Export
#============================================================================
ament_export_targets(export_physics_interface_plugin HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  lotusim_common
  lotusim_msgs
  yaml-cpp
  Boost
  websocketpp
  gz-sim8
  gz-plugin2
)

ament_package()