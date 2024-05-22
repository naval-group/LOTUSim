cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(liquidai_world_plugins VERSION 0.0.1)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)
include(GenerateExportHeader)

find_package(gz-cmake3 REQUIRED)

set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

gz_configure_project(VERSION_SUFFIX)

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(protobuf_MODULE_COMPATIBLE TRUE)

gz_find_package(sdformat14 REQUIRED VERSION)
set(SDF_VER ${sdformat14_VERSION_MAJOR})

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

gz_find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

gz_find_package(gz-transport13 REQUIRED COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-common5
  COMPONENTS
    profiler
    events
    av
  REQUIRED
)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(liquidai_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED system)
find_package(websocketpp REQUIRED)
find_package(gz_liquidai_msgs REQUIRED)

message("gz liquidai msg  " ${CMAKE_INSTALL_FULL_LIBDIR})

include_directories(
  include
  # ahoy Desperate measures for gz_liquidai_msgs linker
  ${CMAKE_INSTALL_FULL_INCLUDEDIR}
  )
link_directories(
${CMAKE_INSTALL_FULL_LIBDIR}
)

# Wave param ROS plugin
add_library(wave_param_plugin 
  SHARED 
  src/WaveParamPlugin.cpp 
)
target_include_directories(wave_param_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>
)
target_link_libraries(wave_param_plugin
  gz-sim8::gz-sim8
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
)
ament_target_dependencies(wave_param_plugin 
  ament_cmake
  geometry_msgs
  liquidai_msgs
  rclcpp
)

install(
  TARGETS wave_param_plugin
  EXPORT  export_wave_param_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# AIS plugin
add_library(ais_plugin 
  SHARED   
  src/AISPlugin.cpp
)
target_include_directories(ais_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR} include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ais_plugin
  ${Boost_SYSTEM_LIBRARY}
  gz-sim8::gz-sim8
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz_liquidai_msgs-msg
)
install(
  TARGETS ais_plugin
  EXPORT  export_ais_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Wave RAO plugin
add_library(wave_rao_plugin 
  SHARED
  src/WaveRaoInterface.cpp
  src/XdynWebSocket.cpp
  src/WaveRAOPlugin.cpp
  # src/XdynGrpc.cpp
  # src/ManualRAO.cpp
)
target_include_directories(wave_rao_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR} include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(wave_rao_plugin
  ${Boost_SYSTEM_LIBRARY}
  gz-sim8::gz-sim8
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  websocketpp::websocketpp
  gz_liquidai_msgs-msg
)
ament_target_dependencies(wave_rao_plugin 
  ament_cmake
  geometry_msgs
  liquidai_msgs
  rclcpp
)
ament_target_dependencies(wave_rao_plugin 
  ament_cmake
  geometry_msgs
  liquidai_msgs
  rclcpp
)
install(
  TARGETS wave_rao_plugin
  EXPORT  export_wave_rao_plugin
  ARCHIVE DESTINATION lib 
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Render plugin
add_library(render_plugin 
  SHARED   
  src/RenderPlugin.cpp
)
target_include_directories(render_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR} include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(render_plugin
  ${Boost_SYSTEM_LIBRARY}
  gz-sim8::gz-sim8
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz_liquidai_msgs-msg
)
install(
  TARGETS render_plugin
  EXPORT  export_render_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Waypoint plugin
add_library(waypoint_plugin 
  SHARED   
  src/WaypointFollowerPlugin.cpp
)
target_include_directories(waypoint_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR} include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(waypoint_plugin
  ${Boost_SYSTEM_LIBRARY}
  gz-sim8::gz-sim8
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
)
install(
  TARGETS waypoint_plugin
  EXPORT  export_waypoint_plugin
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

ament_export_targets(
  export_wave_param_plugin
  export_ais_plugin
  export_wave_rao_plugin
  export_render_plugin
  export_waypoint_plugin
)

ament_export_dependencies(
  geometry_msgs
  liquidai_msgs
  rclcpp
  ament_cmake
)
ament_export_include_directories(include)
ament_export_libraries(
  wave_param_plugin
  ais_plugin
  wave_rao_plugin
  render_plugin
  waypoint_plugin
)

ament_package()

gz_create_packages()



