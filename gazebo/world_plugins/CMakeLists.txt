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

gz_find_package(sdformat13 REQUIRED VERSION)
set(SDF_VER ${sdformat13_VERSION_MAJOR})

find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

gz_find_package(gz-msgs9 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})

gz_find_package(gz-transport12 REQUIRED COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

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

message("gz liquidai msg  " ${gz_liquidai_msgs_INCLUDE_DIRS})

include_directories(include)

# Wave param ROS plugin
add_library(wave_param_plugin 
  SHARED 
  src/WaveParamPlugin.cpp 
)
target_include_directories(wave_param_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
  ${gz_liquidai_msgs_INCLUDE_DIRS}
)
target_link_libraries(wave_param_plugin
  gz-sim7::gz-sim7
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
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
  ${gz_liquidai_msgs_INCLUDE_DIRS}
)
target_link_libraries(ais_plugin
  ${Boost_SYSTEM_LIBRARY}
  gz-sim7::gz-sim7
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz_liquidai_msgs
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
  src/WaveRAOPlugin.cpp
  src/XdynWebSocket.cpp
  # src/XdynGrpc.cpp
  # src/ManualRAO.cpp
)
target_include_directories(wave_rao_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>
  ${gz_liquidai_msgs_INCLUDE_DIRS}
)
target_link_libraries(wave_rao_plugin
  ${Boost_SYSTEM_LIBRARY}
  gz-sim7::gz-sim7
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  websocketpp::websocketpp
  gz_liquidai_msgs
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
)

ament_export_dependencies(
  geometry_msgs
  liquidai_msgs
  rclcpp
  ament_cmake
  gz_liquidai_msgs
)
ament_export_include_directories(include)
ament_export_libraries(
  uuv_underwater_current_ros_plugin
  uuv_sc_ros_interface_plugin
  ais_plugin
  wave_rao_plugin
)

ament_package()

gz_create_packages()



