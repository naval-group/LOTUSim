cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(render_interface_plugin VERSION 0.0.1 LANGUAGES CXX)

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
# Set project-specific options
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
# Search for dependencies
#============================================================================
# ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lotusim_common REQUIRED)
find_package(lotusim_msgs REQUIRED)

# Third-party libraries
find_package(yaml-cpp REQUIRED)
# Boost.System is header-only since 1.69 and BoostConfig.cmake ships no
# boost_system component, so requesting it fails once CMP0167 drops FindBoost.
find_package(Boost REQUIRED)

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
add_library(render_plugin SHARED   
  src/tcpudp_interface.cpp
  src/tcpudp_messages.cpp
  src/render_plugin.cpp
  src/ros_interface.cpp
)

target_include_directories(render_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(render_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  Boost::boost
  yaml-cpp
)

ament_target_dependencies(render_plugin
  rclcpp
  lotusim_common
  lotusim_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nlohmann_json REQUIRED)
  find_package(fmt REQUIRED)
  find_package(gz-math7 REQUIRED)

  # The JSON commands sent to the renderer over TCP; pure functions, no
  # socket or Gazebo needed.
  ament_add_gtest(test_tcpudp_messages
    test/test_tcpudp_messages.cpp
    src/tcpudp_messages.cpp
  )
  if(TARGET test_tcpudp_messages)
    target_include_directories(test_tcpudp_messages PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(test_tcpudp_messages
      fmt::fmt
      gz-math7::gz-math7
      nlohmann_json::nlohmann_json
    )
  endif()
endif()

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
  TARGETS render_plugin
  EXPORT export_render_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#============================================================================
# Export
#============================================================================
ament_export_targets(export_render_plugin HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  lotusim_common
  lotusim_msgs
  yaml-cpp
  Boost
  gz-sim8
  gz-plugin2
  gz-msgs10
  gz-transport13
  sdformat14
  gz-common5
)

ament_package()