cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(entity_manager VERSION 0.0.1 LANGUAGES CXX)

#============================================================================
# Configure the project
#============================================================================
# Add support for C++17
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

# # Code coverage
# if(BUILD_TESTING)
#     add_compile_options(-fprofile-arcs -ftest-coverage -g -Og)
# endif()

#============================================================================
# Search for project-specific dependencies
#============================================================================
find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

gz_configure_project(VERSION_SUFFIX)

set(protobuf_MODULE_COMPATIBLE TRUE)

gz_find_package(sdformat14 REQUIRED VERSION)
set(SDF_VER ${sdformat14_VERSION_MAJOR})

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-common5
  COMPONENTS
    profiler
    events
    av
  REQUIRED
)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(lotusim_common REQUIRED)
find_package(lotusim_msgs REQUIRED)

include_directories(
  include
  ${CMAKE_INSTALL_FULL_INCLUDEDIR}
) 

# link_directories(
#   ${CMAKE_INSTALL_FULL_LIBDIR}
# )

#============================================================================
# Create and install project files
#============================================================================
# Add sources for each plugin to be registered.
add_library(entity_manager_plugin 
  SHARED 
  src/entity_manager.cpp
)
target_include_directories(entity_manager_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR} include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(entity_manager_plugin
  gz-sim8::gz-sim8
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  lotusim_common::lotusim_common
)
ament_target_dependencies(entity_manager_plugin 
  rclcpp
  rclcpp_action
  ament_cmake
  lotusim_msgs
)

ament_export_targets(
  export_entity_manager_plugin
)

ament_export_dependencies(
  rclcpp
  ament_cmake
  lotusim_msgs
  lotusim_common
)
ament_export_include_directories(include)
ament_export_libraries(
  entity_manager_plugin
)

#============================================================================
# Install the files for this package
#============================================================================

install(
  TARGETS entity_manager_plugin
  EXPORT  export_entity_manager_plugin
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
