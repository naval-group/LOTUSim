cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(custom_sensor_plugins VERSION 0.0.1 LANGUAGES CXX)

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
if(BUILD_TESTING)
    add_compile_options(-fprofile-arcs -ftest-coverage -g -Og)
endif()

#============================================================================
# Search for project-specific dependencies
#============================================================================
find_package(gz-cmake3 REQUIRED)

set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

gz_configure_project(VERSION_SUFFIX)

gz_find_package(sdformat14 REQUIRED VERSION)
set(SDF_VER ${sdformat14_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

gz_find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

gz_find_package(gz-transport13 REQUIRED COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-sensors8 REQUIRED)

gz_find_package(gz-common5
  COMPONENTS
    profiler
    events
    av
  REQUIRED
)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lotusim_common REQUIRED)

include_directories(
  include
  ${CMAKE_INSTALL_FULL_INCLUDEDIR}
)
link_directories(
  ${CMAKE_INSTALL_FULL_LIBDIR}
)

#============================================================================
# Create and install project files
#============================================================================
add_library(gazebo_sensor_system_plugin
SHARED
src/CustomSensor.cc
src/SensorFactory.cc
src/SubseaPressureSensor.cc
)
target_include_directories(gazebo_sensor_system_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(gazebo_sensor_system_plugin
  gz-sim8::gz-sim8
  gz-sensors8::gz-sensors8
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  lotusim_common
)
ament_target_dependencies(gazebo_sensor_system_plugin 
  ament_cmake
)

#============================================================================
# Install the files for this package
#============================================================================
install(
  TARGETS gazebo_sensor_system_plugin
  EXPORT  export_gazebo_sensor_system_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(
  export_gazebo_sensor_system_plugin
)
ament_export_libraries(
  gazebo_sensor_system_plugin
)
ament_package()
gz_create_packages()
