cmake_minimum_required(VERSION 3.10.2)
project(sensor_plugins VERSION 1.0.0)

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
find_package(gz_liquidai_msgs REQUIRED)

include_directories(
  include
  ${CMAKE_INSTALL_FULL_INCLUDEDIR}
)
link_directories(
  ${CMAKE_INSTALL_FULL_LIBDIR}
)


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
  gz_liquidai_msgs-msgs
)
ament_target_dependencies(gazebo_sensor_system_plugin 
  ament_cmake
)

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
