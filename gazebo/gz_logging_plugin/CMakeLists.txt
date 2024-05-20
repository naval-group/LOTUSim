cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz_logging_plugin)

find_package(gz-cmake3 REQUIRED)

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

find_package(gz-sim8 REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#add_library(robot_logging_plugin SHARED RobotLoggingPlugin.cpp)
#target_link_libraries(robot_logging_plugin
#  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
#  PRIVATE gz-sim8::gz-sim8)
 
# Add sources for each plugin to be registered.
add_library(LoggingSystem SHARED LoggingSystem.cc) #SampleSystem2.cc)
set_property(TARGET LoggingSystem PROPERTY CXX_STANDARD 17)
target_link_libraries(LoggingSystem
  PRIVATE gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim8::gz-sim8
)

install(TARGETS LoggingSystem DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
