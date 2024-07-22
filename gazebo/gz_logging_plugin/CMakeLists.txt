cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz_logging_plugin)

list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/gz-cmake3/cmake3)

# find_package(gz_cmake_vendor REQUIRED)
# find_package(gz-cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)

# find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED)

# find_package(gz_common_vendor REQUIRED)
find_package(gz-common5 REQUIRED)

# find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)

find_package(sdformat14 REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
 
# Add sources for each plugin to be registered.
add_library(LoggingSystem SHARED LoggingSystem.cc) #SampleSystem2.cc)
set_property(TARGET LoggingSystem PROPERTY CXX_STANDARD 17)
target_link_libraries(LoggingSystem
  gz-common5::gz-common5
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
)

install(TARGETS LoggingSystem DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
