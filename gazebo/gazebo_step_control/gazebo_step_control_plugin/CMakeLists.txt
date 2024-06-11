cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gazebo_step_control)

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

# find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs10 REQUIRED)

# find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport13 REQUIRED)

find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_step_control_interface REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
 
# Add sources for each plugin to be registered.
add_library(gazebo_step_control SHARED src/gazebo_step_control.cpp) #SampleSystem2.cc)
set_property(TARGET gazebo_step_control PROPERTY CXX_STANDARD 17)
target_link_libraries(gazebo_step_control
  gz-common5::gz-common5
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-msgs10::gz-msgs10
)

ament_target_dependencies(gazebo_step_control rclcpp std_srvs std_msgs gazebo_step_control_interface)

install(TARGETS gazebo_step_control DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
