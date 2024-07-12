cmake_minimum_required(VERSION 3.8)
project(simulation_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(liquidai_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(simulation_control SHARED src/simulation_control.cpp)

target_include_directories(simulation_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
)

ament_target_dependencies(simulation_control
  rclcpp
  liquidai_msgs
  lifecycle_msgs
  std_srvs
  rclcpp_components
  ament_index_cpp
)

rclcpp_components_register_node(
  simulation_control
    PLUGIN "SimulationControl"
    EXECUTABLE simulation_control_node
)

ament_export_targets(export_simulation_control)
install(TARGETS simulation_control
        EXPORT export_simulation_control
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_package()
