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

include_directories(include)

add_executable(simulation_control src/simulation_control.cpp)
ament_target_dependencies(simulation_control rclcpp liquidai_msgs lifecycle_msgs ament_index_cpp)  

install(TARGETS
  simulation_control
  DESTINATION lib/${PROJECT_NAME})

ament_package()
