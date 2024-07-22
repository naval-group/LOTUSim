cmake_minimum_required(VERSION 3.8)
project(agent_demo_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(liquidai_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(agent_demo_node src/agent_demo_node.cpp)
target_include_directories(agent_demo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(agent_demo_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(agent_demo_node
  rclcpp
  liquidai_msgs
  std_msgs
  geometry_msgs
)

install(TARGETS agent_demo_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
