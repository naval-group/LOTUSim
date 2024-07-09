cmake_minimum_required(VERSION 3.8)
project(agent_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(liquidai_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sdformat14 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(composition_interfaces REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

# add_executable(agent  src/agent_cpp/agent.cpp
#                       src/agent_cpp/agent_entity.cpp
#                       src/agent_node.cpp)
#   target_link_libraries(agent
#   ${rclcpp_lifecycle_LIBRARIES}
#   ${lifecycle_msgs_LIBRARIES}
#   ${rclcpp_components_LIBRARIES}
#   )

add_library(agent_component SHARED src/agent_cpp/agent.cpp
                  src/agent_cpp/agent_entity.cpp)

target_include_directories(agent_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

include_directories(include/${PROJECT_NAME})

include_directories(
  include
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  )

# Agent component

target_compile_features(agent_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(agent_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  lifecycle_msgs
  liquidai_msgs
  sdformat14
  geometry_msgs
  sensor_msgs
)

rclcpp_components_register_node(
    agent_component
    PLUGIN "AgentEntity"
    EXECUTABLE agent_node
)

ament_export_targets(export_agent_component)
install(TARGETS agent_component
        EXPORT export_agent_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# IMU

add_library(IMU_component SHARED src/MyIMUSensor.cpp)

target_include_directories(IMU_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(IMU_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  lifecycle_msgs
  liquidai_msgs
  sdformat14
  geometry_msgs
  sensor_msgs
)

rclcpp_components_register_node(
  IMU_component
    PLUGIN "MyIMUSensor"
    EXECUTABLE imu_node
)

ament_export_targets(export_IMU_component)
install(TARGETS IMU_component
        EXPORT export_IMU_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Scheduler

add_executable(scheduler src/Scheduler.cpp)
target_link_libraries(scheduler
  ${rclcpp_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
)

ament_target_dependencies(scheduler
  rclcpp
  rclcpp_components
  composition_interfaces
)

install(TARGETS
  scheduler
  DESTINATION lib/${PROJECT_NAME})

# Agent Factory

add_executable(agent_factory src/AgentFactory.cpp)
target_link_libraries(agent_factory
  ${rclcpp_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(agent_factory
  rclcpp
  rclcpp_components
  composition_interfaces
)

install(TARGETS
  agent_factory
  DESTINATION lib/${PROJECT_NAME})

ament_package()