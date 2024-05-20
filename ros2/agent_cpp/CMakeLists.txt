cmake_minimum_required(VERSION 3.8)
project(agent_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(custom_gz_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)

# add_executable(agent  src/agent_cpp/agent.cpp
#                       src/agent_cpp/agent_entity.cpp
#                       src/agent_node.cpp)
#   target_link_libraries(agent
#   ${rclcpp_lifecycle_LIBRARIES}
#   ${lifecycle_msgs_LIBRARIES}
#   ${rclcpp_components_LIBRARIES}
#   )

add_library(agent_component SHARED src/agent_cpp/agent.cpp
                  src/agent_cpp/agent_entity.cpp
                  src/agent_node.cpp)

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

target_compile_features(agent_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(agent_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  lifecycle_msgs
  custom_gz_interfaces
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


# install(TARGETS agent
#   DESTINATION lib/${PROJECT_NAME})


ament_package()
