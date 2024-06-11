cmake_minimum_required(VERSION 3.8)
project(currents_visu)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

ament_export_dependencies(
  rclcpp
  geometry_msgs
  visualization_msgs)

  
add_executable(drone_marker src/drone_marker.cpp)
ament_target_dependencies(drone_marker
  "rclcpp"
  "visualization_msgs"
  "geometry_msgs") 
  
  
add_executable(current_marker src/current_marker.cpp)
ament_target_dependencies(current_marker
  "rclcpp"
  "visualization_msgs"
  "geometry_msgs")
  
  
  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  current_marker
  drone_marker
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY meshes/
 DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()
