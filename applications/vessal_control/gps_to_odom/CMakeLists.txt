cmake_minimum_required(VERSION 3.10)
project(gps_to_odom)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(liquidai_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_library(utm_odometry_component SHARED
  src/gps_to_odom.cpp)
target_include_directories(utm_odometry_component
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(utm_odometry_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(utm_odometry_component "gps_tools::UtmOdometryComponent")
ament_target_dependencies(utm_odometry_component
  nav_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  liquidai_msgs
  tf2
  tf2_ros
)

add_library(utm_odometry_to_navsatfix_component SHARED
  src/utm_odometry_to_navsatfix_component.cpp)
target_include_directories(utm_odometry_to_navsatfix_component
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(utm_odometry_to_navsatfix_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(utm_odometry_to_navsatfix_component "gps_tools::UtmOdometryToNavSatFixComponent")
ament_target_dependencies(utm_odometry_to_navsatfix_component
  "nav_msgs"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS
    utm_odometry_component
    utm_odometry_to_navsatfix_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ 
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export package dependencies
ament_export_dependencies(ament_cmake
  rclcpp
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
