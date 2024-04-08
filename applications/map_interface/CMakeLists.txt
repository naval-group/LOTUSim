cmake_minimum_required(VERSION 3.10)
project(map_interface)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(liquidai_msgs REQUIRED)
find_package(CURL REQUIRED)


include_directories(include)

add_executable(map_interface src/map_interface.cpp)
target_include_directories(map_interface 
  PUBLIC 
  ${CURL_INCLUDE_DIR}
)
target_link_libraries(map_interface 
  ${CURL_LIBRARIES}
)
ament_target_dependencies(map_interface
  rclcpp
  liquidai_msgs
)


install(TARGETS
  map_interface
  DESTINATION lib/${PROJECT_NAME})

ament_package()
