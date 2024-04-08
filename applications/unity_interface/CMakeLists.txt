cmake_minimum_required(VERSION 3.10)
project(unity_interface)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS system )

include_directories(include)

add_executable(unity_interface src/unity_interface.cpp)

TARGET_LINK_LIBRARIES(unity_interface 
    ${Boost_SYSTEM_LIBRARY} 
)

ament_target_dependencies(unity_interface 
    rclcpp 
    std_msgs
    geometry_msgs
    tf2_msgs
)

# add_executable(map_interface src/map_interface.cpp)
# target_include_directories(map_interface 
#   PUBLIC 
#   ${CURL_INCLUDE_DIR}
# )
# target_link_libraries(map_interface 
#   ${CURL_LIBRARIES}
# )
# ament_target_dependencies(map_interface
#   rclcpp
#   liquidai_msgs
# )


install(TARGETS 
  unity_interface
  DESTINATION lib/${PROJECT_NAME})

ament_package()
