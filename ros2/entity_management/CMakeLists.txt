cmake_minimum_required(VERSION 3.8)
project(entity_management)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(liquidai_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(entity_management_service src/entity_management_service.cpp)
ament_target_dependencies(entity_management_service rclcpp liquidai_msgs ament_index_cpp std_srvs)  

install(TARGETS
  entity_management_service
  DESTINATION lib/${PROJECT_NAME})

ament_package()
