cmake_minimum_required(VERSION 3.8)
project(entity_management)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_gz_interfaces REQUIRED)

add_executable(entity_management_service src/entity_management_service.cpp)
ament_target_dependencies(entity_management_service rclcpp custom_gz_interfaces)  

install(TARGETS
  entity_management_service
  DESTINATION lib/${PROJECT_NAME})

ament_package()
