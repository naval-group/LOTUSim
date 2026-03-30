cmake_minimum_required(VERSION 3.8)
project(lotusim_cpp_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#============================================================================
# Dependencies
#============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(lotusim_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#============================================================================
# Executable
#============================================================================
add_executable(spawn_ships src/spawn_ships.cpp)

#============================================================================
# Library
#============================================================================
ament_target_dependencies(spawn_ships
  rclcpp
  rclcpp_action
  geographic_msgs
  lotusim_msgs
)

#============================================================================
# Installation
#============================================================================
install(TARGETS
  spawn_ships
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
