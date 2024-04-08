cmake_minimum_required(VERSION 3.5)

project(assets)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY models worlds launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
