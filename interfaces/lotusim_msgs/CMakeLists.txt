cmake_minimum_required(VERSION 3.5)
project(lotusim_msgs)

find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MASCmd.msg"
  "msg/VesselCmd.msg"
  "msg/VesselCmdArray.msg"
  "msg/VesselPosition.msg"
  "msg/VesselPositionArray.msg"
  "msg/RendererCmd.msg"
  "msg/Wind.msg"
  "msg/SimStats.msg"
  "action/MASCmdArray.action"
  "action/MASCmd.action"
  
  DEPENDENCIES 
    builtin_interfaces
    geometry_msgs 
    geographic_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
