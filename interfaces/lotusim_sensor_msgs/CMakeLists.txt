cmake_minimum_required(VERSION 3.5)
project(lotusim_sensor_msgs)

find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AIS.msg"
  "msg/GPS.msg"
  "msg/PressureDepth.msg"
  "msg/Collision.msg"
  "msg/Collisions.msg"
  "srv/ActivateSensor.srv"
  
  DEPENDENCIES 
    builtin_interfaces
    geometry_msgs 
    geographic_msgs
    sensor_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
