cmake_minimum_required(VERSION 3.5)
project(liquidai_msgs)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AddEntity.msg"
  "msg/AIS.msg"
  "msg/AISArray.msg"  
  "msg/GPS.msg"
  "msg/UnderwaterObjectModel.msg" 
  "msg/ThrusterConversionFcn.msg"
  "msg/FloatStamped.msg"
  "msg/WaveParam.msg"
  "msg/Xdyncmd.msg"
  "msg/XdynThrustercmd.msg"
  "msg/EntityPosition.msg"
  "srv/SetCurrentModel.srv"
  "srv/GetCurrentModel.srv" 
  "srv/SetCurrentVelocity.srv"
  "srv/SetCurrentDirection.srv" 
  "srv/SetOriginSphericalCoord.srv"
  "srv/GetOriginSphericalCoord.srv" 
  "srv/TransformToSphericalCoord.srv"
  "srv/TransformFromSphericalCoord.srv"
  "srv/SetModelSphericalCoord.srv"
  "srv/GetModelSphericalCoord.srv" 
  "srv/SetThrusterEfficiency.srv" 
  "srv/SetThrusterState.srv"
  "srv/GetThrusterEfficiency.srv" 
  "srv/GetThrusterState.srv"
  "srv/SetUseGlobalCurrentVel.srv" 
  "srv/GetModelProperties.srv" 
  "srv/SetFloat.srv"
  "srv/GetFloat.srv" 
  "srv/GetListParam.srv" 
  "srv/GetThrusterConversionFcn.srv"
  "srv/AddEntitySrv.srv"
  "srv/AddEntitySrvArray.srv"
  "srv/RemoveEntity.srv"
  "srv/EmptySrv.srv"
  "srv/GetIdByName.srv"
  DEPENDENCIES 
    builtin_interfaces
    geometry_msgs 
    sensor_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
