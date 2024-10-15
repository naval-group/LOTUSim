cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(gz_liquidai_msgs VERSION 1.0.0)

#============================================================================
# Find gz-cmake
#============================================================================
find_package(gz-cmake3 REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(gz-msgs10 REQUIRED)
# Define a variable 'GZ_MSGS_VER' holding the version number
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

# Define a variable 'MSGS_PROTOS' listing the .proto files
set(MSGS_PROTOS
   ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz_liquidai_msgs/msgs/wavemsg.proto
   ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz_liquidai_msgs/msgs/aismsg.proto
   ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz_liquidai_msgs/msgs/xdyncmdmsg.proto
   ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz_liquidai_msgs/msgs/SensorPressure.proto
)

# Call 'gz_msgs_generate_messages()' to process the .proto files
gz_msgs_generate_messages(
  # The cmake target to be generated for libraries/executables to link
  TARGET msgs
  # The protobuf package to generate (Typically based on the path)
  PROTO_PACKAGE "gz_liquidai_msgs.msgs"
  # The path to the base directory of the proto files
  # All import paths should be relative to this (eg gz/custom_msgs/vector3d.proto)
  MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/proto
  # List of proto files to process
  MSGS_PROTOS ${MSGS_PROTOS}
  # Depenency on gz-msgs
  DEPENDENCIES gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)
ament_export_dependencies(ament_cmake)
ament_package()

