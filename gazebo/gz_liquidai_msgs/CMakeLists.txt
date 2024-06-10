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

# cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
# project(gz_liquidai_msgs VERSION 0.0.1)

# # Add support for C++17
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# include(GNUInstallDirs)
# include(GenerateExportHeader)

# find_package(gz-cmake3 REQUIRED)
# set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

# gz_configure_project(VERSION_SUFFIX)

# set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
# set(protobuf_MODULE_COMPATIBLE TRUE)

# gz_find_package(sdformat14 REQUIRED VERSION)
# set(SDF_VER ${sdformat14_VERSION_MAJOR})

# find_package(gz-sim8 REQUIRED)
# set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

# gz_find_package(gz-msgs10 REQUIRED)
# set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

# gz_find_package(gz-transport13 REQUIRED COMPONENTS log)
# set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

# gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
# set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
# find_package(geometry_msgs REQUIRED)
# gz_find_package(gz-common5
#   COMPONENTS
#     profiler
#     events
#     av
#   REQUIRED
# )
# set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
# set(REQ_PROTOBUF_VER 3)
# gz_find_package(GzProtobuf
#                  VERSION ${REQ_PROTOBUF_VER}
#                  REQUIRED
#                  COMPONENTS all
#                  PRETTY Protobuf)
# set(Protobuf_IMPORT_DIRS ${gz-msgs10_INCLUDE_DIRS} )

# add_subdirectory(msg)

# ament_export_dependencies(ament_cmake)
# ament_package()

# gz_create_packages()
