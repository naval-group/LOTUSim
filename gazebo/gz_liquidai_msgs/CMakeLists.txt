cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(gz_liquidai_msgs VERSION 0.0.1)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)
include(GenerateExportHeader)

find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

gz_configure_project(VERSION_SUFFIX)

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(protobuf_MODULE_COMPATIBLE TRUE)

gz_find_package(sdformat13 REQUIRED VERSION)
set(SDF_VER ${sdformat13_VERSION_MAJOR})

find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

gz_find_package(gz-msgs9 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})

gz_find_package(gz-transport12 REQUIRED COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(geometry_msgs REQUIRED)
gz_find_package(gz-common5
  COMPONENTS
    profiler
    events
    av
  REQUIRED
)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
set(REQ_PROTOBUF_VER 3)
gz_find_package(GzProtobuf
                 VERSION ${REQ_PROTOBUF_VER}
                 REQUIRED
                 COMPONENTS all
                 PRETTY Protobuf)
set(Protobuf_IMPORT_DIRS ${gz-msgs9_INCLUDE_DIRS} )

add_subdirectory(msg)

ament_export_dependencies(ament_cmake)
ament_package()

gz_create_packages()
