cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(physics_engine_interface VERSION 0.0.1 LANGUAGES CXX)

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror=format-security -Wpedantic)
endif()

if(LOTUS_TIDY)
  set(CMAKE_CXX_CLANG_TIDY
    "clang-tidy;-checks=-*,clang-analyzer-*,modernize-*,performance-*,hicpp-*,concurrency-*,bugprone-*,portability-*,cppcoreguidelines-*;-header-filter=.*")
endif()

if(DEBUG)
  message(STATUS "Building ${PROJECT_NAME} with debug options")
  add_compile_options(-g -Og)
  find_package(backward_ros)
  add_definitions(-DDEBUG=${DEBUG})
endif()

if(BUILD_PROFILING)
  add_compile_options(-pg -g -Og)
endif()

#============================================================================
# Dependencies
#============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lotusim_common REQUIRED)
find_package(lotusim_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
# Boost.System is header-only since 1.69 and BoostConfig.cmake ships no
# boost_system component, so requesting it fails once CMP0167 drops FindBoost.
find_package(Boost REQUIRED)
find_package(websocketpp REQUIRED)
# FMI 2.0 FMUs (fmu_interface): unpacked with libzip, modelDescription.xml
# read with tinyxml2, the FMU's library dlopen'ed. libzip through pkg-config:
# Ubuntu's libzip CMake package also imports the zipcmp, zipmerge and ziptool
# executables, and fails when those packages are not installed.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBZIP REQUIRED IMPORTED_TARGET libzip)
find_package(tinyxml2 REQUIRED)

find_package(gz-cmake3 REQUIRED)

gz_find_package(sdformat14 REQUIRED)
find_package(gz-sim8 REQUIRED)
gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)

#============================================================================
# Library
#============================================================================
add_library(physics_interface_plugin SHARED
  src/physics_interface_base.cpp
  src/physics_interface_plugin.cpp
  src/xdyn_websocket.cpp
  src/ros2_interface.cpp
  src/vessel_information.cpp
  src/fmi2_slave.cpp
  src/fmu_interface.cpp
)

target_include_directories(physics_interface_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/fmi2>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(physics_interface_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  websocketpp::websocketpp
  Boost::boost
  yaml-cpp
  PkgConfig::LIBZIP
  tinyxml2::tinyxml2
  ${CMAKE_DL_LIBS}
)

ament_target_dependencies(physics_interface_plugin
  rclcpp
  geometry_msgs
  lotusim_common
  lotusim_msgs
)

#============================================================================
# Installation
#============================================================================
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN "*.h"
)

install(
  DIRECTORY third_party/fmi2/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
)

install(
  TARGETS physics_interface_plugin
  EXPORT export_physics_interface_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#============================================================================
# Testing
#============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # A second, test-only compilation of vessel_information.cpp to test conversion logic only
  add_library(vessel_information_test STATIC
    src/vessel_information.cpp
  )

  target_include_directories(vessel_information_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  target_link_libraries(vessel_information_test
    gz-sim8::gz-sim8
  )

  ament_target_dependencies(vessel_information_test
    lotusim_common
  )

  # Unit tests on the NED<->ENU conversions (pure functions living in
  # vessel_information.cpp). SKIP_LINKING_MAIN_LIBRARIES: the test provides
  # its own main() that calls _Exit() instead of returning (see
  # test_vessel_information.cpp for why).
  ament_add_gtest(test_ned_enu_conversions test/test_vessel_information.cpp
    SKIP_LINKING_MAIN_LIBRARIES
  )
  if(TARGET test_ned_enu_conversions)
    target_link_libraries(test_ned_enu_conversions
      vessel_information_test
      gz-sim8::gz-sim8
    )
  endif()

  # FMU loading and FmuInterface, against a small FMU built here
  # (test/fmu/test_vessel.c) and packed into an .fmu by the tests.
  enable_language(C)
  add_library(test_vessel_fmu MODULE test/fmu/test_vessel.c)
  target_include_directories(test_vessel_fmu PRIVATE third_party/fmi2)
  set_target_properties(test_vessel_fmu PROPERTIES PREFIX "" C_VISIBILITY_PRESET default)
  target_link_libraries(test_vessel_fmu m)

  add_library(fmu_interface_test STATIC
    src/fmi2_slave.cpp
    src/fmu_interface.cpp
    src/vessel_information.cpp
  )
  set_target_properties(fmu_interface_test PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(fmu_interface_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/fmi2>
  )
  find_package(spdlog REQUIRED)
  target_link_libraries(fmu_interface_test
    gz-sim8::gz-sim8
    spdlog::spdlog
    PkgConfig::LIBZIP
    tinyxml2::tinyxml2
    ${CMAKE_DL_LIBS}
  )
  ament_target_dependencies(fmu_interface_test
    lotusim_common
  )

  ament_add_gtest(test_fmu_interface test/test_fmu_interface.cpp
    SKIP_LINKING_MAIN_LIBRARIES
  )
  if(TARGET test_fmu_interface)
    add_dependencies(test_fmu_interface test_vessel_fmu)
    target_compile_definitions(test_fmu_interface PRIVATE
      TEST_FMU_LIBRARY="$<TARGET_FILE:test_vessel_fmu>")
    target_link_libraries(test_fmu_interface
      fmu_interface_test
      gz-sim8::gz-sim8
      PkgConfig::LIBZIP
    )
  endif()
endif()

#============================================================================
# Export
#============================================================================
ament_export_targets(export_physics_interface_plugin HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  lotusim_common
  lotusim_msgs
  yaml-cpp
  Boost
  websocketpp
  gz-sim8
  gz-plugin2
)

ament_package()