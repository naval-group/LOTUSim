cmake_minimum_required(VERSION 3.22)
project(lotusim_common VERSION 0.0.1 LANGUAGES CXX)

#============================================================================
# Project setup
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=format-security)
endif()

# Optional clang-tidy
if(LOTUS_TIDY)
  set(CMAKE_CXX_CLANG_TIDY
      "clang-tidy;-checks=-*,clang-analyzer-*,modernize-*,performance-*,hicpp-*,concurrency-*,bugprone-*,portability-*,cppcoreguidelines-*;-header-filter=.*")
endif()

# Debugging
if(DEBUG)
  message(STATUS "Building ${PROJECT_NAME} with debug options ${DEBUG}")
  add_compile_options(-g -Og)
  add_definitions(-DDEBUG=${DEBUG})
endif()

# Profiling
if(BUILD_PROFILING)
  add_compile_options(-pg -g -Og)
endif()

#============================================================================
# Dependencies
#============================================================================
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(gz-sim8 REQUIRED)

#============================================================================
# Library
#============================================================================
add_library(${PROJECT_NAME} SHARED
  src/logger.cpp
  src/common.cpp
  src/entity_group.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  stdc++fs
  gz-sim8::gz-sim8
)

ament_target_dependencies(${PROJECT_NAME}
  std_msgs
  spdlog
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
  PATTERN ".~" EXCLUDE
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#============================================================================
# Ament export
#============================================================================
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  std_msgs
  spdlog
  gz-sim8
)
ament_export_include_directories(include)

ament_package()