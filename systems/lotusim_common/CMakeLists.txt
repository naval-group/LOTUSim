cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(lotusim_common VERSION 0.0.1  LANGUAGES CXX)

#============================================================================
# Configure the project
#============================================================================
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror=format-security -Wpedantic)
endif()

if(LOTUS_TIDY)
  set(CMAKE_CXX_CLANG_TIDY "clang-tidy;-checks=-*,-header-filter=.*, clang-analyzer-*,modernize-*,performance-*,hicpp-*,concurrency-*,bugprone-*,portability-*,cppcoreguidelines-*")
endif()

#============================================================================
# Set project-specific options
#============================================================================
# For debugging
if(DEBUG)
  message("Building ${PROJECT_NAME} with debug options ${DEBUG}")
  add_compile_options(-g -Og)
  find_package(backward_ros)
  # Used to define the logger level
  add_definitions(-DDEBUG=${DEBUG} )
endif()

# For profiling
if(BUILD_PROFILING)
  add_compile_options(-pg -g -Og)
endif()

if(NOT DEFINED BUILD_SHARED_LIBS)
  option(BUILD_SHARED_LIBS "Build dynamically-linked binaries" ON)
endif()

# Code coverage
if(BUILD_TESTING)
    add_compile_options(-fprofile-arcs -ftest-coverage -g -Og)
endif()

#============================================================================
# Configure the package to be installed
#============================================================================
include(CMakePackageConfigHelpers)
# Create the ${PROJECT_NAME}Config.cmake.
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in
  "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY
)

# Create the ${PROJECT_NAME}ConfigVersion.cmake.
write_basic_package_version_file(
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY AnyNewerVersion
)

#============================================================================
# Search for project-specific dependencies
#============================================================================
find_package(spdlog REQUIRED)
find_package(PkgConfig)

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

#============================================================================
# Create and install project files
#============================================================================
add_library(${PROJECT_NAME}
  src/logger.cpp
  src/common.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}>"
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>
)

message(CMAKE_INSTALL_LIBDIR: ${CMAKE_INSTALL_LIBDIR})
message(CMAKE_INSTALL_BINDIR: ${CMAKE_INSTALL_BINDIR})


target_link_libraries(${PROJECT_NAME} PRIVATE 
  ${Boost_SYSTEM_LIBRARY}
  stdc++fs
  gz-sim8::gz-sim8  
)

# Code coverage
# if(BUILD_TESTING)
#     target_link_libraries(${PROJECT_NAME} PUBLIC gcov)
# endif()

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_export.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME})

#============================================================================
# Install the files for this package
#============================================================================
install(
  FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

# Export targets.
set(export_targets ${export_targets};${PROJECT_NAME})
export(
  EXPORT export_${PROJECT_NAME}
  FILE "${PROJECT_BINARY_DIR}/cmake/${PROJECT_NAME}Targets.cmake"
  NAMESPACE ${PROJECT_NAME}::
)

install(
  EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)