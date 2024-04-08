
find_package(PkgConfig)

PROTOBUF_GENERATE_CPP(PROTO_PRIVATE_SRC PROTO_PRIVATE_HEADERS
  wavemsg.proto
  aismsg.proto
  xdyncmdmsg.proto
)

include(GNUInstallDirs)

set(PROTO_PRIVATE_SRC ${PROTO_PRIVATE_SRC} PARENT_SCOPE)
set(PROTO_PRIVATE_HEADERS ${PROTO_PRIVATE_HEADERS} PARENT_SCOPE)

install(
  DIRECTORY msg
  DESTINATION include/gz_liquidai_msgs
  COMPONENT proto
  FILES_MATCHING PATTERN "*.proto"
)

install(
  FILES ${PROTO_PRIVATE_HEADERS}
  DESTINATION include/gz_liquidai_msgs/msg
)

# message("PROJECT_BINARY_DIR ${PROJECT_BINARY_DIR}")
# message("CMAKE_INSTALL_DATAROOTDIR ${CMAKE_INSTALL_DATAROOTDIR}")
# message("CMAKE_CURRENT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}")
# message("CMAKE_CURRENT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}")
# message("CMAKE_INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR}")
# message("CMAKE_FILES_DIRECTORY ${CMAKE_FILES_DIRECTORY}")
# message("CMAKE_INSTALL_FULL_LIBDIR ${CMAKE_INSTALL_FULL_LIBDIR}")
# message("CMAKE_INSTALL_FULL_INCLUDEDIR ${CMAKE_INSTALL_FULL_INCLUDEDIR}")

add_library(gz_liquidai_msgs 
  ${PROTO_PRIVATE_SRC} 
  ${PROTO_PRIVATE_HEADERS}
)

include(GenerateExportHeader)
generate_export_header(gz_liquidai_msgs)

target_link_libraries(gz_liquidai_msgs
  PUBLIC
    protobuf::libprotobuf
    gz-sim7::gz-sim7      
)

target_include_directories(gz_liquidai_msgs
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/gz_liquidai_msgs>
    $<BUILD_INTERFACE:${CMAKE_INSTALL_FULL_INCLUDEDIR}>
    $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR_FULL}>)

install(
  TARGETS gz_liquidai_msgs
  EXPORT export_gz_liquidai_msgs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

include(CMakePackageConfigHelpers)

set_and_check(GZ_LIQUIDAI_MSGS_INCLUDE_INSTALL_DIR_FULL "${CMAKE_INSTALL_FULL_INCLUDEDIR}")

message("msg INCLUDE dir    ${GZ_LIQUIDAI_MSGS_INCLUDE_INSTALL_DIR_FULL}")

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/gz_liquidai_msgsConfig.cmake.in
    ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/gz_liquidai_msgsConfig.cmake
    INSTALL_DESTINATION lib/cmake/gz_liquidai_msgs
    PATH_VARS GZ_LIQUIDAI_MSGS_INCLUDE_INSTALL_DIR_FULL)

write_basic_package_version_file(
  ${PROJECT_BINARY_DIR}/gz_liquidai_msgsConfigVersion.cmake
  COMPATIBILITY AnyNewerVersion
)

install(
  FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/gz_liquidai_msgsConfig.cmake"
    "${PROJECT_BINARY_DIR}/gz_liquidai_msgsConfigVersion.cmake"
  DESTINATION lib/cmake/gz_liquidai_msgs
)

export(
  EXPORT export_gz_liquidai_msgs
  FILE "${PROJECT_BINARY_DIR}/cmake/gz_liquidai_msgsTargets.cmake"
)

install(
  EXPORT export_gz_liquidai_msgs
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gz_liquidai_msgs
  FILE gz_liquidai_msgsTargets.cmake
)

