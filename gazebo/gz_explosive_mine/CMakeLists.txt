cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

project(ExplosiveMine VERSION 1.0.0)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

gz_configure_project()

gz_find_package(gz-math7 REQUIRED)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

gz_find_package(gz-common5
  REQUIRED
)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

gz_find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})


gz_find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
gz_find_package(gz-plugin2 REQUIRED)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

gz_find_package(sdformat14 REQUIRED)
set(SDF_VER ${sdformat14_VERSION_MAJOR})


add_library(${PROJECT_NAME} SHARED ./src/${PROJECT_NAME}.cc)
target_link_libraries(${PROJECT_NAME}
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})
#install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION $ENV{HOME}/.gz/sim/plugins
)
