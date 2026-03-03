cmake_minimum_required(VERSION 3.8)
project(lotusim_environment)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(lotusim_common REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/weather/wind_model.cpp
  src/weather/wave_model.cpp
  src/weather/sea_state.cpp
  src/weather/weather_conditions.cpp
  src/ocean/ocean_model.cpp
  src/propagation/radar_propagation.cpp
  src/propagation/acoustic_propagation.cpp
  src/propagation/optical_propagation.cpp
  src/vehicle/seakeeping_model.cpp
  src/vehicle/operational_effects.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  lotusim_common
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  fmt::fmt
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_environment test/test_environment.cpp)
  target_link_libraries(test_environment ${PROJECT_NAME})

  ament_add_gtest(test_integration_scenarios test/test_integration_scenarios.cpp)
  target_link_libraries(test_integration_scenarios ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  lotusim_common
)

ament_package()
