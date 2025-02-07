cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

# Set the CMake policy if needed
if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

project(WindAdjusterPlugin)

# Enable Qt support for ROS 2
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED COMPONENTS gui)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Qml)

# Enable CMake auto-MOC for Qt
set(CMAKE_AUTOMOC ON)

# Add the Qt resources file
QT5_ADD_RESOURCES(resources_RCC ${PROJECT_NAME}.qrc)

# Define the shared library (plugin)
add_library(${PROJECT_NAME} SHARED
  WindAdjusterPlugin.cc
  ${resources_RCC}
)

# Link against necessary libraries
target_link_libraries(${PROJECT_NAME}
  PRIVATE gz-sim8::gui
  PRIVATE gz-transport13::gz-transport13
  PRIVATE gz-msgs10::gz-msgs10
  PRIVATE Qt5::Core
  PRIVATE Qt5::Widgets
  PRIVATE Qt5::Qml
)

# Install the plugin to the proper location
install(TARGETS ${PROJECT_NAME}
EXPORT  export_wind_adjuster_plugin
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

ament_package()
