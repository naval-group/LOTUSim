cmake_minimum_required(VERSION 3.10)

project(lotusim)

#============================================================================
# Configure the project
#============================================================================
# Add support for C++17
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror=format-security -Wpedantic)
endif()

if(LOTUS_TIDY)
  set(CMAKE_CXX_CLANG_TIDY "clang-tidy;-checks=-*,-header-filter=.*, clang-analyzer-*,modernize-*,performance-*,hicpp-*,concurrency-*,bugprone-*,portability-*,cppcoreguidelines-*")
endif()


find_package(CLI11 REQUIRED)
find_package(Threads REQUIRED)

add_executable(lotusim src/lotusim.cpp)
target_link_libraries(lotusim CLI11::CLI11)

install(
  TARGETS lotusim
  EXPORT  export_lotusim
  ARCHIVE DESTINATION lib 
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY 
    ../assets 
    ../physics
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    src/bash_completion.sh
  DESTINATION share/${PROJECT_NAME}
)

