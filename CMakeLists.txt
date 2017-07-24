cmake_minimum_required(VERSION 2.8)
project(computer_vision CXX C)

set(SUPERBUILD_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Debug" CACHED)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "-Ofast -msse4 -DNDEBUG")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "-g")
endif()

find_package( OpenCV REQUIRED )
find_package( Boost COMPONENTS system filesystem REQUIRED)
find_package(Eigen3 REQUIRED)

set (PARENT_SRC_DIR ${CMAKE_SOURCE_DIR}/src)

set (KITTI_TOOLS_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/src/kitti_tools )

option(BUILD_CUSTOM_KITTI         "Compile custom kitti tools"        ON)
option(BUILD_ODOMETRY         		"Compile Odometry executables"      ON)
option(BUILD_LOGGER         			"Compile LOGGER"      							ON)

add_subdirectory(src)
