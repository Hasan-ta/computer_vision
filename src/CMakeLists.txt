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

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/external/cmake ${CMAKE_MODULE_PATH})
set(ALTERNATE_EXTERNAL_DIR ${PROJECT_SOURCE_DIR}/../build/install)

find_package( Boost COMPONENTS system filesystem REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5Widgets REQUIRED)


set (KITTI_TOOLS_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/src/kitti_tools )

option(BUILD_CUSTOM_KITTI         "Compile custom kitti tools"        ON)
option(BUILD_ODOMETRY         		"Compile Odometry executables"      ON)
option(BUILD_LOGGER         			"Compile LOGGER"      							ON)
option(BUILD_OPENCV								"Compile opencv 3.1"								OFF)
option(BUILD_VTK									"Compile VTK"												OFF)


################################## Build External OpenCV ########################
include(ExternalProject)
set(EXTERNAL_TARGETS)


include(External_VTK) 

if(BUILD_VTK)
  list(APPEND EXTERNAL_TARGETS VTKProject)
endif()

include(External_OpenCV)

if(BUILD_OPENCV)
  list(APPEND EXTERNAL_TARGETS OpenCVProject)
endif()

if(NOT OpenCV_FOUND)
   message(ERROR "Failed to find OpenCV")
endif()

########################################################
### Specify to look in <build>/install/lib for libraries 
########################################################
link_directories("${PROJECT_BINARY_DIR}/install/lib")

########################################################
### Add custom target 'externals' that builds the externals
########################################################
add_custom_target(externals make -j8 ${EXTERNAL_TARGETS})


add_subdirectory(kitti_tools/custom_tools)
add_subdirectory(odometry/monocular)
add_subdirectory(utils/logger)
