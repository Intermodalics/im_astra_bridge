# This package is used within a docker build environment that
# provides an orbbec astra sdk.
#
# Copyright: malte.ahl@intermodalics.eu

cmake_minimum_required(VERSION 2.8.3)
project(im_astra_bridge)


# Check c++11 / c++0x support.
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
  message(FATAL_ERROR "Compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
endif()

list(APPEND
  CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config
)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS  
  roscpp
  sensor_msgs
)

## Third-party packages ##
find_package(PCL REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Astra REQUIRED)

if(NOT ASTRA_ROOT)
  message(STATUS "No orbbec astra sdk, try -DASTRA_ROOT=../astra/build/sdk")
  return()
endif()

catkin_package(INCLUDE_DIRS include LIBRARIES fov_checker_functions) 

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${ASTRA_INCLUDE_DIR}
)

## Declare a C++ library and link with its dependencies
add_library(fov_filter src/fov_filter.cpp )

target_link_libraries(fov_filter
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
)

## Declare a C++ executable
add_executable(im_astra_bridge_node 
    src/im_astra_bridge_node.cpp
)

## Add cmake target dependencies.
add_dependencies(im_astra_bridge_node 
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(im_astra_bridge_node
   ${catkin_LIBRARIES}
   fov_filter
   ${ASTRA_LIBRARIES}
)
