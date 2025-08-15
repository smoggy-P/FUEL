cmake_minimum_required(VERSION 2.8.3)
project(trajectory_bridge)

set(CMAKE_VERBOSE_MAKEFILE "true")
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  quadrotor_msgs
  agiros_msgs
)

catkin_package()

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(trajectory_bridge_node src/trajectory_bridge_node.cpp)

target_link_libraries(trajectory_bridge_node
   ${catkin_LIBRARIES}
) 