cmake_minimum_required(VERSION 2.8)
#set(CMAKE_VERBOSE_MAKEFILE 1) # to see all make process

project( icp_test )

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (icp_test icp_test.cpp)
target_link_libraries (icp_test ${PCL_LIBRARIES})
