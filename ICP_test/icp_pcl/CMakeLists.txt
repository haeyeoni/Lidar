cmake_minimum_required(VERSION 2.8)
#set(CMAKE_VERBOSE_MAKEFILE 1) # to see all make process

project( icp_test )

find_package(PCL 1.8 REQUIRED)

#add_include_directories("/opt/ros/melodic/include")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (pcltopcl2 pcltopcl2.cpp)
add_executable (icp_test icp_test.cpp)
add_executable (warp_rigid warp_rigid.cpp)
#add_executable (narf_keypoint_extraction narf_keypoint_extraction.cpp)

#target_link_libraries (pcltopcl2 ${PCL_LIBRARIES})
target_link_libraries (icp_test ${PCL_LIBRARIES})
target_link_libraries (warp_rigid ${PCL_LIBRARIES})
#target_link_libraries (narf_keypoint_extraction ${PCL_LIBRARIES})