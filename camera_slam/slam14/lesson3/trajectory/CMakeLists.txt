cmake_minimum_required( VERSION 2.8 )

project(trajectory)

set( CMAKE_BUILD_TYPE "Release" )
#是 O3 不是 03
#set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( Sophus REQUIRED)
find_package( Pangolin REQUIRED)

include_directories(
        "/usr/include/eigen3"
       # "/usr/include/sophus"
        ${Sophus_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS})

add_executable( draw_trajectory compare.cpp)
target_link_libraries( draw_trajectory
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} )