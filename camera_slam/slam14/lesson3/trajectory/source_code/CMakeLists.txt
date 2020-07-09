cmake_minimum_required( VERSION 2.8 )

project(trajectory.txt)

set( CMAKE_BUILD_TYPE "Debug" )
#是 O3 不是 03
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )


find_package( Sophus REQUIRED)
find_package( Pangolin REQUIRED)


include_directories(
        "/usr/include/eigen3"
        ${Sophus_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        )

add_executable( draw_trajectory draw_trajectory.cpp)

target_link_libraries( draw_trajectory ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} )