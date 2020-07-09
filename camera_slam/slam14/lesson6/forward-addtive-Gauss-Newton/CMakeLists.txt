cmake_minimum_required( VERSION 2.8 )
project( optical_flow )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3" )

#OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
#Eigen3
include_directories( "/usr/include/eigen3" )

add_executable( ${PROJECT_NAME} optical_flow.cpp)
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
