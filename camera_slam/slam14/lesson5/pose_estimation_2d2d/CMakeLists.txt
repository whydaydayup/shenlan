cmake_minimum_required( VERSION 2.8 )
project( pose_estimation_2d2d )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
#OpenCV
find_package( OpenCV 3.0 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( ${PROJECT_NAME} pose_estimation_2d2d.cpp )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
