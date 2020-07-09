cmake_minimum_required(VERSION 2.8)
project(joinMap)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

#OpenCV
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
#Eigen3
include_directories( "/usr/include/eigen3" )
#pcl
find_package(PCL REQUIRED COMPONENT common io)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_executable( ${PROJECT_NAME} joinMap.cpp )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES} )
