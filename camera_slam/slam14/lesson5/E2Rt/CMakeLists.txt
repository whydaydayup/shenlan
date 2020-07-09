#Eigen3
#include_directories( "/usr/include/eigen3" )
#LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
#find_package( Eigen3 REQUIRED )

#include_directories( "/usr/local/include/sophus" )
#link_libraries("/usr/local/lib/libSophus.so")

#Sophus
#find_package( Sophus REQUIRED )
#include_directories( ${Sophus_INCLUDE_DIRS} )

cmake_minimum_required( VERSION 2.8 )

project( E2Rt )

set( CMAKE_BUILD_TYPE "Release" )
#是 O3 不是 03
#set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package( Sophus REQUIRED)

include_directories(
        "/usr/include/eigen3"
        # "/usr/include/sophus"
        ${Sophus_INCLUDE_DIRS})

add_executable( ${PROJECT_NAME} main.cpp)
target_link_libraries( ${PROJECT_NAME}
        ${Sophus_LIBRARIES})
