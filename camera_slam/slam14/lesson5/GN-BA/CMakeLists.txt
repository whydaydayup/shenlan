cmake_minimum_required( VERSION 2.8 )
project( GN-BA )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

#eigen3
include_directories( "/usr/include/eigen3" )
#Sophus
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS}  )

add_executable( ${PROJECT_NAME} GN-BA.cpp )
target_link_libraries( ${PROJECT_NAME} ${Sophus_LIBRARIES} )

