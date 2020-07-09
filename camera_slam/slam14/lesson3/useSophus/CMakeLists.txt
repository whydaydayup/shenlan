cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")

project(useSophus)

#include_directories("/usr/include/eigen3")
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME} useSophus.cpp)
target_link_libraries( useSophus ${Sophus_LIBRARIES} )
