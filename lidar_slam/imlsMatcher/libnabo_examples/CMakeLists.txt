cmake_minimum_required(VERSION 2.8.3)
project(libnabo)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(libnabo REQUIRED PATHS ${LIBNABO_INSTALL_DIR} )
include_directories(${libnabo_INCLUDE_DIRS})

set(LIB_NAME nabo)
set(EIGEN3_INCLUDE_DIR /usr/include/eigen3/)

add_executable(trivial trivial.cpp)
target_link_libraries(trivial ${LIB_NAME})

add_executable(usage usage.cpp)
target_link_libraries(usage ${LIB_NAME})