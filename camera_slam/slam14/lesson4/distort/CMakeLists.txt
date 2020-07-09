cmake_minimum_required(VERSION 2.8)
project(undistort_image)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} undistort_image.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} )
