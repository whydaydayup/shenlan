cmake_minimum_required(VERSION 2.8)
project(gaussnewton)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#Eigen3
include_directories("/usr/include/eigen3")
#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIRS})

add_executable(${PROJECT_NAME} gaussnewton.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
