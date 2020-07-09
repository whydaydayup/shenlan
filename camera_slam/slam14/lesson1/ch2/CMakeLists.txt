cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE "Debug")

project(helloSLAM) # 声明一个工程
include_directories("include") #

add_library(hello_shared SHARED src/libHelloSLAM.cpp)
# 添加可执行文件 (程序名  源代码文件)
add_executable(helloSLAM helloSLAM.cpp)
target_link_libraries(helloSLAM hello_shared)