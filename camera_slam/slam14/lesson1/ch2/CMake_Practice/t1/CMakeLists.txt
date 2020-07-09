cmake_minimum_required(VERSION 2.8)
project(HELLO)
set(SRC_LIST main.cpp)
message(STATUS "This is BINARY dir" ${PROJECT_BINARY_DIR})
message(STATUS "This is SOURCE dir" ${PROJECT_SOURCE_DIR})
add_executable(hello ${SRC_LIST})

