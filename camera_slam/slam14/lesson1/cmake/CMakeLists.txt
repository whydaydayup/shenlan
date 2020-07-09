cmake_minimum_required( VERSION 2.5 )
#设置编译模式“Release”模式、"Debug"模式
set( CMAKE_BUILD_TYPE "Release")
project(helloSLAM)
# 添加多个特定头文件搜索路径
include_directories(include)
# add_subdirectory(src bin) #存放源文件的子目录 中间二进制和目标文件存放的位置
# add_subdirectory(src)

add_library(hello SHARED src/hello.cpp)
install(TARGETS hello LIBRARY DESTINATION lib)
add_executable(sayHello useHello.cpp)
target_link_libraries(sayHello hello)
install(FILES include/hello.h DESTINATION include)
