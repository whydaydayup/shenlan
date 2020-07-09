cmake_minimum_required( VERSION 2.8 ) #设定版本
project ( myslam ) # 设定工程名

set( CMAKE_CXX_COMPILER "g++" ) #设定编译器
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
# 设定可执行二进制文件的目录, 二进制就是可以直接运行的程序
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
# 设定存放编译出来的库文件的目录,库文件是为这些二进制提供函数
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )
# 有main函数的代码可以编译成二进制，其他的则编译成库文件。链接时，把库文件链到二进制上

############### dependencies ######################
# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV 3.1 REQUIRED ) # 找OpenCVC
include_directories( ${OpenCV_INCLUDE_DIRS} ) # 添加头文件目录
# Sophus
#set(Sophus_DIR ${PROJECT_SOURCE_DIR}/Sophus)
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
# G2O
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba
)
############### dependencies ######################
# 设定头文件目录
include_directories( ${PROJECT_SOURCE_DIR}/include )

# 添加一个文件夹进行编译，
# 该文件夹下的CMakeLists.txt 负责编译该文件夹下的源码.　
add_subdirectory( src ) # 使用的是相对路径
add_subdirectory( test )