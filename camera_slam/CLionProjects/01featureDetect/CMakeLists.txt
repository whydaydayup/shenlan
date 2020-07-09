cmake_minimum_required( VERSION 2.8 )
project( featureDetect )
set( CMAKE_BUILD_TYPE "Release" )
# -O2 、 -O3编译优化
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
#OpenCV
#SET(OpenCV_DIR /usr/local/opencv300/share/OpenCV)
find_package( OpenCV 4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( ${PROJECT_NAME} featureDetect.cpp)
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )