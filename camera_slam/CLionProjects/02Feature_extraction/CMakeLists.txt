cmake_minimum_required( VERSION 2.8 )
project( featureMatch )
set( CMAKE_BUILD_TYPE "Release" )
# -O2 、 -O3编译优化
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
#OpenCV
find_package( OpenCV 3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
message(STATUS " OpenCV library status:")
message(STATUS " version: ${OpenCV_VERSION}")
message(STATUS " libraries: ${OpenCV_LIBS}")
message(STATUS " include path: ${OpenCV_INCLUDE_DIRS}")

add_executable( ${PROJECT_NAME} featureMatch.cpp)
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
