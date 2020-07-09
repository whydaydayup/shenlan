cmake_minimum_required( VERSION 2.8 )

project( ICP )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

#Pangolin
find_package( Pangolin REQUIRED )
include_directories( ${Pangolin_INCLUDE_DIRS} )
#Sophus
find_package( Sophus REQUIRED )
include_directories( 
	"/usr/include/eigen3"   #Eigen3
	${Sophus_INCLUDE_DIRS}  #Sophus
)
#OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( ${PROJECT_NAME} main.cpp)
target_link_libraries( ${PROJECT_NAME} 
	${Sophus_LIBRARIES}
	${Pangolin_LIBRARIES}
		${OpenCV_LIBS}
)
