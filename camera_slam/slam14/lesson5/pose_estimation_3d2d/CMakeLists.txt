cmake_minimum_required( VERSION 2.8 )
project( pose_estimation_3d_2d )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
# 添加cmake模块以使用g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
#OpenCV
find_package( OpenCV 3 REQUIRED )
# find_package( OpenCV REQUIRED ) # use this if in OpenCV2
include_directories( ${OpenCV_INCLUDE_DIRS} )
#CSparse
find_package( CSparse REQUIRED )
    # CSPARSE_INCLUDE_DIR!!!! NOT DIRS
include_directories( ${CSPARSE_INCLUDE_DIR} ) # sparse稀疏
#g2o
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

#Eigen3
include_directories( "/usr/include/eigen3" )

add_executable( ${PROJECT_NAME} pose_estimation_3d2d.cpp )
target_link_libraries( ${PROJECT_NAME} 
	${OpenCV_LIBS}
    ${CSPARSE_LIBRARY}
	g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
)