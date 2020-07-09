# 库是一种可执行代码的二进制形式，可以被操作系统载入内存执行。
# 库有两种: 静态库（.a/.lib）和动态库（.so/ .dll）
# libName.a    libName.so

# 生成库,SHARED动态库,libmyslam.so
add_library( myslam SHARED
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    g2o_types.cpp
    visual_odometry.cpp
)
# 链接库${THIRD_PARTY_LIBS}到库文件myslam上
target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)