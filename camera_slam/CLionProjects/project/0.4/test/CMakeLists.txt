# 添加可执行文件,(生成为文件名   源文件.cpp)
add_executable( run_vo run_vo.cpp )
# 链接库libmyslam到执行文件run_vo
target_link_libraries( run_vo myslam )