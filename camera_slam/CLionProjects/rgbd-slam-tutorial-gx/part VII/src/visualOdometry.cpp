/*************************************************************************
	> Created Time: 2015年08月01日 星期六 15时35分42秒
 ************************************************************************/
// 在运动时期，由于存在两张图像完全一样的情况，导致匹配时距离为0。
// 由于本节程序的设置，这种情况会被当成没有匹配，导致VO丢失。请你自己fix一下这个bug，

// 两两匹配，搭建一个视觉里程计。那么，这个里程计有什么不足呢？
// 1. 一旦出现了错误匹配，整个程序就会跑飞。
// 2. 误差会累积。常见的现象是：相机转过去的过程能够做对，但转回来之后则出现明显的偏差。
// 3. 效率方面不尽如人意。在线的点云显示比较费时。
// 累积误差是里程计中不可避免的，后续的相机姿态依赖着前面的姿态。
// 想要保证地图的准确，必须要保证每次匹配都精确无误，而这是难以实现的。
// 所以，希望用更好的方法来做slam。不仅仅考虑两帧的信息，而要把所有整的信息都考虑进来，成为一个全slam问题（full slam）。

#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "slamBase.h"

// 给定index，读取一帧数据
FRAME readFrame( int index, ParameterReader& pd );
// 度量运动的大小
double normofTransform( cv::Mat rvec, cv::Mat tvec );

int main( int argc, char** argv )
{
    ParameterReader pd;
    // atoi将string转换成整形integer
    int startIndex  =   atoi( pd.getData( "start_index" ).c_str() );
    int endIndex    =   atoi( pd.getData( "end_index"   ).c_str() );

    // initialize
    cout<<"Initializing ..."<<endl;
    int currIndex = startIndex; // 当前索引为currIndex
    // FRAME readFrame( int index, ParameterReader& pd ) 是读取帧数据的函数。
    // 告诉它我要读第几帧的数据，它就会乖乖的把数据给找出来，返回一个FRAME结构体。
    FRAME lastFrame = readFrame( currIndex, pd ); // 上一帧数据
    // 我们总是在比较currFrame和lastFrame
    string detector = pd.getData( "detector" );
    string descriptor = pd.getData( "descriptor" );
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    // 计算第一帧的关键点detector和描述子descriptor
    computeKeyPointsAndDesp( lastFrame, detector, descriptor );
    // 将rgb图和depth图转换为点云
    PointCloud::Ptr cloud = image2PointCloud( lastFrame.rgb, lastFrame.depth, camera );

    // 创建一个viewer, 创建GUI窗口,命名为viewer
    pcl::visualization::CloudViewer viewer("viewer");

    // 是否显示点云
    bool visualize = pd.getData("visualize_pointcloud")==string("yes");


    // 最小内点
    int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    // 最大运动误差
    double max_norm = atof( pd.getData("max_norm").c_str() );

    for ( currIndex=startIndex+1; currIndex<endIndex; currIndex++ )
    {
        cout<<"Reading files "<<currIndex<<endl;
        FRAME currFrame = readFrame( currIndex,pd ); // 读取currFrame
        // 计算当前帧的关键点和描述子
        computeKeyPointsAndDesp( currFrame, detector, descriptor );
        // 比较currFrame 和 lastFrame, 计算在当前帧坐标下的前一帧的变换
        RESULT_OF_PNP result = estimateMotion( lastFrame, currFrame, camera );
        // 去掉solvePnPRASNAC里，inlier较少的帧，同理定义为： min_inliers=5
        if ( result.inliers < min_inliers ) //inliers不够，放弃该帧
            continue;
        // 计算运动范围是否太大
        // 去掉求出来的变换矩阵太大的情况。
        // 因为假设运动是连贯的，两帧之间不会隔的太远：max_norm=0.3
        double norm = normofTransform(result.rvec, result.tvec);
        cout<<"norm = "<<norm<<endl;
        if ( norm >= max_norm )
            continue; // 运动过大,放弃该帧数据
        // 将旋转和平移合并为一个变换矩阵
        Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
        cout<<"T="<<T.matrix()<<endl;
        
        cloud = joinPointCloud( cloud, currFrame, T, camera );
        
        if ( visualize == true )
            viewer.showCloud( cloud );// 显示output指向的点云数据

        lastFrame = currFrame;
    }
    cerr << endl << "True" << endl;
    pcl::io::savePCDFile( "data/VO_result.pcd", *cloud );
    //pcl::io::savePCDFile( "/home/riki/CLionProjects/rgbd-slam-tutorial-gx/part VII/data/VO_result.pcd", *cloud );


    cout<<"Final result saved."<<endl;

    // 检查GUI是否被关闭, 用户可以自行关闭GUI
    while( !viewer.wasStopped() )
    {

    }
    return 0;
}

FRAME readFrame( int index, ParameterReader& pd )
{
    FRAME f;
    string rgbDir   =   pd.getData("rgb_dir");
    string depthDir =   pd.getData("depth_dir");
    
    string rgbExt   =   pd.getData("rgb_extension");
    string depthExt =   pd.getData("depth_extension");

    stringstream ss;
    ss<<rgbDir<<index<<rgbExt;
    string filename;
    ss>>filename;
    f.rgb = cv::imread( filename );

    ss.clear();
    filename.clear();
    ss<<depthDir<<index<<depthExt;
    ss>>filename;

    f.depth = cv::imread( filename, -1 );
    return f;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    // norm(delta_t) + min( 2*pi-norm(r), norm(r) )
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}