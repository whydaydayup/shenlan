/*************************************************************************
	> File Name: src/jointPointCloud.cpp
	> Author: Xiang gao
	> Mail: gaoxiang12@mails.tsinghua.edu.cn 
	> Created Time: 2015年07月22日 星期三 20时46分08秒
 ************************************************************************/
// 只有两帧的SLAM程序,已经是一个视觉里程计(Visual Odometry)
// 只要不断地把进来的数据与上一帧对比，就可以得到完整的运动轨迹以及地图
// 后期需要先讲讲关键帧的处理，因为把每个图像都放进地图，会导致地图规模增长地太快，所以需要关键帧技术。
// 然后要用g2o做一个SLAM后端

#include<iostream>
using namespace std;

#include "slamBase.h"

#include <opencv2/core/eigen.hpp>

#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

// Eigen !
#include <Eigen/Core>
#include <Eigen/Geometry>

int main( int argc, char** argv )
{
    //本节要拼合data中的两对图像
    ParameterReader pd;
    // 声明两个帧，FRAME结构请见include/slamBase.h
    FRAME frame1, frame2;
    
    //读取图像
    frame1.rgb = cv::imread( "./data/rgb1.png" );
    frame1.depth = cv::imread( "./data/depth1.png", -1);
    frame2.rgb = cv::imread( "./data/rgb2.png" );
    frame2.depth = cv::imread( "./data/depth2.png", -1 );

    // 提取特征并计算描述子
    cout<<"extracting features"<<endl;
    string detecter = pd.getData( "detector" );
    string descriptor = pd.getData( "descriptor" );

    computeKeyPointsAndDesp( frame1, detecter, descriptor );
    computeKeyPointsAndDesp( frame2, detecter, descriptor );

    // 相机内参
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );

    cout<<"solving pnp"<<endl;
    // 求解pnp
    RESULT_OF_PNP result = estimateMotion( frame1, frame2, camera );
        // 旋转矩阵R，虽然有3×3那么大，自由变量却只有三个，不够节省空间。
        // 所以在OpenCV里使用了一个向量来表达旋转。向量的方向是旋转轴，大小则是转过的弧度.
    cout<<result.rvec<<endl<<result.tvec<<endl;

    // 处理result
    // 将旋转向量转化为旋转矩阵
    cv::Mat R;
    // 用罗德里格斯变换（Rodrigues）将旋转向量转换为矩阵cv::Mat R
    cv::Rodrigues( result.rvec, R );
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r); // 将cv中的矩阵cv::Mat R转换为eigen中的3x3的矩阵
  
    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    cout<<"translation"<<endl;
    Eigen::Translation<double,3> trans(result.tvec.at<double>(0,0), result.tvec.at<double>(0,1), result.tvec.at<double>(0,2));
    T = angle;
    T(0,3) = result.tvec.at<double>(0,0); 
    T(1,3) = result.tvec.at<double>(0,1); 
    T(2,3) = result.tvec.at<double>(0,2);

    // 转换点云
    cout<<"converting image to clouds"<<endl;
    // 前后两帧的点云数据
    PointCloud::Ptr cloud1 = image2PointCloud( frame1.rgb, frame1.depth, camera );
    PointCloud::Ptr cloud2 = image2PointCloud( frame2.rgb, frame2.depth, camera );

    // 合并点云
    cout<<"combining clouds"<<endl;
    PointCloud::Ptr output (new PointCloud());
    // 变换矩阵结合了旋转和缩放，是一种较为经济实用的表达方式。
    // 点云的变换函数，只要给定了变换矩阵，就能对移动整个点云
        // 将第一帧的点云数据*cloud1进行变换矩阵变化, 转换到第二帧的坐标下, 并保存到*output下
    pcl::transformPointCloud( *cloud1, *output, T.matrix() );
    Eigen::Matrix<double, 4, 4> T_Transform = T.matrix();
    cout << T_Transform << endl;
    *output += *cloud2; // 将变换后的第一帧点云数据+第二帧点云数据
    pcl::io::savePCDFile("data/result.pcd", *output);
    cout<<"Final result saved."<<endl;

    // 创建GUI窗口,命名为viewer
    pcl::visualization::CloudViewer viewer( "viewer" );
    viewer.showCloud( output ); // 显示output指向的点云数据
    // 检查GUI是否被关闭, 用户可以自行关闭GUI
    while( !viewer.wasStopped() )
    {

    }
    return 0;
}