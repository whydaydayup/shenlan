/*************************************************************************
	> File Name: src/slamBase.cpp
	> Author: xiang gao
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
    > Implementation of slamBase.h
	> Created Time: 2015年07月18日 星期六 15时31分49秒
 ************************************************************************/
// Ubuntu16下OpenCV一些报错和操作
// https://blog.csdn.net/sunshinefcx/article/details/83859408



#include "slamBase.h"
// 将rgb图和depth图转换为点云
// 图像矩阵cv::Mat& rgb, cv::Mat& depth
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    // typedef pcl::PointCloud<PointT> PointCloud;
    // 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    PointCloud::Ptr cloud ( new PointCloud );

    // 遍历深度图
//    for (int m = 0; m < depth.rows; m+=2)
//        for (int n=0; n < depth.cols; n+=2)
    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            // 使用depth.ptr<ushort>(m)[n]来获取深度值
            // cv::Mat的ptr函数会返回指向该图像第m行数据的头指针。
            // 然后加上位移n后，这个指针指向的数据就是我们需要读取的数据
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            // 关于图像上下翻转问题，是因为opencv定义的坐标系和pcl_viewer显示坐标系不同，
            // opencv是x右y下，而pcl显示是x右y上。
            // 解决方法：找到群主程序image2PointCloud函数中，
            // 把计算点空间坐标的公式的p.y值添加负号，这样y方向就可以正常显示了
            // p.y = (m - camera.cy) * p.z / camera.fy;
            p.y = -(m - camera.cy) * p.z / camera.fy;
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}
// point2dTo3d 将单个点从图像坐标转换为空间坐标
// input: 3维点Point3f (u,v,d)
cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    cv::Point3f p; // 3D 点
    p.z = double( point.z ) / camera.scale;
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;
    return p;
}

// computeKeyPointsAndDesp 同时提取关键点与特征描述子
void computeKeyPointsAndDesp( FRAME& frame, string detector, string descriptor )
{
//    cv::Ptr<cv::FeatureDetector> _detector;
//    cv::Ptr<cv::DescriptorExtractor> _descriptor;
//
//    _detector = cv::FeatureDetector::create( detector.c_str() );
//    _descriptor = cv::DescriptorExtractor::create( descriptor.c_str() );
//
//    if (!_detector || !_descriptor)
//    {
//        cerr<<"Unknown detector or discriptor type !"<<detector<<","<<descriptor<<endl;
//        return;
//    }
//
//    _detector->detect( frame.rgb, frame.kp );
//    _descriptor->compute( frame.rgb, frame.kp, frame.desp );

    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    orb->detect( frame.rgb, frame.kp );  //提取关键点
    orb->compute( frame.rgb, frame.kp, frame.desp );
    // cout << frame.desp.size() << endl;

    return;
}

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2
// 输出：rvec 和 tvec
RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    static ParameterReader pd;
    vector< cv::DMatch > matches;
    cv::BFMatcher matcher;
    matcher.match( frame1.desp, frame2.desp, matches );
   
    RESULT_OF_PNP result;
    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    double good_match_threshold = atof( pd.getData( "good_match_threshold" ).c_str() );
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    if ( minDis < 10 ) 
        minDis = 10;
    
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < good_match_threshold*minDis)
            goodMatches.push_back( matches[i] );
    }


    if (goodMatches.size() <= 5) 
    {
        result.inliers = -1;
        return result;
    }
    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;

    // 相机内参
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( frame2.kp[goodMatches[i].trainIdx].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, camera );
        pts_obj.push_back( pd );
    }

    if (pts_obj.size() ==0 || pts_img.size()==0)
    {
        result.inliers = -1;
        return result;
    }

    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // 求解pnp
    // opencv2
    // cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers );
    // opencv3
    // objectPoints一组匹配好的三维点: pts_obj世界坐标系下的3D点,而不是图像中2D点加上深度转换成世界坐标中的3D点
    // imagePoints一组二维图像点: pts_img
    // 返回的结果是旋转向量 rvec 和平移向量tvec
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers );

    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;

    return result;
}


// cvMat2Eigen
// 旋转矩阵R，虽然有3×3那么大，自由变量却只有三个，不够节省空间。
// 所以在OpenCV里使用了一个向量来表达旋转。向量的方向是旋转轴，大小则是转过的弧度.
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec )
{
    cv::Mat R;
    // 用罗德里格斯变换（Rodrigues）将旋转向量转换为矩阵cv::Mat R
    cv::Rodrigues( rvec, R );
    // 将cv::Mat转换成Eigen 3x3的矩阵
    Eigen::Matrix3d r;
    for ( int i=0; i<3; i++ )
        for ( int j=0; j<3; j++ ) 
            r(i,j) = R.at<double>(i,j);
  
    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    T = angle; // 变换矩阵T的旋转部分使用r初始化
    // 变换矩阵的平移部分使用tvec初始化
    T(0,3) = tvec.at<double>(0,0); 
    T(1,3) = tvec.at<double>(1,0); 
    T(2,3) = tvec.at<double>(2,0);
    return T;
}

// joinPointCloud 
// 输入：原始点云，新来的帧以及它的位姿
// 输出：将新来帧加到原始帧后的图像(将新的帧合并到旧的点云里)
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera ) 
{
    PointCloud::Ptr newCloud = image2PointCloud( newFrame.rgb, newFrame.depth, camera );

    // 合并点云
    PointCloud::Ptr output (new PointCloud());
    // 将original图转换成新帧newFrame的坐标下,
    pcl::transformPointCloud( *original, *output, T.matrix() );
    *newCloud += *output;// 将转换后的老的数据和新的数据组合起来

    // Voxel grid 滤波降采样???干啥的??
    static pcl::VoxelGrid<PointT> voxel;
    static ParameterReader pd;
    double gridsize = atof( pd.getData("voxel_grid").c_str() );
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( newCloud );
    PointCloud::Ptr tmp( new PointCloud() );
    voxel.filter( *tmp );
    return tmp;
}
