#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// #include "extra.h" // used in opencv2
using namespace std;
using namespace cv;

void find_feature_matches(
        const Mat& img_1, const Mat& img_2, // 要加const以及&引用符号
        std::vector<KeyPoint>& keypoints_1, // 要加std::范围符号以及&引用符号
        std::vector<KeyPoint>& keypoints_2,
        std::vector< DMatch >& matches )
{
    // 寻找关键点keypoints --> 描述子descriptors --> 匹配matches --> 去除不好的匹配
    // 初始化
    // 定义描述子
    Mat descriptors_1, descriptors_2;
    // opencv3
    // 特征检测器(Feature Detector)寻找特征点的关键点
    Ptr<FeatureDetector> detector = ORB::create();
    // 描述子提取器(Decriptor Extractor)寻找特征点的描述子
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // opencv2
    //Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    //Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    // 描述子匹配器(Descriptor Matcher)
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create( "BruteForce-Hamming");
    //-- 1、检测 Oriented FAST 角点位置（特征点的关键点）
    detector->detect( img_1, keypoints_1 );
    detector->detect( img_2, keypoints_2 );
    //-- 2、根据角点位置计算周围的 BRIEF 描述子
    descriptor->compute( img_1, keypoints_1, descriptors_1 );
    descriptor->compute( img_2, keypoints_2, descriptors_2 );
    //-- 3、对两幅图中的特征点 BRIEF 描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    matcher->match(descriptors_1, descriptors_2, match );
    //-- 4、匹配点筛选
    // 找出所有匹配点中最小和最大距离，即最相似和最不相似的两组匹配点之间的距离
    double min_dist=10000, max_dist=0;
    // 对于图一中的每一个描述子，在图二中都能找到相对应的描述子与其对应
    // match.size() == descriptors_1.rows
    //for ( int i = 0; i < match.size(); i++ )
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist > max_dist ) max_dist = dist;
        if ( dist < min_dist ) min_dist = dist;
    }
    printf ( "-- Max dist : %f \n", max_dist );
    printf (" -- Min dist : %f \n", min_dist );

    // 当描述子之间的距离大于两倍的最小距离的时候，认为匹配有误
    // 但有时候最小距离会非常小，设置一个经验值30作为下限
    // 遍历vector<DMatch> match中的所有匹配对
    // for ( int i = 0; i < match.size(); i++ )
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        // 30.0一定要加0，表示浮点类型，不然max函数重载找不到对应函数
        if ( match[i].distance <= max( 2*min_dist, 30.0 ) )
            matches.push_back( match[i] );
    }
}

void pose_estimation_2d2d (
        // 要加std::区域作用符号，&引用变量， const常量，不允许修改
        const std::vector<KeyPoint>& keypoints_1,
        const std::vector<KeyPoint>& keypoints_2,
        const std::vector< DMatch >& matches,
        Mat& R, Mat& t // R, t 引用赋值
        )
{
    // 相机内参， TUM Freiburg2
    // u    fx,  0, Cx      X/Z
    // v  =  0, fy, Cy  *   Y/Z 归一化平面
    // 1     0,  0,  1      1
    Mat K = ( Mat_<double> ( 3, 3 ) <<
                520.9,     0,  325.1, // fx,  0, Cx
                    0, 521.0,  249.7, //  0, fy, Cy
                    0,     0,      1);//  0,  0,  1

    //-- 把匹配的点转换成vector<Point2f>形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for ( int i = 0; i < matches.size(); i++ )
    {
        points1.push_back( keypoints_1[ matches[i].queryIdx ].pt );
        points2.push_back( keypoints_2[ matches[i].trainIdx ].pt );
    }

    //-- 计算基础矩阵F fundamental_matrix
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat( points1, points2, CV_FM_8POINT );
    cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;

    //-- 计算本质矩阵E essential_matrix
    Point2d principal_point ( 325.1, 249.7 ); // 相机主点， 相机光心，TUM dataset标定值
    int focal_length = 521; // 相机焦距， TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat( points1, points2, focal_length, principal_point );
    cout << "essential_matrix is " << endl << essential_matrix << endl;

    //-- 计算单应矩阵
    Mat homography_matrix;
    homography_matrix = findHomography( points1, points2, RANSAC, 3 );
    cout << "homography_matrix is " << endl << homography_matrix << endl;

    //-- 从本质矩阵中恢复旋转 R 和平移 t 信息
    recoverPose( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cout << "R is " << endl << R << endl;
    cout << "t is " << endl << t << endl;
}

Point2f pixel2cam ( const Point2d& p, const Mat& K )
{
    // 相机内参， TUM Freiburg2
    // u    fx,  0, Cx      X/Z
    // v  =  0, fy, Cy  *   Y/Z 归一化平面
    // 1     0,  0,  1      1
    return Point2f
            (
                    // ( u - Cx ) / fx = X/Z 归一化平面上的 X/Z 坐标
                    ( p.x - K.at<double>( 0, 2 ) ) / K.at<double>( 0, 0 ),
                    // ( v - Cy ) / fy = Y/Z 归一化平面上的 Y/Z 坐标
                    ( p.y - K.at<double>( 1, 2 ) ) / K.at<double>( 1, 1 )
            );
}

void triangulation(
        const std::vector<KeyPoint>& keypoints_1,
        const std::vector<KeyPoint>& keypoints_2,
        const std::vector< DMatch >& matches,
        const Mat& R, const Mat& t,
        std::vector<Point3d>& points )// 返回参数，可以写操作
{
    Mat T1 = (Mat_<float>( 3, 4 ) <<
            1,0,0,0,
            0,1,0,0,
            0,0,1,0);
    Mat T2 = (Mat_<float>( 3, 4 ) <<
            R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
            );
    // 相机内参， TUM Freiburg2
    // u    fx,  0, Cx      X/Z
    // v  =  0, fy, Cy  *   Y/Z 归一化平面
    // 1     0,  0,  1      1
    Mat K = ( Mat_<double> ( 3, 3 ) <<
        520.9,     0,  325.1, // fx,  0, Cx
            0, 521.0,  249.7, //  0, fy, Cy
            0,     0,      1);//  0,  0,  1
    vector<Point2f> pts_1, pts_2;
    for ( DMatch m: matches )
    {
        // 将像素坐标转换成相机归一化坐标X/Z, Y/Z
        pts_1.push_back ( pixel2cam( keypoints_1[m.queryIdx].pt, K) );
        pts_2.push_back ( pixel2cam( keypoints_2[m.trainIdx].pt, K) );
    }

    Mat pts_4d; // 4 x N的矩阵
    // 输入两个相机的位姿T1, T2、特征点在两个相机坐标系下的坐标pts_1, pts_2
    // 输出三角化以后的特征点的3D坐标，
    // 输出的是齐次坐标系，四个维度，需要将前三个维度除以第四个维度得到非齐次次坐标xyz
    // 输出的是在相机坐标系下面的坐标，以输入的两个相机的位姿所在的坐标系为准
    cv::triangulatePoints( T1, T2, pts_1, pts_2, pts_4d );
    // cout << endl << pts_4d << endl;
    // cout << endl << pts_4d.size() << endl;// size()函数输出cols列数rows行数

    // 其次坐标xyzw转换成非齐次坐标xyz，将前三个维度xyz除以第四个维度w得到非齐次坐标xyz
    // 4 x N: 4行N列的矩阵 行1 2 3 / 行4 --> X Y Z
    for ( int i = 0; i<pts_4d.cols; i++ ) // 一列一列进行，除以每一列的最后一个数字
    {
        Mat x = pts_4d.col(i);// 第i列
        x /= x.at<float>( 3, 0 ); //每一行1 2 3 4/最后一行4  归一化
        Point3d p (
                x.at<float>( 0, 0 ), // 每一列第1个数字，即第1行数字
                x.at<float>( 1, 0 ), // 每一列第2个数字，即第2行数字
                x.at<float>( 2, 0 )  // 每一列第3个数字，即第3行数字
                );
        points.push_back( p );
    }
}

int main ( int argc, char** argv )
{
    if ( argc != 3 )
    {
        cout << "usage: triangulation img1 img2" << endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

    // KeyPoint关键点.pt坐标，.x .y坐标
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    // matches.size() 即 vector的大小
    cout << "一个找到了" << matches.size() << "组匹配点" << endl;

    //-- 估计两张图像间的运动
    Mat R,t;
    pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, R, t );

    //-- 三角化
    vector<Point3d> points;
    triangulation( keypoints_1, keypoints_2, matches, R, t, points );

    //-- 验证三角化点于特征点的重投影关系
    // 相机内参， TUM Freiburg2
    // u    fx,  0, Cx      X/Z
    // v  =  0, fy, Cy  *   Y/Z 归一化平面
    // 1     0,  0,  1      1
    Mat K = ( Mat_<double> ( 3, 3 ) <<
        520.9,     0,  325.1, // fx,  0, Cx
            0, 521.0,  249.7, //  0, fy, Cy
            0,     0,      1);//  0,  0,  1
    for ( int i = 0; i < matches.size(); i++ )
    {
        Point2d pt1_cam = pixel2cam( keypoints_1[matches[i].queryIdx ].pt, K );
        // 把3D坐标重投影到两个相机的归一化平面上，从而计算重投影误差  project投影 reproject重投影
        Point2d pt1_cam_3d( // 再次对xyz坐标同时除以z，以得到归一化平面上的坐标
                points[i].x/points[i].z,
                points[i].x/points[i].z
                );

        cout << "point in the first camera frame: " << pt1_cam << endl;
        // d的值是特征点到第一个相机光心O1的距离，但这个距离没有单位，只能表示相对大小。
        cout << "point projected from 3D: " << pt1_cam_3d << ", d = " << points[i].z << endl;

        // 第二个图
        Point2d pt2_cam = pixel2cam( keypoints_2[ matches[i].trainIdx ].pt, K );
        // p2 = R * p1 + t 生成3个元素的列向量
        Mat pt2_trans = R*( Mat_<double>( 3, 1 ) << points[i].x, points[i].y, points[i].z ) + t;
        // X, Y, Z --> X/Z, Y/Z, 1 转换成归一化平面坐标
        pt2_trans /= pt2_trans.at<double>( 2, 0 );
        cout << "point in the second camera frame: " << pt2_cam << endl;
        // .t()列向量转置成行向量
        cout << "point reprojected from second frame: " << pt2_trans.t() << endl;
        cout << endl;
    }
}