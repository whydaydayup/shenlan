#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// #include "extra.h" // use this if in OpenCV2

using namespace std;
using namespace cv;

void find_feature_matches ( const Mat& img1, const Mat& img2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    // -初始化   特征点包括 关键点KeyPoints和描述子descriptors
    Mat descriptors_1, descriptors_2; // 特征点的描述子descriptor
    // Mat M; OpenCV 默认生成的矩阵M为0行0列
    // OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create( "ORB" );
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create ( "BruteForce-Hamming" );

    //-- 1、检测 Oriented FAST 角点位置
    detector->detect( img1, keypoints_1 );
    detector->detect( img2, keypoints_2 );

    //--2、根据角点位置计算 BRIEF 描述子
    descriptor->compute( img1, keypoints_1, descriptors_1 );
    descriptor->compute( img2, keypoints_2, descriptors_2 );

    //--3、对两幅图像中的 BRIEF 描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match; // 初步匹配match
    matcher->match( descriptors_1, descriptors_2, match );
    cout << match.size();

    //--4、匹配点筛选
    double min_dist=10000, max_dist=0;
    // 找出所有匹配点之间的最小距离和最大距离，即最相似和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )  // 多少个描述子descriptors_1.rows
    {
        // vector<DMatch> match;  match[i]第i个匹配点
        double dist = match[i].distance;
        if( match[i].distance < min_dist ) min_dist = match[i].distance;
        if( match[i].distance > max_dist ) max_dist = match[i].distance;
    }
    printf( "-- Max dist : %f \n", max_dist );
    printf( "-- Min dist : %f \n", min_dist );

    // 当描述子之间的距离大于两倍的最小距离时，即认为匹配有误，
    // 但有时候最小距离会非常小，设置一个经验值 30 为下限
    for ( int i = 0; i < match.size(); i++ )
    {
        if ( match[i].distance <= max( 2*min_dist, 30.0) ) // max取最大值，30必须写成30.0
            matches.push_back( match[i] );
    }
}

void pose_estimation_2d_2d ( std::vector<KeyPoint> keypoints_1,
                             std::vector<KeyPoint> keypoints_2,
                             std::vector<DMatch> matches,
                             Mat& R, Mat& t)
{
    // u = fx*(X/Z)+Cx; v = fy*(Y/Z)+Cy   fx,fy单位像素
    // 相机内参, TUM Freiburg2
    Mat K = ( Mat_<double> (3, 3)
                << 520.9,     0, 325.1,  // fx,  0,  Cx,
                       0, 521.0, 249.7,  //  0, fy,  Cy,
                       0,     0,     1 );//  0,  0,   1,

    //-- 把匹配的点转换成 vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    // matches.size()输出匹配点的对数, 即vector的大小
    for ( int i = 0; i < (int) matches.size(); i++ )
    {
        // 存储匹配成功 matches 的 描述子desciptors 对应的 关键点 keypoints 的坐标 pt
        // OpenCV 的 DMatch 结构,queryIdx 为第一图的特征 ID,trainIdx 为第二个图的特征 ID。
        // KeyPoints.pt关键点的坐标
        points1.push_back( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back( keypoints_2[matches[i].trainIdx].pt );
    }

    //-- 计算基础矩阵F fundatmental_matrix
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat( points1, points2, CV_FM_8POINT ); // 8点法
    cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;

    //-- 计算本质矩阵E essential_matrix
    Point2d principal_point ( 325.1, 249.7 ); // 相机光心，相机主点， TUM dataset 标定值
    double focal_length = 521; // 相机焦距，TUM dataset 标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
    cout << "essential_matrix is " << endl << essential_matrix << endl;

    //-- 计算单应矩阵H homography_matrix，
    // 由于并不是很平面化，所以得出的单应矩阵没有太大意义
    Mat homography_matrix;
    homography_matrix = findHomography( points1, points2, RANSAC, 3);
    cout << "homography_matrix is " << endl << homography_matrix << endl;

    //-- 从本质矩阵E essential_matrix中恢复得到旋转R和平移t的信息
    recoverPose( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cout << "R is " << endl << R << endl;
    cout << "t is " << endl << t << endl;
}
// x是朝右的，ｙ是朝下的，ｚ是朝前的
//　得出的ｔ^R应该是跟Ｅ差一个因子，ｋ*E=t^R

Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
            (
                    // K = fx,  0,  Cx,
                    //      0, fy,  Cy,
                    //      0,  0,   1,
                    // X/Z = ( u -Cx ) / fx 归一化后的X坐标
                    ( p.x - K.at<double> ( 0, 2 ) ) / K.at<double> ( 0, 0 ),
                    // Y/Z = ( v - Cy ) / fy 归一化后的Y坐标
                    ( p.y - K.at<double> ( 1, 2 ) ) / K.at<double> ( 1, 1 )
            );
}

int main( int argc, char** argv )
{
    if ( argc != 3 )
    {
        cout << "usage: pose_estimation_2d_2d img1 img2" << endl;
        return 1;
    }
    Mat img1 = imread( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img2 = imread( argv[2], CV_LOAD_IMAGE_COLOR );

    // 特征点包括1、关键点KeyPoints 2、描述子descriptor
    // 特征点的关键点keypoints
    /*
    float x, float y;// 关键点的x,y坐标
    Point2f pt; //<关键点坐标coordinates of the keypoints>
    float size; //<关键点邻域直径大小diameter of the meaningful keypoint neighborhood
    float angle; //<特征点方向computed orientation of the keypoint (-1 if not applicable);
                //< it's in [0,360) degrees and measured relative to
                //< image coordinate system, ie in clockwise. float response;
                //< the response by which the most strong keypoints have been selected. Can be used for the further sorting or subsampling
    int octave; //<关键点所在的图像金字塔的组octave (pyramid layer) from which the keypoint has been extracted
    int class_id; //<用于聚类的ID object class (if the keypoints need to be clustered by an object they belong to) };
     */
    vector<KeyPoint> keypoints_1, keypoints_2;
    // OpenCV 的 DMatch 结构,
    // queryIdx 为第一图的特征 ID,trainIdx 为第二个图的特征 ID，
    // distance 为两个匹配点之间的距离
    vector<DMatch> matches;
    find_feature_matches ( img1, img2, keypoints_1, keypoints_2, matches );
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;

    // 估计两张图像之间的运动
    Mat R, t;
    pose_estimation_2d_2d ( keypoints_1, keypoints_2, matches, R, t );

    //-- 验证E=t^R*scale
    // 向量t 转换成 反对称矩阵t^
    // t(a1, a2, a3)' --> (  0, -a3, a2,
    //                      a3,  0, -a1,
    //                     -a2, a1,   0)
    Mat t_x = ( Mat_<double> ( 3, 3 ) <<
                0,                        -t.at<double> ( 2, 0 ),    t.at<double> ( 1, 0 ),
                t.at<double> ( 2, 0 ),      0,                      -t.at<double> ( 0, 0 ),
                -t.at<double> ( 1, 0 ),     t.at<double> ( 0, 0 ),            0);
    cout << "t^R = " << endl << t_x*R << endl;

    //-- 验证对极约束 epipolar constraint
    // u = fx*(X/Z)+Cx; v = fy*(Y/Z)+Cy   fx,fy单位像素
    // 相机内参, TUM Freiburg2
    Mat K = ( Mat_<double> (3, 3)
            << 520.9,     0, 325.1,  // fx,  0,  Cx,
                   0, 521.0, 249.7,  //  0, fy,  Cy,
                   0,     0,     1 );//  0,  0,   1,
    for ( DMatch& m: matches ) // 遍历所有的匹配对
    {
        // X/Z = ( u -Cx ) / fx 归一化后的X坐标
        // Y/Z = ( v - Cy ) / fy 归一化后的Y坐标
        // pt1, pt2归一化平面的X, Y坐标
        Point2d pt1 = pixel2cam ( keypoints_1[ m.queryIdx ].pt, K );
        // y1, y2归一化平面的坐标( X/Z, Y/Z, 1) x1, x2
        Mat y1 = ( Mat_<double> ( 3, 1 ) << pt1.x, pt1.y, 1 );
        Point2d pt2 = pixel2cam ( keypoints_2[ m.trainIdx ].pt, K );
        Mat y2 = ( Mat_<double> ( 3, 1 ) << pt2.x, pt2.y, 1 );
        // 对极约束 x2t * t^ * R * x1 = 0, x2t矩阵x2的转置，t^向量t的反对称矩阵
        // --> y2t * t_x * R * y1 = 0, y2t矩阵y2的转置，t_x向量t的反对称矩阵
        // y2.t()矩阵求转置   .inv()矩阵求逆
        Mat d = y2.t() * t_x * R * y1;
        cout << "epipolar constraint = " << d << endl;
    }
    return 0;
}