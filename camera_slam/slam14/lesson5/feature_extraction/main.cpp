#include <iostream>
#include <opencv2/core/core.hpp>
// OpenCV 3 中通过调用将Rt恢复出来，OpenCV 2 没有，附带的extra文件中有这个功能实现recoverpose
// features2d支持很多特征。ORB是其中一种
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main( int argc, char* argv[] ) // char* argv[]或者char** argv, 设置文件路径
// Clion: Run --> Edit configurations --> Program arguments && Working directory
{
    if( argc != 3 )
    {
        cout << "usage: feature_exectraction img1 img2" << endl;
        return 1;
    }
    // 读取图像
    Mat img_1 = imread( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread( argv[2], CV_LOAD_IMAGE_COLOR );

    // 初始化
    // 用OpenCVC中的KeyPoint来记录特征点
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    // ORB 图像金字塔，缩放不变性，尺度不变性
    // The ORB constructor. 500多少特征，1.2f默认使用金字塔，Pyramid decimation ratio
    // 8 Pyramid level，31阈值，FAST中间像素和周围像素差别多少之后，
    // Ptr<ORB> orb = ORB::create( 500, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20);
    //FeatureDetector有很多的接口，若要使用BRISK也可以，支持的特征，ORB是slam中常用的特征
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // Ptr<FeatureDetector> detector = FeatureDetector::create(detector_name);
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create(descriptor_name);
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );//使用汉明距离
    /* 通过DescriptorMatcher这个类对图像进行匹配，创建的时候可以指定使用的方法，
    FLANNBASED            = 1,
    BRUTEFORCE            = 2,暴力匹配
    BRUTEFORCE_L1         = 3,距离范数，float的描述子就是用L2或是L2的范数，ORB描述子的可以使用HAMMING距离
    BRUTEFORCE_HAMMING    = 4,
    BRUTEFORCE_HAMMINGLUT = 5,
    BRUTEFORCE_SL2        = 6
     */

    // 1、检测Oriented FAST 角点(关键点keypoint)位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    // 2、根据角点位置计算 BRIEF 描述子
    descriptor->compute( img_1, keypoints_1, descriptors_1 );
    descriptor->compute( img_2, keypoints_2, descriptors_2 );

    Mat outimg1;
    drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    imshow("ORB特征点", outimg1);

    // 3、对两幅图像中的 BRIEF 描述子进行匹配，使用 Hamming 距离
    vector<DMatch> matches;
    matcher->match ( descriptors_1, descriptors_2, matches );
    cout << "matches: " << matches.size() << endl;


    // 4、匹配点
    double min_dist = 10000, max_dist = 0;

    // 找出所有匹配之间的最小距离和最大距离，即最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    // 仅供娱乐的写法
    min_dist = min_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    max_dist = max_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    printf( "-- Max dist : %f \n", max_dist );
    printf( "-- Min dist : %f \n", min_dist );

    // 当描述子之间的距离大于两倍的最小距离时，即认为匹配有误，但有时候最小距离会非常小，
    // 设置一个经验值30作为下限，不然 BRUTEFORCE 会给每个descriptor找一个匹配点，
    // 因此设置一个最小距离的上限，然后通过某种经验方式把他去掉，去掉以后输出滤波的匹配
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
        if ( matches[i].distance <= max( 2*min_dist, 30.0 )) // 30.0一定不能漏写.0不然匹配不到相应的函数
            good_matches.push_back( matches[i] );

    // 5、绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    drawMatches( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
    imshow( "所有匹配点", img_match );
    imshow( "优化后匹配点对", img_goodmatch );
    waitKey(0);

    return 0;
}