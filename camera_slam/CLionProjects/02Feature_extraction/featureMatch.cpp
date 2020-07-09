#include <iostream>
#include<opencv2/opencv.hpp> // 包含所有的opencv2
// #include<opencv2/xfeatures2d.hpp> // 包含contribute中的函数
#include<chrono> // 日期和时间标准库
#include <boost/timer.hpp>

using namespace std;
using namespace cv;


vector<DMatch> ransac(vector<DMatch> matches,vector<KeyPoint> queryKeyPoint,vector<KeyPoint> trainKeyPoint)
{
    //定义保存匹配点对坐标
    vector<Point2f> srcPoints(matches.size()),dstPoints(matches.size());
    //保存从关键点中提取到的匹配点对的坐标
    for(int i=0;i<matches.size();i++)
    {
        srcPoints[i]=queryKeyPoint[matches[i].queryIdx].pt;
        dstPoints[i]=trainKeyPoint[matches[i].trainIdx].pt;
    }
    //保存计算的单应性矩阵
    Mat homography;
    //保存点对是否保留的标志
    vector<unsigned char> inliersMask(srcPoints.size());
    //匹配点对进行RANSAC过滤
    homography = findHomography(srcPoints, dstPoints, CV_RANSAC, 5, inliersMask);
    //RANSAC过滤后的点对匹配信息
    vector<DMatch> matches_ransac;
    //手动的保留RANSAC过滤后的匹配点对
    for(int i=0;i<inliersMask.size();i++)
    {
        if(inliersMask[i])
        {
            matches_ransac.push_back(matches[i]);
            //cout<<"第"<<i<<"对匹配："<<endl;
            //cout<<"queryIdx:"<<matches[i].queryIdx<<"\ttrainIdx:"<<matches[i].trainIdx<<endl;
            //cout<<"imgIdx:"<<matches[i].imgIdx<<"\tdistance:"<<matches[i].distance<<endl;
        }
    }
    //返回RANSAC过滤后的点对匹配信息
    return matches_ransac;
}

int main( int argc, char* argv[] ) // char* argv[]或者char** argv, 设置文件路径
// Clion: Run --> Edit configurations --> Program arguments && Working directory
{
    // 读取图像
    Mat img1 = imread( "../1.png", CV_LOAD_IMAGE_COLOR );
    Mat img2 = imread( "../2.png", CV_LOAD_IMAGE_COLOR );

    // 1. 初始化
    vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    Ptr<ORB> orb = ORB::create(1000);

    // 2. 提取特征点
    orb->detect(img1, keypoints1);
    orb->detect(img2, keypoints2);

    // 3. 计算特征描述符
    orb->compute(img1, keypoints1, descriptors1);
    orb->compute(img2, keypoints2, descriptors2);

    // 4. 对两幅图像的BRIEF描述符进行匹配，使用BFMatch，Hamming距离作为参考
    const float minRatio = 1.f / 1.5f;
    const int k = 2;

    vector<vector<DMatch>> knnMatches;
    vector<DMatch> flann_Matches;
    vector<DMatch> goodMatches;
    Mat descriptors3, descriptors4;

    //matcher.match(descriptors1, descriptors2, matches);
    boost::timer timer;

    FlannBasedMatcher matcher(new cv::flann::LshIndexParams(5, 10, 0));
    matcher.match(descriptors1, descriptors2, flann_Matches);
//
//    for()
//        ;

    BFMatcher matcher_BF(NORM_HAMMING, true);
    matcher_BF.match(descriptors1, descriptors2, goodMatches);


    cout<<"matches cost time: "<<timer.elapsed()*1000 <<endl;

    cout << "good matches: " << goodMatches.size() << endl;

    // 5、绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches( img1, keypoints1, img2, keypoints2, goodMatches, img_match );
    imshow( "match", img_match );
    cv::imwrite( "../result/img_match.png", img_match );


    // vector<DMatch> ransac(vector<DMatch> matches,vector<KeyPoint> queryKeyPoint,vector<KeyPoint> trainKeyPoint)
    vector<DMatch> matchesRansac;
    matchesRansac=ransac(goodMatches, keypoints1, keypoints2);

    Mat img_goodmatch_RANSAC;
    drawMatches( img1, keypoints1, img2, keypoints2, matchesRansac, img_goodmatch_RANSAC );
    imshow( "img_goodmatch_RANSAC", img_goodmatch_RANSAC );

    cout << "RANSAC matches: " << matchesRansac.size() << endl;

    //waitKey(0);

    return 0;
}