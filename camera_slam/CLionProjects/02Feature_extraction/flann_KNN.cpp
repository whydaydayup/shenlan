#include <iostream>
#include<opencv2/opencv.hpp> // 包含所有的opencv2
#include<opencv2/xfeatures2d.hpp> // 包含contribute中的函数
#include<chrono> // 日期和时间标准库

using namespace std;
using namespace cv;

int main( int argc, char* argv[] ) // char* argv[]或者char** argv, 设置文件路径
// Clion: Run --> Edit configurations --> Program arguments && Working directory
{
    // 读取图像
    Mat img1 = imread( "../1.png", CV_LOAD_IMAGE_COLOR );
    Mat img2 = imread( "../2.png", CV_LOAD_IMAGE_COLOR );

    // 1. 初始化
    vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    Ptr<ORB> orb = ORB::create(500);

    // 2. 提取特征点
    orb->detect(img1, keypoints1);
    orb->detect(img2, keypoints2);

    // 3. 计算特征描述符
    orb->compute(img1, keypoints1, descriptors1);
    orb->compute(img2, keypoints2, descriptors2);

    // 4. 对两幅图像的BRIEF描述符进行匹配，使用BFMatch，Hamming距离作为参考
    vector<DMatch> goodMatches;

    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    descriptors1.convertTo(descriptors1,CV_32F);
    descriptors2.convertTo(descriptors2,CV_32F);

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knnMatches;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now(); // 计时开始
    matcher->knnMatch( descriptors1, descriptors2, knnMatches, 2 );
    //-- Filter matches using the Lowe's ratio test

    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knnMatches.size(); i++) {
        const DMatch& bestMatch = knnMatches[i][0];
        const DMatch& betterMatch = knnMatches[i][1];
        float  distanceRatio = bestMatch.distance / betterMatch.distance;
        if (distanceRatio < 1.f/1.5f)
            goodMatches.push_back(bestMatch);
    }


/*
    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch( descriptors1, descriptors2, knn_matches, 2 );
    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f;
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
*/

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now(); // 计时结束
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout << "cost time: " << time_used.count()*1000 << " ms" << endl;

    cout << "matches: " << goodMatches.size() << endl;

    // 5、绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches( img1, keypoints1, img2, keypoints2, goodMatches, img_match );
    imshow( "match", img_match );

    waitKey(0);

    return 0;
}
