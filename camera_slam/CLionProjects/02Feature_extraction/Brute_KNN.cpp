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
    const float minRatio = 1.f / 1.5f;
    const int k = 2;

    vector<vector<DMatch>> knnMatches;
    vector<DMatch> goodMatches;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now(); // 计时开始
    BFMatcher matcher(NORM_HAMMING);
    //matcher.match(descriptors1, descriptors2, matches);

    matcher.knnMatch(descriptors1, descriptors2, knnMatches, k);

    for (size_t i = 0; i < knnMatches.size(); i++) {
        const DMatch& bestMatch = knnMatches[i][0];
        const DMatch& betterMatch = knnMatches[i][1];
        float  distanceRatio = bestMatch.distance / betterMatch.distance;
        if (distanceRatio < minRatio)
            goodMatches.push_back(bestMatch);
    }

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