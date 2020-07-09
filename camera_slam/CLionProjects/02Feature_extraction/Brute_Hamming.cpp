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
    vector<DMatch> matches;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now(); // 计时开始
    BFMatcher bfMatcher(NORM_HAMMING);
    bfMatcher.match(descriptors1, descriptors2, matches);



    // 匹配对筛选
    double min_dist = 1000, max_dist = 0;
    // 找出所有匹配之间的最大值和最小值
    for (int i = 0; i < descriptors1.rows; i++)
    {
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }
    // 当描述子之间的匹配不大于2倍的最小距离时，即认为该匹配是一个错误的匹配。
    // 但有时描述子之间的最小距离非常小，可以设置一个经验值作为下限
    vector<DMatch> good_matches;
    for (int i = 0; i < descriptors1.rows; i++)
    {
        if (matches[i].distance <= max(2 * min_dist, 30.0))
            good_matches.push_back(matches[i]);
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now(); // 计时结束
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout << "cost time: " << time_used.count()*1000 << " ms" << endl;

    printf( "-- Max dist : %f \n", max_dist );
    printf( "-- Min dist : %f \n", min_dist );
    cout << "matches: " << matches.size() << endl;
    cout << "good matches: " << good_matches.size() << endl;

    // 5、绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches( img1, keypoints1, img2, keypoints2, matches, img_match );
    drawMatches( img1, keypoints1, img2, keypoints2, good_matches, img_goodmatch );
    imshow( "all_match", img_match );
    imshow( "brute_good_match", img_goodmatch );

    waitKey(0);

    return 0;
}