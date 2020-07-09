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
    vector<DMatch> goodMatches;

    //-- Step 2: Matching descriptor vectors with a FLANN based matcher



    //cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));直接报错，应该是参数问题
    //FlannBasedMatcher matcher(new flann::LshIndexParams(20,10,2));
    //FlannBasedMatcher matcher(new cv::flann::LshIndexParams(20, 10, 0));
//    descriptors1.convertTo(descriptors1,CV_32F);
//    descriptors2.convertTo(descriptors2,CV_32F);
    // Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    // FlannBasedMatcher matcher(new cv::flann::LshIndexParams(5, 10, 2));
    FlannBasedMatcher matcher(new cv::flann::LshIndexParams(5, 10, 2));
    boost::timer timer;
    std::vector< std::vector<DMatch> > knnMatches;
    matcher.knnMatch( descriptors1, descriptors2, knnMatches, 2 );
    //-- Filter matches using the Lowe's ratio test
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knnMatches.size(); i++) {
        const DMatch& bestMatch = knnMatches[i][0];
        const DMatch& betterMatch = knnMatches[i][1];
        float  distanceRatio = bestMatch.distance / betterMatch.distance;
        if (distanceRatio < 1.f/1.5f)
            goodMatches.push_back(bestMatch);
    }
    cout<<"matches cost time: "<<timer.elapsed()*1000 <<endl;

    cout << "matches: " << goodMatches.size() << endl;

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
    //imshow( "img_goodmatch_RANSAC", img_goodmatch_RANSAC );
    //cv::imwrite( "../result/goodmatch_RANSAC.png", img_goodmatch_RANSAC );


    cout << "RANSAC matches: " << matchesRansac.size() << endl;






//    vector<unsigned char> RansacStatus(keypoints1.size());
//    Mat Fundamental = findFundamentalMat(keypoints1, keypoints2, FM_RANSAC, 3, 0.99, RansacStatus);
//    //Mat fundamental = findFundamentalMat(points1, points2, FM_RANSAC, 3, 0.99);
//    //重新定義關鍵點RR_KP和RR_matches來儲存新的關鍵點和基礎矩陣，通過RansacStatus來刪除誤匹配點
//    vector <KeyPoint> RR_KP1, RR_KP2;
//    vector <DMatch> RR_matches;
//    int index = 0;
//    for (size_t i = 0; i < goodMatches.size(); i++)
//    {
//        if (RansacStatus[i] != 0)
//        {
//            RR_KP1.push_back(keypoints1[i]);
//            RR_KP2.push_back(keypoints2[i]);
//            goodMatches[i].queryIdx = index;
//            goodMatches[i].trainIdx = index;
//            RR_matches.push_back(goodMatches[i]);
//            index++;
//        }
//    }
//    cout << "RANSAC後匹配點數" <<RR_matches.size()<< endl;
//    Mat img_RR_matches;
//    drawMatches(img1, RR_KP1, img2, RR_KP2, RR_matches, img_RR_matches);
//    imshow("After Fundamental RANSAC",img_RR_matches);


    waitKey(0);

    return 0;
}



/*
    const int minNumbermatchesAllowed = 8;
    if (knnMatches.size() < minNumbermatchesAllowed)
        return 1;

    //Prepare data for findHomography从匹配成功的匹配对中获取关键点
    vector<Point2f> srcPoints(goodMatches.size());
    vector<Point2f> dstPoints(goodMatches.size());

    for (size_t i = 0; i < goodMatches.size(); i++) {
        srcPoints[i] = keypoints1[goodMatches[i].trainIdx].pt;
        dstPoints[i] = keypoints2[goodMatches[i].queryIdx].pt;
    }

    // find homography matrix and get inliers mask计算透视变换
    double reprojectionThreshold = 3; // 处理点对为内围层时，允许重投影误差的最大值，一般1-10之间
    vector<uchar> inliersMask(srcPoints.size());
    Mat homography = findHomography(srcPoints, dstPoints, CV_RANSAC, reprojectionThreshold, inliersMask);

    vector<DMatch> inliers;
    for (size_t i = 0; i < inliersMask.size(); i++){
        if (inliersMask[i])
            inliers.push_back(goodMatches[i]);
    }
    goodMatches.swap(inliers);
 */
