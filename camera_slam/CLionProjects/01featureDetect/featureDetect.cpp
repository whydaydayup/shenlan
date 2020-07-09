#include<opencv2/opencv.hpp> // 包含所有的opencv2
#include<opencv2/xfeatures2d.hpp> // 包含contribute中的函数
#include<chrono> // 日期和时间标准库

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

int main() {
    // imread读入图片保存在src矩阵中
    cv::Mat src = cv::imread("../tum.png");
    // cv::Mat src = cv::imread("../residential.png"); // campus.png  tum.png
    if (src.empty())
    {
        printf("can not load image \n");
        return -1;
    }
    //imshow("original", src);

    Mat dstSIFT; // SIFT特征检测
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now(); // 计时开始
    Ptr<SIFT> sift_detector = SIFT::create(1000);
    // KeyPoint类是为特征点检测而生的数据结构,用于表示特征点
    std::vector<cv::KeyPoint> keypointsSIFT; // 创建保存关键点的vector
    sift_detector->detect(src, keypointsSIFT); // 提取SIFT特征点
        // (Mat &srcImage, vector<KeyPoint>& keypoints, Mat& outImage, 关键点的颜色的默认值, 关键点的特征标识符)
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now(); // 计时结束
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout << "SIFT cost time: " << time_used.count()*1000 << " ms" << endl;
    cout << "SIFT keypoints numbers: " << keypointsSIFT.size() << endl;
    drawKeypoints(src, keypointsSIFT, dstSIFT, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
        // drawKeypoints(src, keypointsSIFT, dstSIFT, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imwrite( "../result/sift.png", dstSIFT );
    //imshow("SIFT_output", dstSIFT);


    Mat dstSURF; // SURF特征检测
    t1 = chrono::steady_clock::now(); // 计时开始
    Ptr<SURF> surf_detector = SURF::create(100);
    std::vector<cv::KeyPoint> keypointsSURF; // 创建保存关键点的vector
    surf_detector->detect(src, keypointsSURF); // 提取SURF特征点
        // (Mat &srcImage, vector<KeyPoint>& keypoints, Mat& outImage, 关键点的颜色的默认值, 关键点的特征标识符)
    t2 = chrono::steady_clock::now(); // 计时结束
    time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout << "SURF cost time: " << time_used.count()*1000 << " ms" << endl;
    cout << "SURF keypoints numbers: " << keypointsSURF.size() << endl;
    drawKeypoints(src, keypointsSURF, dstSURF, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
        // drawKeypoints(src, keypointsSURF, dstSURF, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imwrite( "../result/surf.png", dstSURF );
    //imshow("SURF_output", dstSURF);


    Mat dstORB; // ORB特征检测
    t1 = chrono::steady_clock::now(); // 计时开始
    Ptr<ORB> orb_detector = ORB::create(1000);
    std::vector<cv::KeyPoint> keypointsORB; // 创建保存关键点的vector
    orb_detector->detect(src, keypointsORB); // 提取ORB特征点
    t2 = chrono::steady_clock::now(); // 计时结束
    time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout << "ORB cost time: " << time_used.count()*1000 << " ms" << endl;
        // (Mat &srcImage, vector<KeyPoint>& keypoints, Mat& outImage, 关键点的颜色的默认值, 关键点的特征标识符)
    cout << "ORB keypoints numbers: " << keypointsORB.size() << endl;
    drawKeypoints(src, keypointsORB, dstORB, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
        // drawKeypoints(src, keypointsORB, dstORB, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imwrite( "../result/orb.png", dstORB );
    //imshow("ORB_output", dstORB);

//
//    Mat dstBRISK; // BRISK特征检测
//    t1 = chrono::steady_clock::now(); // 计时开始
//    Ptr<BRISK> brisk_detector = BRISK::create(43);
//    // KeyPoint类是为特征点检测而生的数据结构,用于表示特征点
//    std::vector<cv::KeyPoint> keypointsBRISK; // 创建保存关键点的vector
//    brisk_detector->detect(src, keypointsBRISK); // 提取BRISK特征点
//    // (Mat &srcImage, vector<KeyPoint>& keypoints, Mat& outImage, 关键点的颜色的默认值, 关键点的特征标识符)
//    t2 = chrono::steady_clock::now(); // 计时结束
//    time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
//    cout << "BRISK cost time: " << time_used.count()*1000 << " ms" << endl;
//    cout << "BRISK keypoints numbers: " << keypointsBRISK.size() << endl;
//    drawKeypoints(src, keypointsBRISK, dstBRISK, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
//    // drawKeypoints(src, keypointsBRISK, dstBRISK, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//    //imshow("BRISK_output", dstBRISK);

    //waitKey(0); // 暂停程序,等待一个按键输入
    return 0;
}