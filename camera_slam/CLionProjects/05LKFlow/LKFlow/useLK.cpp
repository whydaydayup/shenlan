#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
using namespace std; 

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

int main( int argc, char** argv )
{
//    if ( argc != 2 )
//    {
//        cout<<"usage: useLK path_to_dataset"<<endl;
//        return 1;
//    }
    //string path_to_dataset = argv[1];
    string path_to_dataset = "../data";
    string associate_file = path_to_dataset + "/associate.txt";

    // ifstream文件输入流,类fstream下的
    // 创建一个ifstream,并打开associate_file文件
    ifstream fin( associate_file );
    if ( !fin )  // 若文件不存在,报错
    {
        cerr<<"I cann't find associate.txt!"<<endl;
        return 1;
    }

    // 创建string保存rgb/depth的文件名和时间戳
    string rgb_file, depth_file, time_rgb, time_depth;
    // vector可变大小的数组,支持快速访问,在尾部之外的地方插入或删除元素可能很慢
    // list双向链表,支持双向访问,任何地方进行添加/删除都很快
    list< cv::Point2f > keypoints;      // 因为要删除跟踪失败的点，使用list
    cv::Mat color, depth, last_color;
    
    for ( int index=0; index<100; index++ )
    {
        // 1305031453.359684 rgb/1305031453.359684.png 1305031453.374112 depth/1305031453.374112.png
        // associate.txt文件中保存的数据的格式,分别保存到相应的string中去
        // 依次读取文件中的每一行数据
        fin>>time_rgb>>rgb_file>>time_depth>>depth_file;
        //cout << time_rgb << rgb_file << time_depth << depth_file << endl;
        color = cv::imread( path_to_dataset+"/"+rgb_file ); // ../data / rgb/xxxxx.png
        // IMREAD_UNCHANGED = -1,读取的深度图是16bit 无符号的
        depth = cv::imread( path_to_dataset+"/"+depth_file, -1 ); // ../data / depth/xxxx.png
        if (index ==0 ) // 读取的是第一个文件
        {
            // 对第一帧提取FAST特征点
            vector<cv::KeyPoint> kps;// vector保存提取的特征点
            // CV_WRAP static Ptr<FastFeatureDetector> create( int threshold=10, bool nonmaxSuppression=true, int type=FastFeatureDetector::TYPE_9_16 );
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            // 提取FAST特征点,并保存到kps中
            detector->detect( color, kps );

            // The first thing we need to do is get the features
            // we want to track.
            //
            vector< cv::Point2f > cornersA, cornersB;
            const int MAX_CORNERS = 500;
            cv::goodFeaturesToTrack(
                    color,                         // Image to track
                    cornersA,                     // Vector of detected corners (output)
                    MAX_CORNERS,                  // Keep up to this many corners
                    0.01,                         // Quality level (percent of maximum)
                    5,                            // Min distance between corners
                    cv::noArray(),                // Mask
                    3,                            // Block size
                    false,                        // true: Harris, false: Shi-Tomasi
                    0.04                          // method specific parameter
            );

            cv::cornerSubPix(
                    imgA,                           // Input image
                    cornersA,                       // Vector of corners (input and output)
                    cv::Size(win_size, win_size),   // Half side length of search window
                    cv::Size(-1, -1),               // Half side length of dead zone (-1=none)
                    cv::TermCriteria(
                            cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                            20,                         // Maximum number of iterations
                            0.03                        // Minimum change per iteration
                    )
            );

            for ( auto kp:kps ) // 遍历当前帧所有的特征点
                keypoints.push_back( kp.pt );// 保存当前帧每个特征点的Point2f pt即x,y坐标
            last_color = color;
            continue; // 执行下一for循环
        }
        if ( color.data==nullptr || depth.data==nullptr ) // 若rgb和depth文件为空,指向nullptr空空指针
            continue; // 执行下一个for循环

        // 对其他帧(除了第一帧)用LK跟踪特征点,而不需要提取特征点
        vector<cv::Point2f> next_keypoints; // 下一帧的关键点
        vector<cv::Point2f> prev_keypoints; // 上一帧的关键点
        for ( auto kp:keypoints ) // 遍历上一帧提取的所有的关键点
            prev_keypoints.push_back(kp); // prev_keypoints保存到上一帧的关键点的vector中
        vector<unsigned char> status;// 保存光流跟踪的成功与否
        vector<float> error;// 保存误差?

        chrono::steady_clock::time_point t1 = chrono::steady_clock::now(); // 计时开始
        // 使用金字塔模型Pyr计算光流, (上一帧图片,当前帧图片,上一帧特征点,当前帧特征点,status返回成功与否的标志,error误差)
        cv::calcOpticalFlowPyrLK( last_color, color, prev_keypoints, next_keypoints, status, error );
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now(); // 计时结束
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
        cout<<"LK Flow use time："<<time_used.count()<<" seconds."<<endl;

        // 把跟丢的点删掉
        int i=0;
        // iter是迭代器,使用迭代器遍历第一帧所有的特征点,每一帧都会有跟丢的光流,每一帧都会删除跟丢的光流点
        for ( auto iter=keypoints.begin(); iter!=keypoints.end(); i++)
        {
            // 若特征点跟丢,删除
            if ( status[i] == 0 ) // 根据返回的status来删除跟丢的特征点
            {
                // erase删除迭代器iter指定的元素,返回一个指向被删元素之后元素的迭代器
                iter = keypoints.erase(iter);
                continue;// 执行for循环,直到删除所有的跟丢的特征点
            }
            // 特征点没跟丢,迭代器指向当前读取的文件的特征点,  keypoints保存当前帧所有的特征点信息
            *iter = next_keypoints[i]; // 迭代器指向下一特征点中的第i个元素
            iter++;// 迭代器++
        }
        cout<<"tracked keypoints: "<<keypoints.size()<<endl;
        if (keypoints.size() == 0)
        {
            cout<<"all keypoints are lost."<<endl;
            break; 
        }
        // 画出 keypoints
        cv::Mat img_show = color.clone(); // clone复制当前图像
        for ( auto kp:keypoints )// 遍历所有的当前的特征点
            // CV_EXPORTS_W void circle(InputOutputArray img, Point center, int radius, const Scalar& color, int thickness = 1, int lineType = LINE_8, int shift = 0);
            // (图像, 圆心, 半径, 颜色, 线宽)
            cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);
        cv::imshow("corners", img_show);
        cv::waitKey(0);
        last_color = color;
    }
    return 0;
}