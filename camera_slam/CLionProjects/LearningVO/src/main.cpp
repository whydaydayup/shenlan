// 基本过程：
// 1、获取图像
// 2、对图像进行处理
// 3、通过FAST算法对图像进行特征检测，通过KLT光流法跟踪图像的特征，如果跟踪的特征有所丢失，特征数小于某个阈值，那么重新特征检测。
// 4、通过RANSAC的5点算法来估计出两幅图像的本质矩阵。
// 5、通过本质矩阵进行估计R和t
// 6、对尺度信息进行估计，最终确定旋转矩阵和平移向量

#include <fstream>
#include <iostream>
#include <iomanip>
#include "visual_odometry.h"

using namespace std;

int main(int argc, char *argv[])
{
    //定义相机,
    // 动态内存new创建一个PinholeCamera对象，并返回指向相机类对象的指针，cam是个指针
    PinholeCamera *cam = new PinholeCamera(1241.0, 376.0, // width, height
                                           718.8560, 718.8560, 607.1928, 185.2157);// fx, fy, cx, cy
    //初始化对象vo
    VisualOdometry vo(cam);

    // fstream：负责与文件输入输出打交道。包含着ifstream, ofstream, fstream三类。
    // ifstream表示从一个给定文件读取数据。ofstream表示向一个给定文件写入数据。
    std::ofstream out("position.txt"); // 用于保存轨迹数据
    // 创建窗体用于显示读取的图片以及显示轨迹
    char text[100];
    int font_face = cv::FONT_HERSHEY_PLAIN;
    double font_scale = 1;
    int thickness = 1;
    cv::Point text_org(10, 50);
    cv::namedWindow("Road facing camera", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Trajectory", cv::WINDOW_AUTOSIZE);
    cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);// 用于绘制轨迹

    double x=0.0, y=0.0,z=0.0;// 用于保存轨迹
    for (int img_id = 0; img_id < 2000; ++img_id)
    {
        // 导入图像
        std::stringstream ss;
        // setw(int n)用来控制输出间隔,
        // setw()默认填充的内容为空格，可以setfill()配合使用设置其他字符填充。
        // cout<<setfill('*')<<setw(5)<<'a'<<endl;
        ss <<  "/media/riki/TOSHIBA EXT/dataset/sequences/00/image_1/"
           << std::setw(6) << std::setfill('0') << img_id << ".png";


        cv::Mat img(cv::imread(ss.str().c_str(), 0));
        // assert如果它的条件返回错误，则终止程序执行。
        assert(!img.empty()); //如果没有照片就返回错误，终止程序

        // 处理帧
        vo.addImage(img, img_id);
        cv::Mat cur_t = vo.getCurrentT(); // 获得当前的平移分量

        // 画面呈现,把轨迹画出来
        if (cur_t.rows!=0)
        {
            // Mat.at<>()访问元素
            x = cur_t.at<double>(0);
            y = cur_t.at<double>(1);
            z = cur_t.at<double>(2);
        }
        out << x << " " << y << " " << z << std::endl;
        //中心点
        // 先将x转为int类型，之后(300,100)是起始点的坐标。
        // 绘制轨迹，为什么y是第三个z，汽车相机朝向正前方
        int draw_x = int(x) + 300;
        int draw_y = int(z) + 100;
        // 用于绘制轨迹 ，在视频当中看到黑色的画布。
        // 画圆(图像， 圆心坐标， 半径, 红色， 线条粗细)
        cv::circle(traj, cv::Point(draw_x, draw_y), 1, CV_RGB(255, 0, 0), 2);

        //矩形（图像，对角的顶点，黑色，填充），放文字前先用矩形覆盖住。
        cv::rectangle(traj, cv::Point(10, 30), cv::Point(580, 60), CV_RGB(0, 0, 0), CV_FILLED);
        // sprintf输出x,y,z的文本以及在画面上呈现文字putText().	//坐标实时变化
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", x, y, z);
        // 放文字（图像你，文字内容，位置，字体类型，颜色白，厚度）
        cv::putText(traj, text, text_org, font_face, font_scale, cv::Scalar::all(255), thickness, 8);

        cv::imshow("Road facing camera", img);
        cv::imshow("Trajectory", traj);

        cv::waitKey(1);
    }

    delete cam; // 删除new创建的指针
    out.close(); // 关闭文件流ofstream
    // getchar(); // 等待按键输入
    return 0;
}