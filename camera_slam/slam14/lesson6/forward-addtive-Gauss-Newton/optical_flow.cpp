#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace cv;
using namespace Eigen;

#define bInverse false
// this program shows how to use optical flow

string file_1 = "../1.png";  // first image
string file_2 = "../2.png";  // second image

// TODO implement this funciton
/**
 * single level optical flow
 * @param [in] img1 the first image
 * @param [in] img2 the second image
 * @param [in] kp1 keypoints in img1
 * @param [in|out] kp2 keypoints in img2, if empty, use initial guess in kp1
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 */
void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = true
);

// TODO implement this funciton
/**
 * multi level optical flow, scale of pyramid is set to 2 by default
 * the image pyramid will be create inside the function
 * @param [in] img1 the first pyramid
 * @param [in] img2 the second pyramid
 * @param [in] kp1 keypoints in img1
 * @param [out] kp2 keypoints in img2
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 */
void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);
// 在光流中,关键点的坐标值通常是浮点数,但图像数据都是以整数作为下标的。
// 之前我们直接取了浮点数的整数部分,把小数部分归零。
// 但是在光流中,通常的优化值都在几个像素内变化,所以使用浮点数的像素插值。
// 函数 GetPixelValue 提供了一个双线性插值方法(常用的图像插值法之一)获得浮点的像素值。
/**
 * get a gray scale value from reference image (bi-linear interpolated)
 * @param img
 * @param x
 * @param y
 * @return
 */
 // step可以理解为Mat矩阵中每一行的“步长”，以字节为基本单位，
 // 每一行中所有元素的字节总量，是累计了一行中所有元素、所有通道、所有通道的elemSize1之后的值；
inline float GetPixelValue(const cv::Mat &img, float x, float y)
{
    // 1x1像素的小方块
    uchar *data = &img.data[int(y) * img.step + int(x)];// 指针指向方块的左上角(0,0)点
    // 如果选择一个坐标系统使得 f 的四个已知点坐标分别为 (0, 0)、(0, 1)、(1, 0) 和 (1, 1)，双线性插值公式化简为
    // f(x,y)=f(0,0)(1-x)(1-y)+f(1,0)x(1-y)+f(0,1)(1-x)y+f(1,1)xy
    float xx = x - floor(x); // x离0的距离,x的小数部分
    float yy = y - floor(y); // y离0的距离,y的小数部分
    return float(
            (1 - xx) * (1 - yy) * data[0] + // (1-x)*(1-y)*f(0,0)
            xx * (1 - yy) * data[1] +       // x*(1-y)*f(1,0)
            (1 - xx) * yy * data[img.step] +// (1-x)*y*f(0,1)
            xx * yy * data[img.step + 1]    // x*y*f(1,1)
    );
}


int main(int argc, char **argv) {
    cout<<"optical-flow main..."<<endl;
    cout<<"bInverse: "<<bInverse<<endl;
    // images, note they are CV_8UC1, not CV_8UC3
    // 0表示灰度读入
    Mat img1 = imread(file_1, 0);
    Mat img2 = imread(file_2, 0);

    // key points, using GFTT here.
    vector<KeyPoint> kp1;
    Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(img1, kp1); // 找到图1的关键点

    // now lets track these key points in the second image
    // first use single level LK in the validation picture
    vector<KeyPoint> kp2_single;// 返回追踪的关键点在图2中坐标位置
    vector<bool> success_single;// 返回追踪的关键点的成功与否
    //bInverse: true使用反向法inverse, false使用正向法forward
    OpticalFlowSingleLevel(img1, img2, kp1, kp2_single, success_single, bInverse);

    // then test multi-level LK
    vector<KeyPoint> kp2_multi;
    vector<bool> success_multi;
    OpticalFlowMultiLevel(img1, img2, kp1, kp2_multi, success_multi, bInverse);

    // use opencv's flow for validation
    vector<Point2f> pt1, pt2;// 返回图2中追踪的关键点坐标
    for (auto &kp: kp1) pt1.push_back(kp.pt);// 存储图1中关键点的坐标
    vector<uchar> status;// 返回每个关键点追踪的情况，默认为1，没追踪到置0
    vector<float> error;// 返回错误
    // cv:Size(8,8)追踪的关键点所在的图像块大小
    cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error, cv::Size(8, 8));

    // plot the differences of those functions
    Mat img2_single;
    // 将灰度图转换为BGR三通道图像
    cv::cvtColor(img2, img2_single, CV_GRAY2BGR);
    for (int i = 0; i < kp2_single.size(); i++)
    {
        // 遍历跟踪的图2所有的关键点，若跟踪成功，
        if (success_single[i])
        {
            // 在图2上以跟踪成功的点的坐标为圆心，画半径为2，BGR颜色为(0, 250, 0)，线宽为2的圆，
            // circle(InputOutputArray img, Point center, int radius, const Scalar& color, int thickness = 1, int lineType = LINE_8, int shift = 0);
            cv::circle(img2_single, kp2_single[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            // line(InputOutputArray img, Point pt1, Point pt2, const Scalar& color, int thickness = 1, int lineType = LINE_8, int shift = 0);
            // 在图2上，将追踪点原坐标和追踪到的坐标点连线
            cv::line(img2_single, kp1[i].pt, kp2_single[i].pt, cv::Scalar(0, 0, 250));
        }
    }

    Mat img2_multi;
    cv::cvtColor(img2, img2_multi, CV_GRAY2BGR);
    for (int i = 0; i < kp2_multi.size(); i++)
    {
        if (success_multi[i])
        {
            cv::circle(img2_multi, kp2_multi[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi, kp1[i].pt, kp2_multi[i].pt, cv::Scalar(0, 0, 250));
        }
    }

    Mat img2_CV;
    cv::cvtColor(img2, img2_CV, CV_GRAY2BGR);
    for (int i = 0; i < pt2.size(); i++) {
        if (status[i]) {
            cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 0, 250));
        }
    }

    cv::imshow("tracked single level", img2_single);
    cv::imshow("tracked multi level", img2_multi);
    cv::imshow("tracked by opencv", img2_CV);
    cv::waitKey(0);

    return 0;
}

void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse
)
{
    // parameters
    int half_patch_size = 8;
    int iterations = 10;
    bool have_initial = !kp2.empty();

    // 遍历图1中所有的关键点
    for (size_t i = 0; i < kp1.size(); i++)
    {
        auto kp = kp1[i];
        double dx = 0, dy = 0; // dx,dy need to be estimated
        if (have_initial)
        {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }

        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded

        // Gauss-Newton iterations对其中一个关键点所对应的图像块迭代数次求最优值
        for (int iter = 0; iter < iterations; iter++)
        {
            Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
            Eigen::Vector2d b = Eigen::Vector2d::Zero();
            cost = 0;

            // 去除那些提在图像边界附近的点,不然你的图像块可能越过边界
            if (kp.pt.x + dx <= half_patch_size || kp.pt.x + dx >= img1.cols - half_patch_size ||
                kp.pt.y + dy <= half_patch_size || kp.pt.y + dy >= img1.rows - half_patch_size)
            {   // go outside
                succ = false;// 其中一个关键点所对应的图像块超出图像范围，越界，跟踪失败
                break;
            }

            // compute cost and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++)
                {
                    // TODO START YOUR CODE HERE (~8 lines)
                    float u1 = float(kp.pt.x + x);	 float v1 = float(kp.pt.y + y);
                    float u2 = float(u1 + dx); 		 float v2 = float(v1 + dy);
                    double error = 0;
                    Eigen::Vector2d J;  // Jacobian
                    if (inverse == false)
                    {
                        // Forward Jacobian
                        // 图像在该点处x方向梯度I2x = dI2/dx = ( I2(x+1,y)-I2(x-1,y) ) / 2
                        J.x() = double(GetPixelValue(img2, u2 + 1, v2) - GetPixelValue(img2, u2 - 1, v2))/2;
                        // 图像在该点处y方向梯度I2y = dI2/dy = ( I2(x,y+1)-I2(x,y-1) ) / 2
                        J.y() = double(GetPixelValue(img2, u2, v2 + 1) - GetPixelValue(img2, u2, v2 - 1))/2;
                        // 每个像素的误差e = I2(x+dx,y+dy) - I1(x,y)
                        error = double(GetPixelValue(img2, u2, v2) - GetPixelValue(img1, u1, v1));
                    }
                    else
                    {
                        // 迭代开始时, Gauss-Newton 的计算依赖于 I2 在 (xi, yi) 处的梯度信息。
                        // 然而,角点提取算法仅保证了 I1(xi, yi) 处是角点(可以认为角度点存在明显梯度)
                        // 但对于 I2, 我们并没有办法假设 I2 在 xi, yi 处亦有梯度,从而 Gauss-Newton 并不一定成立。
                        // 反向的光流法(inverse)则做了一个巧妙的技巧,即
                        // 用 I1 (xi, yi) 处的梯度,替换掉原本要计算的 I2 (xi+∆xi, yi+∆yi) 的梯度。
                        // 这样做的好处有:
                        // •I1 (xi, yi) 是角点,梯度总有意义;
                        // •I1 (xi, yi) 处的梯度不随迭代改变,所以只需计算一次,就可以在后续的迭代中一直使用,节省了大量计算时间。
                        // Inverse Jacobian
                        // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                        // 图像在该点处x方向梯度I1x = dI1/dx = ( I1(x+1,y)-I1(x-1,y) ) / 2
                        J.x() = double(GetPixelValue(img1, u1 + 1, v1) - GetPixelValue(img1, u1 - 1, v1))/2;
                        // 图像在该点处y方向梯度I1y = dI1/dy = ( I1(x,y+1)-I1(x,y-1) ) / 2
                        J.y() = double(GetPixelValue(img1, u1, v1 + 1) - GetPixelValue(img1, u1, v1 - 1))/2;
                        // 每个像素的误差e = I2(x+dx,y+dy) - I1(x,y)
                        error = double(GetPixelValue(img2, u2, v2) - GetPixelValue(img1, u1, v1));
                    }

                    // compute H, b and set cost;
                    H += J * J.transpose();
                    b += -J * error;
                    cost += error * error;
                    // TODO END YOUR CODE HERE
                }

            // compute update
            // TODO START YOUR CODE HERE (~1 lines)
            Eigen::Vector2d update; 
            update = H.ldlt().solve(b);
            //cout<<"iter: "<<iter<<" update: "<<update.transpose()<<endl;
            // TODO END YOUR CODE HERE

            if (std::isnan(update[0]))
            {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = false;// 对其中一个关键点跟踪失败
                break;
            }
            if (iter > 0 && cost > lastCost)
            {
                //cout << "cost increased: " << cost << ", " << lastCost << endl;
                break;
            }

            // update dx, dy
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;// 对其中一个关键点跟踪成功
        }

        success.push_back(succ); // 返回当前关键点的跟踪状态，true跟踪成功，false跟踪失败

        // set kp2
        if (have_initial)
        {
            // kp2不为空，在当前kp2[i]关键点直接加上增量
            kp2[i].pt = kp.pt + Point2f(dx, dy);
        }
        else
        {
            // auto kp = kp1[i]; kp是kp1的第i个关键点
            KeyPoint tracked = kp;
            // kp2为空，在当前kp1的关键点上加上增量，并添加到kp2的vector中
            tracked.pt += cv::Point2f(dx, dy);
            kp2.push_back(tracked);
        }
    }
}

void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse    )
{
    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.25;
    double scales[] = {1.0, 0.25, 0.0625, 0.015625};// {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<Mat> pyr1, pyr2; // image pyramids
    // TODO START YOUR CODE HERE (~8 lines)
    // 从金字塔底层开始缩放图像
    for (int i = 0; i < pyramids; i++) // 0,1,2,3总共4层金字塔
    {
        Mat img1_temp,  img2_temp;
        // 对图像1进行构建缩放scale[i]倍，
        resize(img1, img1_temp, Size(img1.cols * scales[i], img1.rows * scales[i]));
        // 对图像2进行构建缩放scale[i]倍，
        resize(img2, img2_temp, Size(img2.cols * scales[i], img2.rows * scales[i]));
        pyr1.push_back(img1_temp);// 图像1的金字塔0(原图),1(缩小1/4),2(缩小1/16),3(缩小1/64)
        pyr2.push_back(img2_temp);// 图像2的金字塔0,1,2,3
        cout <<"Pyramid: "<< i <<" img1 size: "<< img1_temp.cols << " " << img1_temp.rows << endl;
    }

    // TODO END YOUR CODE HERE
    // coarse-to-fine LK tracking in pyramids
    // TODO START YOUR CODE HEREi
    vector<KeyPoint> vkp2_now;
    vector<KeyPoint> vkp2_last;
    vector<bool> vsucc;
    // 从金字塔顶层开始coarse-to-fine恢复
    for(int i = pyramids - 1; i >= 0; i--)//遍历金字塔每一层3,2,1,0
    {
        vector<KeyPoint> vkp1;
        // 遍历图1中所有关键点，对其坐标进行缩放
        for(int j = 0; j < kp1.size(); j++)
        {
            KeyPoint kp1_temp = kp1[j];// 遍历kp1的关键点
            //对图1关键点的坐标进行缩放,scales[3](1/64)-->2(1/16)-->1(1/4)-->0(1原图)
            kp1_temp.pt *= scales[i];
            // 缩放后的关键点坐标存储在一个vector中vkp1
           vkp1.push_back(kp1_temp);
           // ???? 有什么用????
          /* if(i < pyramids - 1) // i<3, i=2,1,0金字塔的2,1,0层
            {
                KeyPoint kp2_temp = vkp2_last[j]; // 遍历上一层kp2的关键点
                kp2_temp.pt /= pyramid_scale;// 上一层kp2关键点的坐标/金字塔的尺度0.25,即x4恢复图像大小
                vkp2_now.push_back(kp2_temp);// 存储新一层的kp2关键点的坐标
            }*/
        }
        // 清空金字塔当前层的关键点跟踪成功标识符
        vsucc.clear();// clear()删除vector中所有的元素，vector容器的尺寸变为0
        // 使用单层光流法，传入金字塔当前层的图像pyr1[i]、pyr2[i],以及关键点坐标vkp1,vkp2_now,
        // [out] success true if a keypoint is tracked successfully超出图像范围，越界，跟踪不成功
        // [in] true: inverse(反向法) false: forward(正向法)
        OpticalFlowSingleLevel(pyr1[i], pyr2[i], vkp1, vkp2_now, vsucc, bInverse);
        // 删除所有前一层的kp2的关键点坐标
        vkp2_last.clear();// clear()删除vector中所有元素，vector容器尺寸变为0
        // swap: exchange交换两个相同类型的vector的信息
        // 将当前层的kp2关键点坐标与上一层的关键点坐标（已经clear，为空）交换
        // 就是将当前层的kp2关键点坐标存起来在vkp2_last，成为上一层的kp2关键点信息，并清空当前层变量vkp2_now
        vkp2_last.swap(vkp2_now);
        cout<<"pyramid: "<<i<<" vkp2_last size: "<<vkp2_last.size() <<" vkp2_now size: "<<vkp2_now.size()<<endl;
    }
    // 图2的关键点坐标=存起来的上一层的关键点坐标
    kp2 = vkp2_last;
    // 追踪成功标识符=金字塔追踪的成功标识符
    success = vsucc;
    // TODO END YOUR CODE HERE
    // don't forget to set the results into kp2
}