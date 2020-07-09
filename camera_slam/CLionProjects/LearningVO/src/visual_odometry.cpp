// 根据结果我们就会发现算法后期的误差是越来越大，而这个累计误差在目前是无法避免的，
// 因为后续相机的位姿都依赖于前面相机的位姿，因此要保证更好的精度不仅仅考虑两帧的信息，
// 而是应该考虑全部信息，而对于帧率本机测试大概在10帧每秒，还没有达到实时，
// 如果考虑更多的帧的信息，效率肯定会进一步下降，

#include "visual_odometry.h"

#include <string>
#include <fstream>

// 如果跟踪特征点数小于给定阈值，进行重新特征检测
const int kMinNumFeature = 2000;

VisualOdometry::VisualOdometry(PinholeCamera* cam):
        cam_(cam)
{
    focal_ = cam_->fx(); // 焦距
    pp_ = cv::Point2d(cam_->cx(), cam_->cy()); // 相机中心位移
    frame_stage_ = STAGE_FIRST_FRAME; // 当前帧状态, 初始化为第一帧状态
}

VisualOdometry::~VisualOdometry()
{

}

/// 提供一个图像
void VisualOdometry::addImage(const cv::Mat& img, int frame_id)
{
    //对添加的图像进行判断
    if (img.empty() || img.type() != CV_8UC1 || img.cols != cam_->width() || img.rows != cam_->height())
        throw std::runtime_error("Frame: provided image has not the same size as the camera model or image is not grayscale");
    // 添加新帧
    new_frame_ = img;
    bool res = true;//结果状态
    if (frame_stage_ == STAGE_DEFAULT_FRAME)
        res = processFrame(frame_id);
    else if (frame_stage_ == STAGE_SECOND_FRAME)
        res = processSecondFrame();
    else if (frame_stage_ == STAGE_FIRST_FRAME)
        res = processFirstFrame();
    // 处理结束之后将当前帧设为上一处理帧
    last_frame_ = new_frame_;

}
// 根据当前帧的状态分别放进三个函数里，第一帧、第二帧、正常帧

// 第一帧
bool VisualOdometry::processFirstFrame()
{
    // 对第一帧进行关键点检测
    featureDetection(new_frame_, px_ref_);//第一帧计算出来的特征，为参考特征点
    // 修改状态，表明第一帧已经处理完成
    frame_stage_ = STAGE_SECOND_FRAME;
    return true;
}
// 第二帧       2d_2d使用对极约束求解相机运动
// 第二帧算出第一次R，t，特征跟踪、基础矩阵(本质矩阵)、恢复位姿
bool VisualOdometry::processSecondFrame()
{
    // 对第一帧检测出来的FAST特征进行LK光流法跟踪
    featureTracking(last_frame_, new_frame_, px_ref_, px_cur_, disparities_); //通过光流跟踪确定第二帧中的相关特征
    // 计算初始位置
    cv::Mat E, R, t, mask;
    // 本质矩阵是归一化图像坐标下的基础矩阵的特殊形式，归一化图像坐标也就是用当前像素坐标左乘相机矩阵的逆。
    // 采用随机抽样一致性算法（RANSAC），由于所有的点不是完全的匹配，
    // 如果用最小二乘，难免因为很多误匹配的点导致最后的结果有很大的偏差，
    // RANSAC的做法，随机抽取5个点(5点法)用于估计两帧之间的本质矩阵，
    // 检测其它点是否满足该本质矩阵，得到满足该本质矩阵的个数（称为内点），
    // 然后进行多次迭代，得到内点最多的那组计算出的本质矩阵即为所求解。
    E = cv::findEssentialMat(px_cur_, px_ref_, // 当前帧的关键点的Point2f的vector, 参考帧的关键点,
                             focal_, pp_, // 焦距, 光心,
                             cv::RANSAC, // 计算方法(RANSAC算法(默认)/LMedS算法
                             0.999, // double prob = 0.999,规定了估计矩阵正确的可信度（概率）,用于RANSAC或LMedS方法
                             1.0, // double threshold = 1.0用于RANSAC的参数。
                             // 它是从点到极线的最大距离（以像素为单位），超出此点时，该点被视为异常值，不用于计算最终的基本矩阵。
                             // 根据点定位精度，图像分辨率和图像噪声的不同，可将其设置为1-3。
                             mask); // OutputArray mask = noArray()输出N个元素的数组，
                             // 其中每个元素对于异常值设置为0，对其他点设置为1。 用于RANSAC和LMedS方法。
    // 通过本质矩阵EssentialMat恢复旋转和平移信息
    // 当假设第一幅图像的摄像机矩阵P=[I|0]的时候，
    // 对于本质矩阵通过SVD分解，可以计算出第二幅图像的摄像机矩阵。
    cv::recoverPose(E, px_cur_, px_ref_, R, t, focal_, pp_,
                    mask); // Input/output mask for inliers in points1 and points2.
    cur_R_ = R.clone(); // 当前帧的旋转分量
    cur_t_ = t.clone(); // 当前帧的平移分量
    frame_stage_ = STAGE_DEFAULT_FRAME;// 设置状态处理默认帧
    //当前特征点变为参考帧特征点
    px_ref_ = px_cur_;
    return true;
}
// 正常帧
// 之后的帧R、t要累加
bool VisualOdometry::processFrame(int frame_id)
{
    double scale = 1.00; //初始尺度为1
    // 在new_frame_上跟踪last_frame_的FAST特征,
    // 返回的是px_curr_当前帧的特征点2D坐标,以及disparities_对应特征之前的像素距离
    featureTracking(last_frame_, new_frame_, px_ref_, px_cur_, disparities_); //通过光流跟踪确定第二帧中的相关特征
    cv::Mat E, R, t, mask;
    // 计算本质矩阵
    E = cv::findEssentialMat(px_cur_, px_ref_, focal_, pp_, cv::RANSAC, 0.999, 1.0, mask);
    // 从本质矩阵恢复旋转和平移信息
    cv::recoverPose(E, px_cur_, px_ref_, R, t, focal_, pp_, mask);
    scale = getAbsoluteScale(frame_id);//得到当前帧的实际尺度
    if (scale > 0.1) // 如果尺度小于0.1可能计算出的Rt存在一定的问题,则不做处理，保留上一帧的值
    {
        // 通过本质矩阵可以得到两帧计算的相对位置关系，
        // 因为第二幅图像对应相机矩阵是在假设第一幅图像的相机矩阵为P=[I|0]的情况，
        // 因此我们对每次计算的相对量进行累加，
        cur_t_ = cur_t_ + scale*(cur_R_*t);
        cur_R_ = R*cur_R_;
    }
    // 如果跟踪特征点数小于给定阈值，进行重新特征检测
    if (px_ref_.size() < kMinNumFeature)
    {
        featureDetection(new_frame_, px_ref_);
        featureTracking(last_frame_, new_frame_, px_ref_, px_cur_, disparities_);
    }
    // 当前特征点变为参考帧特征点
    px_ref_ = px_cur_;
    return true;
}
// 注意：由于单目相机定位存在scale的问题，
// 这边先采用真实数据计算两帧之间的距离作为scale，后续会对scale的问题进行进一步分析。
// 获取绝对尺度?????,通过groundtruth来获得绝对尺度.......
double VisualOdometry::getAbsoluteScale(int frame_id)
{
    std::string line;
    int i = 0;
    // KITTI数据集
    std::ifstream ground_truth("/media/riki/TOSHIBA EXT/dataset/poses/00.txt");
    double x = 0, y = 0, z = 0;
    double x_prev, y_prev, z_prev;
    // 获取当前帧真实位置与前一帧的真实位置的距离作为尺度值
    if (ground_truth.is_open())
    {
        while ((std::getline(ground_truth, line)) && (i <= frame_id))
        {
            z_prev = z;
            x_prev = x;
            y_prev = y;
            std::istringstream in(line);
            for (int j = 0; j < 12; j++)  {
                in >> z;
                if (j == 7) y = z;
                if (j == 3)  x = z;
            }
            i++;
        }
        ground_truth.close();
    }

    else {
        std::cerr<< "Unable to open file";
        return 0;
    }

    return sqrt((x - x_prev)*(x - x_prev) + (y - y_prev)*(y - y_prev) + (z - z_prev)*(z - z_prev));
}
// FAST算法比其他已知的角点检测法要快很多倍,但是当图片的噪点较多时，它的健壮性并不好。这依靠一个阙值。
// FAST关键点检测角点Corners, 不计算描述子
// 得到关键点的x,y坐标的集合
void VisualOdometry::featureDetection(cv::Mat image, std::vector<cv::Point2f>& px_vec)
{
    std::vector<cv::KeyPoint> keypoints;
    int fast_threshold = 20; // FAST检测的阈值???什么阈值
    // 非极大抑制, 在一定区域内仅保留响应极大值的角点,避免角点集中的问题
    bool non_max_suppression = true; // 是否启用非极大值抑制,默认true
    cv::FAST(image, keypoints, fast_threshold, non_max_suppression); // image应该是grayscale灰度图
    // 将vector<KeyPoint>转换成vector<Point2f>
    cv::KeyPoint::convert(keypoints, px_vec);
// 下面是自己写的转换函数的例子https://blog.csdn.net/liaojiacai/article/details/53705509
//    void KeyPointsToPoints(vector<KeyPoint>& kpts, vector<Point2f> &pts)
//    {
//        for (int i = 0; i < kpts.size(); i++) {
//            pts.push_back(kpts[i].pt);
//        }
//    }
//    void PointsToKeyPoints(vector<Point2f>pts,vector<KeyPoint>kpts)
//    {
//        for (size_t i = 0; i < pts.size(); i++) {
//            kpts.push_back(KeyPoint(pts[i], 1.f));
//        }
//    }
}
// 特征提取是一个比较耗时的操作，即使使用了FAST特征检测，
// 而且都是使用特征提取之后要进行特征匹配，通过RANSAC等方法进行特征提纯等操作，
// 然后由于两帧之间时间间隔较短，完全可以采用跟踪的方法来预测下一帧的特征点。
// 这里直接采用OpenCV中calcOpticalFlowPyrLK方法即金字塔的KLT跟踪算法，
// 采用金字塔的KLT算法是为了解决运动范围过大及不连贯的情况，
// 通过在图像金字塔的最高层计算光流，用得到的运动估计结果作为下一次金字塔的起始点，
// 重复这个过程直到金字塔的最底层。

// 具体将图像It中检测到的特征Ft跟踪到图像It+1中，得到对应的特征Ft+1

// 这边记录了跟踪之后对应特征之间的像素距离存入std::vector<double>& disparities中，
// 后续会对这个像素距离设定阈值，以确保初始位置确定的准确度。
// 在对特征进行跟踪的时候，跟踪到的特征会越来越少（由于有相同视野的部分越来越小），
// 因此设定一个阈值以保证特征的个数，小于给定阈值的时候，我们重新进行特征检测。
void VisualOdometry::featureTracking(cv::Mat image_ref, cv::Mat image_cur,
                                     std::vector<cv::Point2f>& px_ref, std::vector<cv::Point2f>& px_cur, std::vector<double>& disparities)
{
    const double klt_win_size = 21.0; // 每个金字塔层搜索窗大小, 默认21
    const int klt_max_iter = 30;  // 最大迭代次数
    const double klt_eps = 0.001; // 迭代收敛的最小误差(期望的精度)
    std::vector<uchar> status;
    std::vector<float> error;
    std::vector<float> min_eig_vec;
    // Criteria type, can be one of: COUNT, EPS or COUNT + EPS
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, klt_max_iter, klt_eps);// 默认值
    // 使用金字塔模型Pyr计算光流, (上一帧图片,当前帧图片,上一帧特征点,当前帧特征点,status返回成功与否的标志,error误差)
    cv::calcOpticalFlowPyrLK(image_ref, image_cur, // image_ref上一帧图片,img_curr当前帧图片
                             px_ref, px_cur, // px_ref上一帧特征点2D坐标,px_cur当前帧特征点2D坐标,必须为float单精度浮点数类型
                             status, error, // status返回成功与否的标志,error误差
                             cv::Size2i(klt_win_size, klt_win_size), // -winSize：每个金字塔层搜索窗大小。
                             4,  // -maxLevel：金字塔层的最大数目；如果置0，金字塔不使用(单层)；如果置1，金字塔2层，等等以此类推。
                             termcrit, // -criteria：指定搜索算法收敛迭代的类型
                             0); // -flags,默认为0

    std::vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();// 参考帧的关键点的坐标
    std::vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();// 追踪到的当前帧的关键点的坐标
    // 跟踪之后对应特征之间的像素距离存入std::vector<double>& disparities中,
    // 后续会对这个像素距离设定阈值，以确保初始位置确定的准确度。
    disparities.clear(); // 清空vector中的元素
    disparities.reserve(px_cur.size()); // 预留px_cur.size()个元素空间大小
    for (size_t i = 0; px_ref_it != px_ref.end(); ++i)
    {
        // 根据返回的status来删除跟丢的特征点,返回1表示跟踪到了,返回0表示未跟踪到
        if (!status[i]) // 未跟踪到,删除当前迭代器指向的元素,进行下一个循环
        {
            // 返回的迭代器指向删除元素的下一个元素
            px_ref_it = px_ref.erase(px_ref_it);
            px_cur_it = px_cur.erase(px_cur_it);
            continue;
        }
        // 没有跟丢,保存对应特征之前的像素距离,使用norm计算二范数,即欧式距离
        disparities.push_back(norm(cv::Point2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y)));
        // 迭代器递增
        ++px_ref_it;
        ++px_cur_it;
    }
}