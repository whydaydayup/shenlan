// 和0.3相比
// 1. ORB部分去掉了参考帧3D点、描述子
// 2. 匹配变成了地图点3D（match_3dpts_）和帧中关键点2d（match_2dkp_index_）
// 3. T变成了Tcw而不是之前的Tcr，当前帧直接和地图点区别绝对变化位姿，而非与参考帧之间找相对位姿变化
// 4. 增加的函数：优化地图、添加地图点、取得视角


#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"

#include <opencv2/features2d/features2d.hpp>

namespace myslam 
{
class VisualOdometry
{
public:
    // VisualOdometry类型的智能指针, 将此类型定义为Ptr
    // 之后用myslam::VisualOdometry::Ptr调用此智能指针,创建一个对象
    typedef shared_ptr<VisualOdometry> Ptr;
    // 枚举VO若干种状态
    enum VOState {
        INITIALIZING=-1, // 初始化,设定第一帧
        OK=0, // 正常,顺利跟踪
        LOST // 丢失
        // 可以把它看成一个有限状态机(Finite State Machine, FSM)
        // 当然状态也可以有更多种,例如单目 VO 至少还有一个初始化状态。
        // 在我们的实现中,考虑最简单的三个状态:初始化、正常、丢失。
    };
    // 把一些中间变量定义在类中,这样可省去复杂的参数传递。
    // 因为它们都是定义在类内部的,所以各个函数都可以访问它们。
    VOState     state_;     // current VO status, 当前的VO状态
    // 地图, 有关键帧keyframes_和路标点MapPoint_
    Map::Ptr    map_;       // map with all frames and map points
    
    Frame::Ptr  ref_;       // reference key-frame 参考帧
    Frame::Ptr  curr_;      // current frame 当前帧

    //在ORB部分去掉了关于参考帧的东西，3D点，描述子等
    // ORB特征提取: Oriented FAST关键点检测(一种改进的FAST角点) + BRIEF描述子计算
    cv::Ptr<cv::ORB> orb_;  // orb detector and computer,
    // 当前帧Oriented-FAST关键点 集合vector
    vector<cv::KeyPoint>    keypoints_curr_;    // keypoints in current frame
    // 当前帧BRIEF描述子,二进制描述子,描述向量由很多的0和1组成
    Mat                     descriptors_curr_;  // descriptor in current frame

    //在匹配器中，所需要的匹配变成了地图点mapPoint和当前帧中的关键点。
    // FLANN(快速近似最近邻)匹配
    cv::FlannBasedMatcher   matcher_flann_;     // flann matcher
    //cv::BFMatcher matcher_brute_; // brute-force matcher
    //!!!! 匹配变成了地图点3D和帧中关键点2d
    // 匹配的3D点坐标, MapPoint指向路标点的动态指针
    vector<MapPoint::Ptr>   match_3dpts_;       // matched 3d points 
    // 匹配的2D像素点(当前帧的关键点keypoints_curr_的索引index)
    vector<int>             match_2dkp_index_;  // matched 2d pixels (index of kp_curr)

    // !!!!! T变成了Tcw(世界坐标系下的当前帧的相机位姿)而不是之前的Tcr(当前帧下的参考帧的位姿)
    // 当前帧的估计的位姿T_c_w, 世界坐标系下的当前帧的相机坐标
    SE3 T_c_w_estimated_;    // the estimated pose of current frame 
    // 内点数
    int num_inliers_;        // number of inlier features in icp
    // 丢失的特征点数量
    int num_lost_;           // number of lost times
    
    // parameters 参数
    // ORB特征提取, 关键点的检测和描述子的计算
    int num_of_features_;   // number of features 最多提取的特征点数量
    double scale_factor_;   // scale in image pyramid 金字塔图像的尺度参数
    int level_pyramid_;     // number of pyramid levels 高斯金字塔的层数
    // 好的匹配点的筛选系数
    float match_ratio_;     // ratio for selecting  good matches
    // 最大持续丢失时间
    int max_num_lost_;      // max number of continuous lost times连续跟踪丢失的次数
    // 最小内点数
    int min_inliers_;       // minimum inliers最少的内点数量
    // 选择关键帧标准，最小旋转和平移
    double key_frame_min_rot;   // minimal rotation of two key-frames两个关键帧之间最小的旋转大小
    double key_frame_min_trans; // minimal translation of two key-frames两个关键帧之间最小的平移大小

    // 匹配率=匹配次数/可见次数, 最低匹配率0.1
    double  map_point_erase_ratio_; // remove map point ratio删除路标点的系数
    
public: // functions 
    VisualOdometry();  // 默认构造函数
    ~VisualOdometry(); // 默认析构函数

    // 添加新的图像帧
    bool addFrame( Frame::Ptr frame ); // add a new frame

// protected对于子女、朋友来说，就是public的，可以自由使用，没有任何限制，
// 而对于其他的外部class，protected就变成private。
protected:

    // inner operation 内部处理函数
    void extractKeyPoints(); // 关键点的提取
    void computeDescriptors(); // 描述子的计算
    void featureMatching(); // ORB特征匹配
    void poseEstimationPnP(); // PnP位姿估计
    void optimizeMap();	// 增加的优化地图函数,实现对整个地图的优化

    // 关键帧的功能函数
    void addKeyFrame();
    void addMapPoints(); // 增加的添加地图点函数addMapPoints()
    bool checkEstimatedPose(); 
    bool checkKeyFrame();
        // 增加的取得视角函数getViewAngle(frame,point)
    double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );
    
};
}

#endif // VISUALODOMETRY_H
