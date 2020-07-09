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
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOState {
        INITIALIZING=-1,
        OK=0,
        LOST
    };
    
    VOState     state_;     // current VO status
    Map::Ptr    map_;       // map with all frames and map points
    
    Frame::Ptr  ref_;       // reference key-frame 
    Frame::Ptr  curr_;      // current frame 
    //orb特征
    cv::Ptr<cv::ORB> orb_;  // orb detector and computer 
    // 参考帧相机坐标系下的3D坐标XYZ
    vector<cv::Point3f>     pts_3d_ref_;        // 3d points in reference frame
    // 当前帧特征点KeyPoint
    vector<cv::KeyPoint>    keypoints_curr_;    // keypoints in current frame
    // 当前帧描述子
    Mat                     descriptors_curr_;  // descriptor in current frame
    // 参考帧描述子
    Mat                     descriptors_ref_;   // descriptor in reference frame
    // 匹配关系
    vector<cv::DMatch>      feature_matches_;   // feature matches
    // flann匹配
    cv::FlannBasedMatcher   matcher_flann_;     // flann matcher

    // 在上一帧情况下当前帧位姿,即上一帧和当前帧的转换关系
    SE3 T_c_r_estimated_;    // the estimated pose of current frame 
    int num_inliers_;        // number of inlier features in icp
    int num_lost_;           // number of lost times
    
    // parameters 
    int num_of_features_;   // number of features
    double scale_factor_;   // scale in image pyramid
    int level_pyramid_;     // number of pyramid levels
    float match_ratio_;     // ratio for selecting  good matches
    int max_num_lost_;      // max number of continuous lost times
    int min_inliers_;       // minimum inliers

    // 关键帧标准，最小旋转和平移
    double key_frame_min_rot;   // minimal rotation of two key-frames
    double key_frame_min_trans; // minimal translation of two key-frames


    double  map_point_erase_ratio_; // remove map point ratio
    
public: // functions 
    VisualOdometry();// 默认构造函数
    ~VisualOdometry();// 默认析构函数
    
    bool addFrame( Frame::Ptr frame );      // add a new frame 
    
protected:  
    // inner operation 
    void extractKeyPoints();
    void computeDescriptors(); 
    void featureMatching();
    void setRef3DPoints();
    void poseEstimationPnP(); 
    
    void addKeyFrame();
    bool checkEstimatedPose(); 
    bool checkKeyFrame();
    
};
}

#endif // VISUALODOMETRY_H
