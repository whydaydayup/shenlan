// MapPoint 表示路标点。
// 我们将估计它的世界坐标,并且会拿当前帧提取到的特征点与地图中的路标点匹配,
// 来估计相机的运动,因此还需要存储它对应的描述子。
// 此外会记录一个点被观测到的次数和被匹配到的次数,作为评价它的好坏程度的指标。
#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam
{
    
class Frame; // Frame类,定义了一个图像帧
class MapPoint // 路标点
{
public:
    // 地图点本质上就是空间中3D点，成员函数为地图点的基本信息。
    typedef shared_ptr<MapPoint> Ptr;// 指向路标点的动态指针
    unsigned long      id_;        // ID, 路标点的ID
    // 静态成员和全局变量没有初始化的时候,系统自动初始化为0
    static unsigned long factory_id_;    // factory id静态成员
    bool        good_;      // wheter a good point 是不是好的路标点
    Vector3d    pos_;       // Position in world 路标点在世界坐标系下的位置
    Vector3d    norm_;      // Normal of viewing direction
    Mat         descriptor_; // Descriptor for matching 匹配的描述子

    // Frame*类型的 list，用于保存 观察到此地图点(路标点)的帧。
    // list双向链表,支持双向访问,在任何位置添加/删除操作都很快
    list<Frame*>    observed_frames_;   // key-frames that can observe this point 

    // 评价路标点好坏程度的指标
    int         matched_times_;     // being an inliner in pose estimation作为内点,被匹配到的次数
    int         visible_times_;     // being visible in current frame一个点被观测到的次数
    
    MapPoint();// 构造函数
    MapPoint( 
        unsigned long id, // id
        const Vector3d& position, // 世界坐标系下的路标点坐标
        const Vector3d& norm,  // normal
        Frame* frame=nullptr,  // 记录观察到此地图点(路标点)的帧
        const Mat& descriptor=Mat() // 匹配的描述子
    );
    // 内联函数, 取得地图点(路标点)3维坐标的功能函数
    inline cv::Point3f getPositionCV() const {
        return cv::Point3f( pos_(0,0), pos_(1,0), pos_(2,0) );
    }

    // factory function地图点生成函数
    static MapPoint::Ptr createMapPoint();// 默认的生成地图点(路标点)
    static MapPoint::Ptr createMapPoint( 
        const Vector3d& pos_world, // 3D坐标
        const Vector3d& norm_, //
        const Mat& descriptor, // 描述子
        Frame* frame ); // 指向图像帧的智能指针
};
}

#endif // MAPPOINT_H
