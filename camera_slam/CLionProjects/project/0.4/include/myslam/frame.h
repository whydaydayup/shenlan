// Frame类中,定义了一个帧中含有的最重要的信息,如ID、时间戳、位姿、相机、图像这几个量,
// 还有几个重要的方法:创建 Frame、寻找给定点对应的深度、获取相机光心、判断某个点是否在视野内等等。

#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam
{
    
// forward declare类的前向声明!!!!
class MapPoint;
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;// 指向Frame类的动态指针的别名
    unsigned long                  id_;         // id of this frame, ID
    double                         time_stamp_; // when it is recorded, 时间戳
    SE3                            T_c_w_;      // transform from world to camera,世界坐标系-->相机坐标系的转换
    Camera::Ptr                    camera_;     // Pinhole RGBD Camera model,指向Camera类的动态指针
    Mat                            color_, depth_; // color and depth image,彩色图/深度图
    // std::vector<cv::KeyPoint>      keypoints_;  // key points in image
    // std::vector<MapPoint*>         map_points_; // associated map points
    bool                           is_key_frame_;  // whether a key-frame, 是不是一个关键帧
    
public: // data members 
    Frame();// 默认构造函数
    Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat() );
    ~Frame();

    // factory function
    // createFrame()函数,返回一个指向Frame类的静态 动态指针
    // 调用了默认构造函数创建了个空白帧对象，唯一赋值的参数就是给id_赋了个值。
    static Frame::Ptr createFrame(); // 创建Frame????
    
    // find the depth in depth map寻找color图给定点对应的深度
    // 给一个关键点,返回一个double depth
    double findDepth( const cv::KeyPoint& kp );
    
    // Get Camera Center获取相机光心?????
    Vector3d getCamCenter() const;
    
    void setPose( const SE3& T_c_w ); // 设置世界坐标系到相机坐标系的变换????
    
    // check if a point is in this frame判断某个点是否在frame视野内
    bool isInFrame( const Vector3d& pt_world );
};

}

#endif // FRAME_H
