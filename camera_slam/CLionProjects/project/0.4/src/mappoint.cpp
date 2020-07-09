#include "myslam/common_include.h"
#include "myslam/mappoint.h"

namespace myslam
{
// 默认构造函数,设定默认值
// 参数(ID、3D坐标、正常视角???、好(坏)点、内点、当前帧可观测???? 地图点被观测到的次数为0, 被匹配到的次数为0)
MapPoint::MapPoint()
: id_(-1), pos_(Vector3d(0,0,0)), norm_(Vector3d(0,0,0)), good_(true), visible_times_(0), matched_times_(0)
// id默认为-1, 世界坐标系下面的路标点坐标(0,0,0), normal初始化为(0,0,0), good默认为好的路标点,一个点被观测到的次数为0, 被匹配到的次数为0
{

}

// 构造函数
MapPoint::MapPoint ( unsigned long id, const Vector3d& position, const Vector3d& norm, Frame* frame, const Mat& descriptor )
: id_(id), pos_(position), norm_(norm), good_(true), visible_times_(1), matched_times_(1), descriptor_(descriptor)
// id默认为id, 世界坐标系下面的路标点坐标position, normal初始化为norm, good默认为好true的路标点,一个点被观测到的次数为1, 被匹配到的次数为1, 匹配的描述子为descriptor
{
    // observed_frames_,一个list用于记录观察到此地图点(路标点)的帧。
    observed_frames_.push_back(frame);
}


// 创建地图点MapPoint时，直接在累加上ID然后构造一个就好了。
// 返回定义的MapPoint类型智能指针Ptr
// 这里创建的是空的MapPoint路标点
MapPoint::Ptr MapPoint::createMapPoint()
{
    // factory_id是静态成员,默认初始化为0,全局存在,
    return MapPoint::Ptr( 
        new MapPoint( factory_id_++, Vector3d(0,0,0), Vector3d(0,0,0) )
        // 新的路标点, id自加1, 路标点坐标为(0,0,0),  norm为(0,0,0),
        // Frame* frame=nullptr记录观察到此地图点(路标点)的帧,
        // const Mat& descriptor=Mat()匹配的描述子为空
    );
}

// 使用已知的东西创建地图点MapPoint,
MapPoint::Ptr MapPoint::createMapPoint ( 
    const Vector3d& pos_world, // 已知的世界坐标系下的路标点坐标
    const Vector3d& norm, // norm
    const Mat& descriptor, // 匹配的描述子
    Frame* frame ) // 记录观察到此地图点(路标点)的帧的指针
    // Frame::Ptr  curr_;智能指针, curr_.get()返回curr_中保存的指针.
    // 要小心使用,若智能指针释放了其对象,返回的指针所指向的对象也就消失了
{
    return MapPoint::Ptr( 
        new MapPoint( factory_id_++, pos_world, norm, frame, descriptor )
    );
}
// static unsigned long factory_id_;静态成员,初始化为0
unsigned long MapPoint::factory_id_ = 0;

}
