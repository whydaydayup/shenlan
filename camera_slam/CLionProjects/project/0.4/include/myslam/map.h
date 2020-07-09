// Map 类管理着所有的路标点,并负责添加新路标点、删除不好的路标点等工作。
// VO 的匹配过程只需要和 Map 打交道即可。
#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam
{
class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    // Map 类中实际存储了各个关键帧和路标点,既需要随机访问,又需要随时插入和删除,因此使用散列(Hash)来存储它们。
    // unordered_map用Hash函数组织的map,关联数组,保存关键字-值对
    // unordered_map<unsigned long, MapPoint::Ptr >是
    // unsigned long无符号long类型 键 - MapPoint::Ptr智能指针类型的值  对
    // all landmarks路标点的无序map
    unordered_map<unsigned long, MapPoint::Ptr >  map_points_;
    // all key-frames关键帧的无序map
    unordered_map<unsigned long, Frame::Ptr >     keyframes_;

    Map() {}// 默认构造函数

    // 添加一个关键帧
    void insertKeyFrame( Frame::Ptr frame );
    // 添加一个lanmark路标
    void insertMapPoint( MapPoint::Ptr map_point );
};
}

#endif // MAP_H
