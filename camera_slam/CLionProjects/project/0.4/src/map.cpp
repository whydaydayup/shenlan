#include "myslam/map.h"

namespace myslam
{
// 输入:指向当前帧的智能指针,     向关键帧的序列中添加关键帧
void Map::insertKeyFrame ( Frame::Ptr frame )
{
    // unordered_map<unsigned long, Frame::Ptr >  keyframes_;
    // 所有的关键帧的键值对
    cout<<"Key frame size = "<<keyframes_.size()<<endl;// 所有关键帧的数量
    // frame->id_帧的ID,
    // keyframes_.find(frame->id_)返回一个迭代器,指向key==frame->id_的元素
    if ( keyframes_.find(frame->id_) == keyframes_.end() ) // 没有这个关键帧
    {
        // make_pair(frame->id_, frame),
        // make_pair用两个元素初始化的pair,pair的类型由这两个元素推断而来
        // insert添加一个元素/对象
        // 总共有四种方法添加一个元素
        //keyframes_.insert( make_pair(frame->id_, frame) );
        keyframes_.insert( {frame->id_, frame} );
        //keyframes_.insert( pair<unsigned long, Frame::Ptr >(frame->id_, frame) );
        //keyframes_.insert( unordered_map<unsigned long, Frame::Ptr >::value_type {frame->id_, frame} );
    }
    else // 已经有了这个关键帧
    {
        // 则覆盖当前的关键帧,
        // 键key=unsigned long(frame->id)_, 值value=Frame::Ptr(frame)
        keyframes_[ frame->id_ ] = frame;
    }
}

// 输入: 指向路标点的智能指针,    向路标点的序列中添加路标点
void Map::insertMapPoint ( MapPoint::Ptr map_point )
{
    cout<<"map points size = "<<map_points_.size()<<endl;
    // unordered_map<unsigned long, MapPoint::Ptr >  map_points_;
    // 所有路标点的键值对
    // map_point->id路标点的ID
    // map_points_.find(map_point->id_)返回一个迭代器,指向键值key==map_point->id的元素
    if ( map_points_.find(map_point->id_) == map_points_.end() )// 没有这个路标点
    {
        // {路标点的ID, 指向路标点的智能指针}组成一个键值对
        // 添加到路边点序列中去
        map_points_.insert( make_pair(map_point->id_, map_point) );
    }
    else // 当前路标点的ID已经存在
    {
        // 将当前的ID键指向的值 = 指向当前的路标点的智能指针
        map_points_[map_point->id_] = map_point;
    }
}


}
