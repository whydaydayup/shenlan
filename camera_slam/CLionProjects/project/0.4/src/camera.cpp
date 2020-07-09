#include "myslam/camera.h"
#include <myslam/config.h>

// p_c相机坐标系下的坐标
// p_w世界坐标系下的坐标
// p_p像素坐标系下的坐标
// T_c_w世界坐标系-->相机坐标系的转换
namespace myslam
{
    
Camera::Camera() // 默认构造函数
{
    fx_ = Config::get<float>("camera.fx");
    fy_ = Config::get<float>("camera.fy");
    cx_ = Config::get<float>("camera.cx");
    cy_ = Config::get<float>("camera.cy");
    depth_scale_ = Config::get<float>("camera.depth_scale");
}
// 世界坐标系-->相机坐标系  T_c_w世界坐标系到相机坐标系之间的变换
Vector3d Camera::world2camera ( const Vector3d& p_w, const SE3& T_c_w )
{
    return T_c_w*p_w;   // p_w是世界坐标系下的坐标
}
// 相机坐标系-->世界坐标系
Vector3d Camera::camera2world ( const Vector3d& p_c, const SE3& T_c_w )
{
    return T_c_w.inverse() *p_c;
}
// fx,fy指相机在x,y两个轴上的焦距，
// cx,cy指相机的光圈中心，s指深度图的缩放因子。
// 相机坐标系-->像素坐标系
// u = fx * X/Z + cx
// v = fy * Y/Z + cy
// Vector3d& p_c是X, Y, Z
Vector2d Camera::camera2pixel ( const Vector3d& p_c )
{
    return Vector2d (
               fx_ * p_c ( 0,0 ) / p_c ( 2,0 ) + cx_,
               fy_ * p_c ( 1,0 ) / p_c ( 2,0 ) + cy_
           );
}
// 像素坐标系-->相机坐标系
// X = (u - cx)*Z/fx
// Y = (v - cy)*Z/fy
// Z = depth
Vector3d Camera::pixel2camera ( const Vector2d& p_p, double depth )
{
    return Vector3d (
               ( p_p ( 0,0 )-cx_ ) *depth/fx_,
               ( p_p ( 1,0 )-cy_ ) *depth/fy_,
               depth
           );
}
// 世界坐标系-->像素坐标系 = 世界坐标系-->相机坐标系-->像素坐标系
Vector2d Camera::world2pixel ( const Vector3d& p_w, const SE3& T_c_w )
{
    return camera2pixel ( world2camera(p_w, T_c_w) );
}
// 像素坐标系-->世界坐标系 = 像素坐标系u.v-->相机坐标系XYZ-->世界坐标系
Vector3d Camera::pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth )
{
    return camera2world ( pixel2camera ( p_p, depth ), T_c_w );
}


}
