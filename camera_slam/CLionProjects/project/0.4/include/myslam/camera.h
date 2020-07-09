// camera类存储相机的内参和外参,并完成相机坐标系/像素坐标系/世界坐标系之间的坐标转换
// 在世界坐标系中需要一个相机的(变动的)外参,需要以参数的形式传入


// 都有防止头文件重复引用的ifndef宏定义,
// 如果没有这个定义,在两处引用此头文件会出现类的重复定义,所以每个头文件中都会定义一个这样的宏
#ifndef CAMERA_H // if n define (防止类被重复定义)
#define CAMERA_H // 定义一个头文件

// 常用的头文件的定义,可避免每次书写很长的一串include
#include "myslam/common_include.h"

// 使用命名空间namespace myslam将类的定义包裹起来,
// 命名空间可以防止不小心定义出别的库同名的函数,是一种比较安全和规范的做法,
namespace myslam
{


/*
// 不用shared_ptr，直接的new
//声明相机、VO类以及构造函数
class PinholeCamera{
public:
    PinholeCamera(double width, double height,
                  double fx, double fy, double cx, double cy,
                  double k1 = 0.0, double k2 = 0.0, double p1 = 0.0, double p2 = 0.0, double k3 = 0.0);
};
class VisualOdometry{
public:
    VisualOdometry(PinholeCamera* cam);
}
// ----------------------------------------------------------
//调用。直接new，返回是指针类型cam
PinholeCamera *cam = new PinholeCamera(1241.0, 376.0,
                                       718.8560, 718.8560, 607.1928, 185.2157);
VisualOdometry vo(cam);
 */



// Pinhole RGBD camera model针孔模型
class Camera
{
public:
    // 将智能指针定义为Camera的指针类型,以后在传递参数的时使用Camera::Ptr类型即可
    // shared_ptr指向Camera类型的智能指针,智能指针也是模板,
    // typedef类型别名
    typedef std::shared_ptr<Camera> Ptr;
    // fx,fy指相机在x,y两个轴上的焦距，
    // cx,cy指相机的光心中心，depth指深度图的缩放因子。
    // 相机的内参fx, fy, cx, cy, 相机的外参:相机的位姿R,t,外参随着相机的运动而改变,同时也是SLAM待估计的目标,代表机器人的轨迹
    // 归一化平面?????
    float   fx_, fy_, cx_, cy_, depth_scale_;

    Camera(); // 默认构造函数,直接初始化
    Camera ( float fx, float fy, float cx, float cy, float depth_scale=0 ) :
        fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), depth_scale_ ( depth_scale )
    {}

    // 对于引用的操作就是对对象本身的操作，引用是被引用对象本身的一个“别名”，按引用传递参数与引用类似，形参成为了实参的一个“别名”
    // 当按引用传值时，在函数中修改形参的值时，实参的值也被修改了
    // 1. 避免进行拷贝操作 2. 可用于返回信息

    // 使用Sophus::SE3来表示相机的位姿,
    // coordinate transform: world, camera, pixel
    // 相机坐标系/像素坐标系/世界坐标系之间的坐标转换
    Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w );
    Vector3d camera2world( const Vector3d& p_c, const SE3& T_c_w );
    Vector2d camera2pixel( const Vector3d& p_c );
    Vector3d pixel2camera( const Vector2d& p_p, double depth=1 ); 
    Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth=1 );
    Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w );

};

}
#endif // CAMERA_H