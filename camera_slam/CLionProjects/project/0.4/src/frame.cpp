// frame帧,相机采集到的图像单位
#include "myslam/frame.h"

namespace myslam
{
Frame::Frame()// 默认构造函数
: id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
// 帧的ID默认为-1, 时间戳默认为-1, 指向Camera类的动态指针为空指针, 检测是否为关键帧的bool为false
{

}

// 构造函数
Frame::Frame ( long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat color, Mat depth )
: id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth), is_key_frame_(false)
// 帧的ID默认为id, 时间戳默认为time_stamp, T_c_w世界坐标系->相机坐标系的转换为T_c_w
// 指向Camera类的动态指针为camera, 彩色图为color, 深度图为depth, 检测是否为关键帧的bool为false
{

}

// 析构函数
Frame::~Frame()
{

}

// 创建Frame
Frame::Ptr Frame::createFrame()
{
    // factory_id对应的是MapPoint类中的factory_id??????是不是????
    static long factory_id = 0;// 在第一次进入这个函数的时候，变量factory_id被初始化为0,
    // 这里的factory_id++指的是构造函数中的id++
    // 创建只有编号的帧
    return Frame::Ptr( new Frame(factory_id++) );// 并接着自加1，以后每次进入该函数，就不会被再次初始化了，仅进行自加1的操作；
    // 在static发明前，要达到同样的功能，则只能使用全局变量

    // 由factory_id++一个数去构造Frame对象时，调用的是默认构造函数，
    // 由于默认构造函数全都有默认值，所以就是按顺序，先填第一个id_，
    // 所以也就是相当于创建了一个只有ID号的空白帧。
}

// 寻找给定关键点对应的深度
// 给一个关键点,返回一个double depth
double Frame::findDepth ( const cv::KeyPoint& kp )
{
    // cvRound()：返回跟参数最接近的整数值，即四舍五入；
    // cvFloor()：返回不大于参数的最大整数值，即向下取整；
    // cvCeil()：返回不小于参数的最小整数值，即向上取整；
    int x = cvRound(kp.pt.x); // x坐标
    int y = cvRound(kp.pt.y); // y坐标
    // Mat color_, depth
    // .ptr<>()函数得到一行的指针，并用[]操作符访问某一列的像素值。
    ushort d = depth_.ptr<ushort>(y)[x];
    // rgbd相机极有可能某个点没采集到深度值
    if ( d!=0 ) // 若深度值不为0,返回深度值/scale
    {
        // camera_->depth_scale_是camera类中的depth_scale_变量
        return double(d)/camera_->depth_scale_;
    }
    else 
    {
        // check the nearby points 左/下/右/上 四个点
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ )
        {
            d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
            if ( d!=0 )
            {
                return double(d)/camera_->depth_scale_;
            }
        }
    }
    return -1.0; // 如果失败,返回的深度值为double -1.0
}

// 世界坐标系->相机坐标系的变换
void Frame::setPose ( const SE3& T_c_w )
{
    T_c_w_ = T_c_w;
}

// 取相机光心的话，.translation()是取平移部分！不是取转置！！！！！！！！！
// T_c_w_.inverse()求出来的平移部分就是R^(-1)*(-t),
// 也就是相机坐标系下的(0,0,0)在世界坐标系下的坐标，也就是相机光心的世界坐标！！！！！！
// 获取相机光心
Vector3d Frame::getCamCenter() const
{
    // T_c_w_.inverse() 相机坐标系->世界坐标系的变换,
    // translation()取其中的平移部分xyz平移向量
    return T_c_w_.inverse().translation();
    // 相机的光心在相机坐标系下就是（0，0，0），
    // 所以求变换矩阵的逆之后，直接求对应的平移矩即R^(-1)*(-t)，
}

// 判断某个3D点是否在frame视野内
// pt_world世界坐标系下的坐标点
bool Frame::isInFrame ( const Vector3d& pt_world )
{
    // camera_->world2camera( pt_world, T_c_w_ );将世界坐标系点转换为相机坐标系下的点
    // p_cam相机坐标系下的点XYZ
    Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
    // cout<<"P_cam = "<< p_cam.transpose()<<endl;
    // Z小于0,不在frame视野内,返回false
    // Z小于0时,该点在相机平面后面,可能不会在相机平面上成像,应当检查Z是否大于0
    // p_cam(0,0): x,  p_cam(1,0): y,  p_cam(2,0): z
    if ( p_cam(2,0)<0 ) return false;

    // camera_->world2pixel( pt_world, T_c_w_ );将世界坐标系点转换为像素坐标系下的点
    // pixel像素坐标系下的点uv
    Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
    // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
    // u>0 && v>0 && u<cols && v<rows
    return pixel(0,0)>0 && pixel(1,0)>0 
        && pixel(0,0)<color_.cols 
        && pixel(1,0)<color_.rows;
}

}