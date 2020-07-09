// 将 VO 匹配到的特征点放到地图中,并将当前帧与地图点进行匹配,计算位姿。
// 在两两帧间比较时,只计算参考帧与当前帧之间的特征匹配和运动关系,在计算之后把当前帧设为新的参考帧。
// 而在使用地图的 VO 中,每个帧为地图贡献一些信息,比方说添加新的特征点或更新旧特征点的位置估计。
// 地图中的特征点位置往往是使用世界坐标的。
// 因此,当前帧到来时,我们求它和地图之间的特征匹配与运动关系,即直接计算了T_cw。

// 好处是,我们能够维护一个不断更新的地图。只要地图是正确的,即使中间某帧出了差错,
// 仍有希望求出之后那些帧的正确位置。
// 请注意,我们现在还没有详细地讨论 SLAM 的建图问题,所以这里的地图仅是一个临时性的概念,
// 指的是把各帧特征点缓存到一个地方,构成了特征点的集合,我们称它为地图。

// 地图又可以分为局部(Local)地图和全局(Global)地图两种,由于用途不同,往往分开讨论。
// 局部地图描述了附近的特征点信息——我们只保留离相机当前位置较近的特征点,而把远的或视野外的特征点丢掉。
// 这些特征点是用来和当前帧匹配来求相机位置的,所以我们希望它能够做的比较快。
// 另一方面,全局地图则记录了从 SLAM 运行以来的所有特征点。
// 它显然规模要大一些,主要用来表达整个环境,
// 但是直接在全局地图上定位,对计算机的负担就太大了。它主要用于回环检测和地图表达。


// 在视觉里程计中,我们更关心可以直接用于定位的局部地图(如果决心要用地图的话).
// 所以本讲维护一个局部地图。随着相机运动,我们往地图里添加新的特征点,并去掉之前,
// 我们仍然要提醒读者:是否使用地图取决你对精度——效率这个矛盾的把握。
// 我们完全可以出于效率的考量,使用两两无结构式的 VO;
// 也可以为了更好的精度,构建局部地图乃至考虑地图的优化。

// 局部地图的一件麻烦事是维护它的规模。
// 为了保证实时性,我们需要保证地图规模不至于太大(否则匹配会消耗大量的时间)。
// 此外,单个帧与地图的特征匹配存在着一些加速手段,但由于它们技术上比较复杂,例程中就不给出



// 视觉里程计能够估算局部时间内的相机运动以及特征点的位置,但是这种局部的方式有明显的缺点:
// 1.容易丢失。一旦丢失,我们要么“等待相机转回来”(保存参考帧并与新的帧比较),要么重置整个 VO 以跟踪新的图像数据。
// 2. 轨迹漂移。主要原因是每次估计的误差会累计至下一次估计,导致长时间轨迹不准确。
// 大一点儿的局部地图可以缓解这种现象,但它始终是存在的。
// 值得一提的是,如果只关心短时间内的运动,或者 VO 的精度已经满足应用需求,那么有时候你可能需要的仅仅就是一个视觉里程计,而不用完全的 SLAM。


//      实时定位的需要的是局部地图，全局地图一般用于回环检测和整体建图!!!!
// -------------- test the visual odometry -------------
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"

int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }

    // namespace::class::member_function()
    // 使用作用域运算符直接访问静态成员setParameterFile
    // 静态成员不与任何对象绑定在一起,不包含this指针
    // myslam::Config::setParameterFile ( argv[1] );
    // namespace::类::成员函数
    myslam::Config::setParameterFile ( "./config/default.yaml" ); // 创建参数Config类
    // typedef shared_ptr<VisualOdometry> Ptr;
    // 表示引用成员函数及变量，作用域成员运算符, System::Math::Sqrt() 相当于System.Math.Sqrt()
    // myslam::VisualOdometry::shared_ptr<VisualOdometry> vo一个可以指向VisualOdometry类的智能指针
    // new myslam::VisualOdometry默认构造函数,读取参数配置类Config设置
    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );

    string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );// 读取文件地址
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin ( dataset_dir+"/associate.txt" );// ifstream文件输入流
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;// 图片名数组
    vector<double> rgb_times, depth_times;// 时间戳数组
    while ( !fin.eof() )// 没有到达文件末尾
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;// 输出文件中每行的数据
        // atof表示的是将string字符串转换成为double浮点数
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );

        if ( fin.good() == false )// 文件
            break;
    }

    // typedef std::shared_ptr<Camera> Ptr;
    // 创建相机类Camera的对象
    // 使用默认构造函数构造Camera类, 默认构造函数读取参数配置Config类
    myslam::Camera::Ptr camera ( new myslam::Camera );

    // 用 OpenCV3 的 viz 模块显示估计位姿
    // visualization
    // 1. 创建可视化窗口
    cv::viz::Viz3d vis ( "Visual Odometry" );
    // 2. 创建坐标系部件，参数是坐标系长度
    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    // 3. （可选）设置视角，相机位置坐标，相机焦点坐标，相机y轴朝向
    cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    // 视角位姿
    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    // 设置观看视角
    vis.setViewerPose ( cam_pose );

    // 渲染属性，第一个参数是枚举这里是线宽，后面数值
    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
    // showWeiget函数将部件添加到窗口内
    vis.showWidget ( "World", world_coor );
    vis.showWidget ( "Camera", camera_coor );

    cout<<"read total "<<rgb_files.size() <<" entries"<<endl;
    for ( int i=0; i<rgb_files.size(); i++ )
    {
        cout<<"****** loop "<<i<<" ******"<<endl;
        Mat color = cv::imread ( rgb_files[i] );
        Mat depth = cv::imread ( depth_files[i], -1 );
        if ( color.data==nullptr || depth.data==nullptr )
            break;
        // typedef std::shared_ptr<Frame> Ptr;
        // 创建帧Frame的对象
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        // 设定帧中的成员:相机类camera_, 彩色图color_, 深度图depth_, 时间戳time_stamp_
        pFrame->camera_ = camera; // Camera::Ptr camera_;
        pFrame->color_ = color; // Mat color_, depth_;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];// double time_stamp_;

        boost::timer timer; // 每帧的运算时间，看实时性
        // 指针调用成员函数
        vo->addFrame ( pFrame );// 添加帧, 即调用addFrame估计其位姿
        cout<<"VO costs time: "<<timer.elapsed()*1000 << "ms" <<endl;

        if ( vo->state_ == myslam::VisualOdometry::LOST ) // 若跟踪失败
            break; // 终止循环
        //可视化窗口动的是相机坐标系，求相机坐标系下的点在世界坐标系下的坐标
        SE3 Twc = pFrame->T_c_w_.inverse();

        // show the map and the camera pose
        // 旋转rotation和平移translation
        cv::Affine3d M (
            cv::Affine3d::Mat3 (
                Twc.rotation_matrix() ( 0,0 ), Twc.rotation_matrix() ( 0,1 ), Twc.rotation_matrix() ( 0,2 ),
                Twc.rotation_matrix() ( 1,0 ), Twc.rotation_matrix() ( 1,1 ), Twc.rotation_matrix() ( 1,2 ),
                Twc.rotation_matrix() ( 2,0 ), Twc.rotation_matrix() ( 2,1 ), Twc.rotation_matrix() ( 2,2 )
            ),
            cv::Affine3d::Vec3 (
                Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
            )
        );

        // 局部地图的点投影到图像平面并显示出来。
        // 如果位姿估计正确的话,它们看起来应该像是固定在空间中一样。
        // 反之,如果感觉到某个特征点不自然地运动,
        // 可能是相机位姿估计不够准确,或特征点的位置不够准确。
        Mat img_show = color.clone();
        for ( auto& pt:vo->map_->map_points_ )
        {
            myslam::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
            cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
        }

        cv::imshow ( "image", img_show );
        cv::waitKey ( 1 );
        // viz可视化窗口
        vis.setWidgetPose ( "Camera", M );
        vis.spinOnce ( 1, false );
        cout<<endl;
    }

    return 0;
}
