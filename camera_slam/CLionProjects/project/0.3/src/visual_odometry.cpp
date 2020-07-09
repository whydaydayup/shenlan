#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include "myslam/g2o_types.h"

namespace myslam
{

VisualOdometry::VisualOdometry() :
    state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_( new cv::flann::LshIndexParams(5,10,2) )
{
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    map_point_erase_ratio_ = Config::get<double> ("map_point_erase_ratio");
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
}

VisualOdometry::~VisualOdometry()
{

}
// 基本的VO
// 1. 对新来的当前帧,提取关键点和描述子。   2DKeyPoints点
// 2. 如果系统未初始化,以该帧为参考帧,根据深度图计算关键点的3D位置,返回1。
// 3. 估计参考帧与当前帧间的运动。    T_c_r参考帧坐标系下的当前帧的坐标,
// 4. 判断上述估计是否成功。
// 5. 若成功,把当前帧作为新的参考帧,返回1。
// 6. 若失败,计连续丢失帧数。当连续丢失超过一定帧数,置 VO 状态为丢失,算法结束。若未超过,返回 1。
bool VisualOdometry::addFrame ( Frame::Ptr frame )
{
    switch ( state_ )
    {
    case INITIALIZING:// 1.初始化状态
    {
        state_ = OK;
        curr_ = ref_ = frame;// 参考帧=当前帧=第一帧frame
        map_->insertKeyFrame ( frame );// 将当前帧插入到关键帧中
        // extract features from first frame and add them into map
        // 从第一帧提取关键点, 计算描述子
        extractKeyPoints();
        computeDescriptors();

        // compute the 3d position of features in ref frame
        // 参考帧特征点的3D坐标,补全depth数据
        setRef3DPoints();
        break;
    }
    case OK: // 2.匹配成功状态
    {
        curr_ = frame; // 当前帧=frame
        // ORB特征计算: 关键点的提取和描述子的计算
        extractKeyPoints();// 关键点提取
        computeDescriptors();// 描述子计算

        featureMatching();// ORB特征匹配

        // 参考帧3d点,和当前帧2d点计算位姿,  在参考帧的基础上,当前帧的位姿,T_c_r
        poseEstimationPnP();// 3d-2d EPnP位姿计算

        if ( checkEstimatedPose() == true ) // a good estimation
        {
            // 以参考帧ref为坐标系,把当前帧curr与它进行特征匹配,并估计运动关系。
            // 假设参考帧相对世界坐标的变换矩阵为T_r_w ,当前帧与世界坐标系间为T_c_w ,
            // 两个帧的变换矩阵T_c_r,参考帧到当前帧的变换关系
            // 则待估计的运动与这两个帧的变换矩阵构成左乘关系:T_c_w = T_c_r * T_r_w
            curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;  // T_c_w = T_c_r*T_r_w 
            ref_ = curr_; // 参考帧 = 当前帧
            setRef3DPoints();// 参考帧特征点的3D坐标,补全depth数据
            num_lost_ = 0;
            if ( checkKeyFrame() == true ) //检查是否为关键帧
            {
                addKeyFrame(); // 是关键帧,则将其添加到map中去
            }
        }
        //位姿估计坏，丢失点计数+1，判断是否大于最大丢失数，是则VO状态切换为lost
        else // bad estimation due to various reasons
        {
            num_lost_++;
            if ( num_lost_ > max_num_lost_ )
            {
                state_ = LOST;
            }
            return false;
        }
        break;
    }
    case LOST:// 3.位姿丢失状态
    {
        cout<<"vo has lost."<<endl;
        break;
    }
    }

    return true;
}
// 对当前帧curr进行关键点的检测和描述子的计算
void VisualOdometry::extractKeyPoints()
{
    boost::timer timer;
    orb_->detect ( curr_->color_, keypoints_curr_ );
    cout<<"extract keypoints cost time: "<<timer.elapsed()<<endl;
}
// 对当前帧计算描述子
void VisualOdometry::computeDescriptors()
{
    boost::timer timer;
    orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
    cout<<"descriptor computation cost time: "<<timer.elapsed()<<endl;
}
// 直接通过前后两帧(参考帧ref/当前帧curr)之间的描述子进行特征匹配
void VisualOdometry::featureMatching()
{
    boost::timer timer;
    vector<cv::DMatch> matches;
    // 暴力匹配
//    vector<cv::DMatch> matches;
//    cv::BFMatcher matcher ( cv::NORM_HAMMING );
    matcher_flann_.match( descriptors_ref_, descriptors_curr_, matches );
    // select the best matches
    // 找到match数组中最小距离,赋值给min_distance
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    // vector<cv::DMatch> feature_matches_;匹配对vector,
    // vector::clear()清空容器中的内容，但如果容器中的对象是指针，并不能清空指针指向的内容,返回void
    // vector::erase(args)删除args指定的元素,并返回指向删除元素的下一个元素的迭代器
    feature_matches_.clear(); // 清空vector中的成员
    // 根据距离筛选匹配的特征点
    for ( cv::DMatch& m : matches )// 遍历每个matche匹配对
    {
        // 每个匹配对的距离 < max(最小距离*match_ratio(2.0) , 30.0)
        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            feature_matches_.push_back(m);
        }
    }
    cout<<"good matches: "<<feature_matches_.size()<<endl;
    cout<<"match cost time: "<<timer.elapsed()<<endl;
}

// pnp需要参考帧3D,当前帧2D，所以当前帧迭代为参考帧时，需要加上depth数据
void VisualOdometry::setRef3DPoints()
{
    // select the features with depth measurements
    // 参考帧相机坐标系下的3D坐标XYZ
    pts_3d_ref_.clear(); // 清空 vector中的元素
    descriptors_ref_ = Mat(); // 清空 参考帧的描述子
    for ( size_t i=0; i<keypoints_curr_.size(); i++ )
    {
        // 查找当前帧的depth,
        // 当前帧变为参考帧
        double d = ref_->findDepth(keypoints_curr_[i]);
        if ( d > 0)
        {
            // 将像素坐标系uv转换相机坐标系的XYZ
            Vector3d p_cam = ref_->camera_->pixel2camera(
                    Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d
            );
            // 构造Point3f, 3D坐标, 相机坐标系下的XYZ
            pts_3d_ref_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ));
            // 参考帧描述子，将当前帧描述子按行放进去
            descriptors_ref_.push_back(descriptors_curr_.row(i));
        }
    }
}

// 直接PnP进行位姿估计, 在参考帧ref下的当前帧curr的位姿变换T_c_r
void VisualOdometry::poseEstimationPnP()
{
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d; // 参考帧3D坐标
    vector<cv::Point2f> pts2d; // 当前帧2D坐标

    // 遍历所有的匹配关系
    for ( cv::DMatch m:feature_matches_ )
    {
        pts3d.push_back( pts_3d_ref_[m.queryIdx] ); // query descriptor index查询序号
        // .pt像素坐标
        pts2d.push_back( keypoints_curr_[m.trainIdx].pt );// trainIdx训练描序号
    }
    // K内参矩阵: fx   0  cx
    //           0  fy  cy
    //           0   0   1
    Mat K = ( cv::Mat_<double>(3,3)<<
                                   ref_->camera_->fx_, 0, ref_->camera_->cx_,
            0, ref_->camera_->fy_, ref_->camera_->cy_,
            0,0,1
    );

    //旋转向量(分别是绕xyz的弧度)、平移向量、内点数组
    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers, cv::SOLVEPNP_EPNP );
    //内点数为内点行数
    num_inliers_ = inliers.rows;
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    //由旋转向量和平移向量构造出当前帧相对于参考帧的T
    // 李群SE3
    T_c_r_estimated_ = SE3(
            SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)),
            Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );
    cout << "T_c_r before BA" << endl << T_c_r_estimated_ << endl;

    // 新增的优化仍然是无结构的,规模小,对时间的影响几乎可以忽略不计
    // using bundle adjustment to optimize the pose 
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
        T_c_r_estimated_.rotation_matrix(), 
        T_c_r_estimated_.translation()
    ) );
    optimizer.addVertex ( pose );

    // edges
    for ( int i=0; i<inliers.rows; i++ )
    {
        int index = inliers.at<int>(i,0);
        // 3D -> 2D projection
        EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setId(i);
        edge->setVertex(0, pose);
        edge->camera_ = curr_->camera_.get();
        edge->point_ = Vector3d( pts3d[index].x, pts3d[index].y, pts3d[index].z );
        edge->setMeasurement( Vector2d(pts2d[index].x, pts2d[index].y) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        optimizer.addEdge( edge );
    }
    
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    
    T_c_r_estimated_ = SE3 (
        pose->estimate().rotation(),
        pose->estimate().translation()
    );
    cout << "T_c_r after BA" << endl << T_c_r_estimated_ << endl;
}

//位姿检验模块，匹配点不能太少，运动不能太大
bool VisualOdometry::checkEstimatedPose()
{
    // check if the estimated pose is good
    //匹配点太少，false
    if ( num_inliers_ < min_inliers_ ) // min_inliers_配置文件中为10
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    // SE3 T_c_r_estimated_;
    // 李群SE3-->.log()李代数se3, 一个6维向量,前三个为平移分量,后三个为旋转分量
    // typedef Matrix< double, 6, 1 > Vector6d;
    Sophus::Vector6d d = T_c_r_estimated_.log();
    // x.norm() 二范数，欧氏距离，（注意：Eigen中没有norm(R)）
    // x.squaredNorm() 二范数的平方和，（注意：对于复数而言，不等价）
    if ( d.norm() > 5.0 ) // 二范数>5,运动过大
    {
        cout<<"reject because motion is too large: "<<d.norm()<<endl;
        return false;
    }
    return true;
}

// 检查关键帧，T中R或t比较大都是关键帧
bool VisualOdometry::checkKeyFrame()
{
    // typedef Matrix< double, 6, 1 > Vector6d;
    // Eigen::Matrix<double, 6, 1> d = T_c_r_estimated_.log();
    Sophus::Vector6d d = T_c_r_estimated_.log(); // 6维向量
    Eigen::Vector3d trans = d.head<3>(); // 前3个变量是平移分量
    Eigen::Vector3d rot = d.tail<3>(); // 后3个变量是旋转分量
    // 平移分量的二范数>关键帧的最小旋转二范数,|| 或  旋转分量的二范数>关键帧的最小二范数
    if ( rot.norm() > key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;// 返回true表示是关键帧
    return false; // 返回false表示不是关键帧
}
// 添加关键帧

void VisualOdometry::addKeyFrame()
{
    cout<<"adding a key-frame"<<endl;
    map_->insertKeyFrame ( curr_ );
}

}
