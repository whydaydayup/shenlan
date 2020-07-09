// 0.2版本的VO 是无结构的,特征点的 3D 位置被当作真值来估计运动。
// 但实际上,RGB-D 的深度图必定带有一定的误差,特别是那些深度过近或过远的地方。
// 并且,由于特征点往往位于物体的边缘处,那些地方的深度测量值通常是不准确的。
// 所以现在的做法不够精确,需要把特征点也放在一起优化。

// 只考虑参考帧/当前帧的方式,一方面使得位姿估计过于依赖参考帧。
// 如果参考帧质量太差,比如出现严重遮挡、光照变化的情况下,跟踪容易会丢失。
// 并且,当参考帧位姿估计不准时,还会出现明显的漂移。
// 另一方面,仅使用两帧数据显然没有充分地利用所有的信息。
// 更自然的方式是比较当前帧和地图,而不是比较当前帧和参考帧。
// 于是,我们要关心如何把当前帧与地图进行匹配,以及如何优化地图点的问题。


// 本节的例程会把局部地图的点投影到图像平面并显示出来。
// 如果位姿估计正确的话,它们看起来应该像是固定在空间中一样。
// 反之,如果你感觉到某个特征点不自然地运动,
// 那可能是相机位姿估计不够准确,或特征点的位置不够准确
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
// 默认构造函数,
VisualOdometry::VisualOdometry() :
    state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_ ( new cv::flann::LshIndexParams ( 5,10,2 ) )
    //state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_brute_ ( cv::NORM_HAMMING )
// 相机位姿初始化VOState初始化,设定第一帧, 指向参考帧ref_/当前帧curr_的智能指针为空指针nullptr, 保存Mappoint和
{
    // 特征提取和匹配当中的参数,从参数文件中读取。
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    //
    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );
    // ORB特征提取
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
}

// 析构函数
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




// addFrame函数是外部调用的接口。
// 使用VO时,将图像数据装入Frame类后,调用addFrame估计其位姿。
// 该函数根据 VO 所处的状态实现不同的操作:

// addFrame函数，没有依赖参考帧，删除了setRef3DPoint函数，多了和地图之间操作
bool VisualOdometry::addFrame ( Frame::Ptr frame )
{
    switch ( state_ )
    {
    case INITIALIZING: // 1. 初始化状态,第一帧
    {
        state_ = OK; // 将VOState置为OK,正常状态
        curr_ = ref_ = frame; // 当前帧和参考帧都是第一帧
        // extract features from first frame and add them into map
        // 当前帧(第一帧)的关键点检测和描述子计算
        extractKeyPoints();
        computeDescriptors();

        // setRef3DPoints(); // 参考帧特征点的3D坐标,补全depth数据
        // 老版本补全depth数据,新版本将第一帧加入到关键帧中

        // 之前用map类中insertKeyFrame函数,现在用的是visual_odometry中addKeyFrame()函数
        // 直接将第一帧加入到关键帧中,同时将所有的路标点mapPoint加入到地图中
        addKeyFrame(); // the first frame is a key-frame
        break;
    }
    case OK: // 2.匹配成功状态
    {
        // 整个流程的改变就是只需要不断进行每一帧的位姿迭代，
        // 而不需要用到参考帧的3D点进行匹配得T了
        curr_ = frame; // 当前帧=frame
        // 先将参考帧ref_的位姿T赋值给当前帧curr_
        // 1. 因为考虑到匹配失败的情况下，这一帧就定义为丢失了，所以位姿就用参考帧的了。
        // 如果一切正常，求得了当前帧的位姿，就进行赋值覆盖掉就好了
        curr_->T_c_w_ = ref_->T_c_w_;

        extractKeyPoints();// 关键点提取
        computeDescriptors();// 描述子计算

        featureMatching(); // ORB特征匹配,主要是使用描述子进行匹配

        // T变成了Tcw而不是之前的Tcr，
        // 当前帧直接和地图点之间绝对变化位姿，而非与参考帧之间找相对位姿变化
        poseEstimationPnP();
        if ( checkEstimatedPose() == true ) // a good estimation
        {
            // 计算的不是两帧之间的变换关系(参考帧到当前帧的变换关系)T_c_r
            // 计算的是当前帧相对于世界坐标系的位姿T_c_w
            // 2.正常求得位姿T，对当前帧位姿进行更新
            curr_->T_c_w_ = T_c_w_estimated_;
            optimizeMap(); // 优化地图,删除一些地图点,在点过少时增加地图点
            num_lost_ = 0;
            if ( checkKeyFrame() == true ) // is a key-frame
            {
                addKeyFrame(); // 是关键帧,则将其添加到map中去
            }
        }
        // 3. 考虑到匹配失败的情况下，当前帧也不会没有位姿，只是还是前一帧的罢了。
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
    case LOST: // 3.位姿丢失状态
    {
        cout<<"vo has lost."<<endl;
        break;
    }
    }

    return true;
}
// 特征点的提取和匹配占据了绝大多数的计算时间,
// 而看似复杂的 PnP 优化,计算量与之相比基本可以忽略。
// 因此,如何提高特征提取、匹配算法的速度,将是特征点方法的一个重要的主题。
// 一种可预见的方式是使用直接法/光流,可有效地避开繁重的特征计算工作。
void VisualOdometry::extractKeyPoints() // 对当前帧curr进行关键点的检测和描述子的计算
{
    boost::timer timer;
    // curr_指向当前帧frame的智能指针, color_是frame类中的存储彩色照片的Mat,
    // vector<cv::KeyPoint>    keypoints_curr_;是visual_odometry类中的成员
    // orb_是指针
    orb_->detect ( curr_->color_, keypoints_curr_ );
    cout<<"extract keypoints cost time: "<<timer.elapsed()*1000 << "ms" <<endl;
}

// 对当前帧计算BRIEF描述子
void VisualOdometry::computeDescriptors()
{
    boost::timer timer;
    // Mat descriptors_curr_; 当前帧BRIEF描述子,二进制描述子,描述向量由很多的0和1组成
    orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
    cout<<"descriptor computation cost time: "<<timer.elapsed()*1000 << "ms" <<endl;
}

// featureMatching特征匹配函数，先从地图中拿出一些候选点（出现在当前帧curr_视野内的点），
// 然后将他们与当前帧的特征描述子进行匹配。
void VisualOdometry::featureMatching()
{
    boost::timer timer;
    vector<cv::DMatch> matches;
    //1. 在地图map中选出视野内候选点candidates
    Mat desp_map; // 建立装描述子的目标地图，匹配需要的是描述子
        // candidate只是中间存储,临时变量,新帧会刷新。
    vector<MapPoint::Ptr> candidate; // 建立候选地图点数组
    // 检查地图点(路标点)是否是匹配需要的（地图点在当前帧可观察到isInFrame()函数）
    // 逻辑是遍历维护的局部地图中所有地图点，
    // 然后利用isInFrame()函数检查有哪些地图点在当前观察帧中，
    // 如果在则把地图点push进candidate中，描述子push进desp_map中
    // unordered_map<unsigned long, MapPoint::Ptr >  map_points_;
    for ( auto& allpoints: map_->map_points_ ) // &表示引用,遍历每一个无序map的键值对
    {
        // STL用法，allpoints为map类中定义的双模板类型类成员，此表示第二个模板类型,总之功能上就是把地图点取出，赋值给p
        // pair类型中的first和second成员
        // first成员是unsigned long id_, second成员是MapPoint::Ptr map_point
        MapPoint::Ptr& p = allpoints.second;
        // 检查世界坐标系下的3D坐标点(地图点/路标点)是否在当前帧curr_视野内
        if ( curr_->isInFrame(p->pos_) )
        {
            // add to candidate
            p->visible_times_++;//观察次数加一
            // 关键点push_back进candidate
            candidate.push_back( p );
            // 描述子push_back进desp_map
            desp_map.push_back( p->descriptor_ );
        }
    }
//    VO 0.3版本的使用的是暴力匹配,使用的是前一参考帧和当前帧进行匹配
//    cv::BFMatcher matcher ( cv::NORM_HAMMING );
//    matcher.match ( descriptors_ref_, descriptors_curr_, matches );

// 特征匹配代码。
// 匹配之前,我们从地图中拿出一些候选点(出现在视野内的点),
// 然后将它们与当前帧的特征描述子进行匹配。
    // 地图点3D（match_3dpts_）和帧中关键点2d（match_2dkp_index_）
    // 2. 候选点描述子地图desp_map与当前帧描述子descriptors_curr_进行匹配
    // matcher_flann_.match ( desp_map, descriptors_curr_, matches );
        matcher_brute_.match ( desp_map, descriptors_curr_, matches );
    // select the best matches
    // 3. 找最小距离
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    // vector<MapPoint::Ptr> match_3dpts_; 匹配的3D点坐标, MapPoint指向路标点的动态指针
    match_3dpts_.clear(); // clear()清空vector中的成员
    // vector<int> match_2dkp_index_; 匹配的2D像素点(当前帧的关键点keypoints_curr_的索引index)
    match_2dkp_index_.clear();
    for ( cv::DMatch& m : matches )// 遍历所有的匹配对matches
    {
        // 每个匹配对的距离 < max(最小距离*match_ratio(2.0) , 30.0)
        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            // 这里变化是不像之前直接将好的m  push进feature_matches_就完了。
            // 这里感觉像做一个记录，candidate中存的是观察到的地图点
            // 进一步，在candidate中选出好的匹配的点，push进match_3dpts_，
            // 这个match_3dpts_代表当前这一帧计算T时所利用到的所有好的地图点，放入其中。
            // 由此可见，candidate只是个中间存储，新的一帧过来会被刷新。
            // 同样match_2dkp_index_也是同样的道理，
            // 只不过是记录的当前帧detect出来的keypoint数组中的点的索引。
            match_3dpts_.push_back( candidate[m.queryIdx] );
            match_2dkp_index_.push_back( m.trainIdx );
        }
    }
    cout<<"good matches: "<<match_3dpts_.size() <<endl;
    cout<<"match cost time: "<<timer.elapsed()*1000 << "ms" << endl;
}


// 这里删除了setRef3DPoints()函数，用不到了


// 使用了 RANSAC 求出的 PnP 解。由于 RANSAC 只采用少数几个随机点来计算 PnP,
// 虽然能够确定 inlier,但该方法容易受噪声影响。
// 在 3D-2D点存在噪声的情形下,要用 RANSAC 的解作为初值,再用非线性优化求一个最优值。

// 本节的目标是估计位姿而非结构,我们以相机位姿 ξ 为优化变量,通过最小化重投影误差,来构建优化问题。
// 与之前一样,我们自定义一个 g2o 中的优化边。它只优化一个位姿,因此是一个一元边。
void VisualOdometry::poseEstimationPnP()
{
    boost::timer timer;
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d; // 出现在当前帧视野中的路标点的3D坐标
    vector<cv::Point2f> pts2d; // 当前帧的2D坐标

    // 遍历所有的匹配的2D点的序列号
    for ( int index:match_2dkp_index_ )
    {
        pts2d.push_back ( keypoints_curr_[index].pt );
    }
    // vector<MapPoint::Ptr> match_3dpts_;
    // 遍历所有的匹配的3D路标点mapPoint,
    for ( MapPoint::Ptr pt:match_3dpts_ )
    {
        // pt->getPositionCV()获取每个路标点的3D坐标
        pts3d.push_back( pt->getPositionCV() );
    }

    // K内参矩阵: fx   0  cx
    //           0  fy  cy
    //           0   0   1
    Mat K = ( cv::Mat_<double> ( 3,3 ) <<
              ref_->camera_->fx_, 0, ref_->camera_->cx_,
              0, ref_->camera_->fy_, ref_->camera_->cy_,
              0,0,1
            );

    //旋转向量(分别是绕xyz的弧度)、平移向量、内点数组
    Mat rvec, tvec, inliers;
    cv::solvePnPRansac ( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
    // 内点数为内点行数
    num_inliers_ = inliers.rows;
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    // T变成了Tcw而不是之前的Tcr，当前帧直接和地图点之间绝对变化位姿(世界坐标系下的当前帧的相机坐标)，而非与参考帧之间找相对位姿变化
    // 李群SE3
    T_c_w_estimated_ = SE3 (
                           SO3 ( rvec.at<double> ( 0,0 ), rvec.at<double> ( 1,0 ), rvec.at<double> ( 2,0 ) ),
                           Vector3d ( tvec.at<double> ( 0,0 ), tvec.at<double> ( 1,0 ), tvec.at<double> ( 2,0 ) )
                       );
    cout << "T_c_w before BA" << endl << T_c_w_estimated_ << endl;

    // 以RANSAC PnP结果为初值,再调用g2o进行优化位姿T，最小化重投影误差。
    // 位姿6维,观测点2维
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    // LinearSolverDense稠密Dense求解器
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    // Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
    Block* solver_ptr = new Block ( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // vertex添加顶点,即  估计第二个相机的位姿pose
    // VertexSE3Expmap李代数位姿
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId ( 0 ); // 设置ID
    pose->setEstimate ( g2o::SE3Quat (
        T_c_w_estimated_.rotation_matrix(), T_c_w_estimated_.translation()
    ));// 设置优化的初始值,即EPnP_Ransac求解出来的初始值
    optimizer.addVertex ( pose ); // 添加顶点,将待优化的位姿添加为顶点vertex

    // edges边,所有匹配的内点, 边是重投影误差
    for ( int i=0; i<inliers.rows; i++ )
    {
        int index = inliers.at<int> ( i,0 );
        // 3D -> 2D projection 3D -> 2D 投影
        EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setId ( i );
        edge->setVertex ( 0, pose );
        // 相机参数
        edge->camera_ = curr_->camera_.get();
        // 3D点
        edge->point_ = Vector3d ( pts3d[index].x, pts3d[index].y, pts3d[index].z );
        // 测量值是2维的
        edge->setMeasurement ( Vector2d ( pts2d[index].x, pts2d[index].y ) );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        // set the inlier map points 
        match_3dpts_[index]->matched_times_++;
    }

    // 执行优化
    optimizer.initializeOptimization();
    optimizer.optimize ( 10 );

    T_c_w_estimated_ = SE3 (
        pose->estimate().rotation(),
        pose->estimate().translation()
    );
    
    cout<<"T_c_w_estimated_: "<<endl<<T_c_w_estimated_.matrix()<<endl;
    cout<<"PnPRansac_Bundle cost time: "<<timer.elapsed()*1000 << "ms" <<endl;
}

// 由于各种原因,我们设计的上述 VO 算法,每一步都有可能失败。
// 例如图片中不易提特征、特征点缺少深度值、误匹配、运动估计出错等等。
// 因此,要设计一个鲁棒的 VO,必须(最好是显式地)考虑到上述所有可能出错的地方——
// 那自然会使程序变得非常复杂。
// 我们在 checkEstimatedPose 中,根据内点(inlier)的数量以及运动的大小做一个简单的检测:
// 认为内点不可太少,而运动不可能过大。
// 当然,读者也可以思考其他检测问题的手段,尝试一下效果。

// 位姿检验模块，匹配点不能太少，运动不能太大
bool VisualOdometry::checkEstimatedPose()
{
    // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    // ref_->T_c_w_参考帧相对于世界坐标系的位姿,
    // T_c_w_estimated_当前帧相对于世界坐标系的位姿,
    // T_c_w_estimated_.inverse()世界坐标系相对于当前帧的位姿
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse(); // T_r_c当前帧下的参考帧的位姿
    // 0.3使用的是SE3 T_c_r_estimated_;

    // 李群SE3-->.log()李代数se3, 一个6维向量,前三个为平移分量,后三个为旋转分量
    // typedef Matrix< double, 6, 1 > Vector6d;
    Sophus::Vector6d d = T_r_c.log();
    // x.norm() 二范数，欧氏距离，（注意：Eigen中没有norm(R)）
    // x.squaredNorm() 二范数的平方和，（注意：对于复数而言，不等价）
    if ( d.norm() > 5.0 )// 二范数>5,运动过大
    {
        cout<<"reject because motion is too large: "<<d.norm() <<endl;
        return false;
    }
    return true;
}
// 还引入了“关键帧”(Key-frame)的概念。
// 关键帧在许多视觉 SLAM 中都会用到,不过这个概念主要是给后端用的,
// 所以我们下几讲再讨论对关键帧的详细处理。

// 在实践中,肯定不希望对每个图像都做详细的优化和回环检测,那样太耗费资源。
// 至少相机搁在原地不动时,我们不希望整个模型(地图也好、轨迹也好)变得越来越大。
// 因此,后端优化的主要对象就是关键帧。
// 关键帧是相机运动过程当中某几个特殊的帧,这里“特殊”的意义是可以由我们自己指定的。
// 常见的做法时,每当相机运动经过一定间隔,就取一个新的关键帧并保存起来.
// 这些关键帧的位姿将被仔细优化,而位于两个关键帧之间的那些东西,
// 除了对地图贡献一些地图点外,就被理所当然地忽略掉了。
// 本节的实现也会提取一些关键帧,为后端的优化作一些数据上的准备。
bool VisualOdometry::checkKeyFrame()
{
    // 参考帧下的当前帧的位姿
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    // typedef Matrix< double, 6, 1 > Vector6d;
    // Eigen::Matrix<double, 6, 1> d = T_r_c.log();
    Sophus::Vector6d d = T_r_c.log(); // 6维向量
    Eigen::Vector3d trans = d.head<3>(); // 前3个变量是平移分量
    Eigen::Vector3d rot = d.tail<3>(); // 后3个变量是旋转分量
    // 平移分量的二范数>关键帧的最小旋转二范数,|| 或  旋转分量的二范数>关键帧的最小二范数
    if ( rot.norm() > key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;// 返回true表示是关键帧
    return false; // 返回false表示不是关键帧
}

// 1. 在提取第一帧的特征点之后,将第一帧的所有特征点全部放入地图中:
// addKeyFrame()函数增加关键帧函数多了一步
// 在第一帧时，将其对应的地图点mapPoint全部添加进地图中。
void VisualOdometry::addKeyFrame()
{

    // 1. 在第一帧时，将地图点全部添加进地图中
    // unordered_map<unsigned long, Frame::Ptr > keyframes_;
    if ( map_->keyframes_.empty() )// 地图中没有关键帧, 即在第一帧的时候
    {
        // first key-frame, add all 3d points into map
        // 遍历当前帧的所有关键点
        for ( size_t i=0; i<keypoints_curr_.size(); i++ )
        {
            // 对于当前帧,查找关键点i对应的深度值,这里返回的是d/depth_scale
            double d = curr_->findDepth ( keypoints_curr_[i] );
            if ( d < 0 ) // 剔除深度<0
                continue;
            // 将关键点的像素坐标-->世界坐标系下的3D坐标
            Vector3d p_world = ref_->camera_->pixel2world (
                Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), curr_->T_c_w_, d
                // 当前帧第i个关键点的像素坐标x,y     当前帧的世界坐标系-->相机坐标系的转换,   深度
            );
            // 世界坐标减去相机光心坐标，求得模长
            Vector3d n = p_world - ref_->getCamCenter();
            n.normalize(); // 将向量单位化
            // 构造一个地图点，3D点、模长、描述子、帧
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
            // 世界坐标系下的路标点坐标p_world XYZ, Vector3d& norm, descriptor匹配的描述子,
            // Frame* frame记录观察到此地图点(路标点)的帧的指针
            // Frame::Ptr  curr_;智能指针, curr_.get()返回curr_中保存的指针.
            // 要小心使用,若智能指针释放了其对象,返回的指针所指向的对象也就消失了
            );

            // 将第一帧中的所有地图点mapPoint添加进地图map中
            map_->insertMapPoint( map_point );
        }
    }
    // 添加关键帧到地图中(也将第一帧添加到关键帧中去)
    map_->insertKeyFrame ( curr_ );
    ref_ = curr_; // Frame::Ptr  ref_参考帧 = 当前帧
}

// 新增函数, 添加地图点(路标点), 局部地图类似于slidewindow一样，随时的增删地图中的点，来跟随运动
// 1. 匹配matches在match_2dkp_index_和地图匹配中索引为true
// 2. 添加地图点mappoint进入地图中
void VisualOdometry::addMapPoints()
{
    // add the new map points into map
    // bool类型数组matched,大小为当前帧keypoints关键点的数量,并将所有值初始化为false
    vector<bool> matched(keypoints_curr_.size(), false);

    //首先这个match_2dkp_index_是新来的当前帧跟地图匹配时，好的匹配到的关键点在keypoins数组中的索引
    //在这里将已经匹配的keypoint索引置为true
    // match_2dkp_index_是当前帧与地图匹配到的关键点在keypoints数组中索引
    // 设置匹配好的关键点索引为true.
    for ( int index:match_2dkp_index_ )
        matched[index] = true;
    // 和addKeyFrame中第一帧的时候添加地图点类似
    for ( int i=0; i<keypoints_curr_.size(); i++ )//遍历当前keypoints数组
    {
        //如果为true，说明在地图中找到了匹配，也就意味着地图中已经有这个点了。
        // 直接continue
        if ( matched[i] == true )// true表示在地图中有这个点匹配上
            continue;


        //如果没有continue的话，说明这个点在地图中没有找到匹配，认定为新的点，
        //下一步就是找到depth数据，构造3D点，然后构造地图点，添加进地图即可。

        // 找深度d,构造3D点，减去相机光心，构造地图点，添加进地图。
        double d = ref_->findDepth ( keypoints_curr_[i] );
        if ( d<0 )  
            continue;
        // 构造3D点
        Vector3d p_world = ref_->camera_->pixel2world (
            Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), 
            curr_->T_c_w_, d
        );
        // 减去相机光心
        Vector3d n = p_world - ref_->getCamCenter();
        n.normalize(); // 将向量单位化
        // 构造MapPoint
        MapPoint::Ptr map_point = MapPoint::createMapPoint(
            p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
        );
        // 向地图中添加MapPoint
        map_->insertMapPoint( map_point );
    }
}
// 新增函数，维护地图的规模。删除一些地图点，在点过少时增加地图点等操作。
// 后续的帧中,使用 OptimizeMap 函数对地图进行优化。
// 包括删除不在视野内的点,在匹配数量减少时添加新点等等
void VisualOdometry::optimizeMap()
{
    // remove the hardly seen and no visible points
    // 删除地图点
    // 使用迭代器,遍历所有的地图点(路标点)
    for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end();  )
    {
        // 四种情况,符合任何一个,就删除该路标点,并进行下一次循环,检查下一个路标点mapPoint
        // 1.1当前帧中看不见该点,说明跑的比较远了,删除改点
        if ( !curr_->isInFrame(iter->second->pos_) )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        // 1.2 匹配率=匹配次数/可见次数，过低删除, 默认的参数是0.1
        // 匹配率过低说明经常见但是没有几次匹配。
        // 应该是一些比较难识别的点，也就是出来的描述子比较奇葩。所以删掉
        float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
        if ( match_ratio < map_point_erase_ratio_ )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        // 1.3 角度大于pi/6,即30度
        double angle = getViewAngle( curr_, iter->second );
        if ( angle > M_PI/6. )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        // 1.4 不是好点,     可以根据一些其他条件自己添加要删除点的情况
        if ( iter->second->good_ == false )
        {
            // TODO try triangulate this map point
            // 可以使用三角化来更新特征点的世界坐标
        }
        iter++; // 迭代器++
    }
    // 2. 增加点
        // 2.1 当前帧与地图匹配点少于100个，
        // 一般是运动幅度过大了，跟之前的帧没多少交集了，所以增加
    if ( match_2dkp_index_.size()<100 )
        addMapPoints();
        // 2.2 路标点多于1000，增加释放率，让地图处于释放点的趋势,否则维持0.1
    if ( map_->map_points_.size() > 1000 )  
    {
        // TODO map is too large, remove some one
        // 考虑更好地动态管理地图规模的策略
        map_point_erase_ratio_ += 0.05;
    }
        //如果没有多于1000个，保持释放率在0.1，维持地图的体量为平衡态
    else 
        map_point_erase_ratio_ = 0.1;
    // cout<<"map points: "<<map_->map_points_.size()<<endl;
}
// 取得视角, 取得一个空间点在一个帧下的视角角度,返回的是double类型的角度值?????没懂
double VisualOdometry::getViewAngle ( Frame::Ptr frame, MapPoint::Ptr point )
{
    // 空间点坐标减去相机中心(光心)坐标，得到从相机中心(光心)指向空间点的向量
    Vector3d n = point->pos_ - frame->getCamCenter();
    // 单位化
    n.normalize();
    // 返回角度，acos()反余弦
    //返回一个角度，acos()为反余弦，
    // 向量*乘为：a*b=|a||b|cos<a,b>
    // 所以单位向量的*乘得到的是两个向量的余弦值，再用acos()即得到角度，返回
    // 物理意义就是得到匆匆世界坐标系下看空间点和从相机坐标系下看空间点，视角差的角度。
    return acos( n.transpose() * point->norm_ );
}


}