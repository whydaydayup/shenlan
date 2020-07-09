#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>// 李代数表达的节点和边
#include <chrono>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <boost/timer.hpp>

using namespace std;
using namespace cv;

void find_feature_matches (
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches );

// 像素坐标转相机归一化坐标
Point2d pixel2cam ( const Point2d& p, const Mat& K );

void bundleAdjustment (
    const vector<Point3f> points_3d,
    const vector<Point2f> points_2d,
    const Mat& K,
    Mat& R, Mat& t,
    Mat& R_, Mat& t_
);

int main ( int argc, char** argv )
{
    //-- 读取图像
    Mat img_1 = imread ( "../1.png", CV_LOAD_IMAGE_COLOR ); // CV_LOAD_IMAGE_COLOR总是读取三通道的图像，默认值
    Mat img_2 = imread ( "../24.png", CV_LOAD_IMAGE_COLOR );

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"一共找到了" << matches.size() <<"组匹配点"<<endl;

    // 建立3D点
    // 深度图为16位无符号数，单通道图像，CV_LOAD_IMAGE_UNCHANGED通道数由文件决定，允许加载超过８bit的深度
    Mat d1 = imread ( "../1_depth.png", CV_LOAD_IMAGE_UNCHANGED );
    // Mat_是opencv中的inline矩阵，
    // fx,  0, cx
    //  0, fy, cy
    //  0,  0,  1
    Mat K = ( Mat_<double> ( 3,3 ) << 517.3, 0, 318.6, 0, 516.5, 255.3, 0, 0, 1 );

    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    for ( DMatch m:matches )
    {
        // m.queryIdx匹配的图１的描述子的在keypoints_1中的序号，　pt表示的是keypoint的坐标y,x
        // 通过在深度图中找到匹配点x,y相对应的深度值
        ushort d = d1.ptr<unsigned short> (int ( keypoints_1[m.queryIdx].pt.y )) [ int ( keypoints_1[m.queryIdx].pt.x ) ];
        if ( d == 0 )   // bad depth，深度值为0
            continue;
        float dd = d/5000.0; // 深度相机的scale为5000（和kinect默认的1000是不同的）。也就是depth/中图像像素值5000为真实世界中的一米。
        // p1 =  (x - cx)/fx, (y-cy)/fy
        Point2d p1 = pixel2cam ( keypoints_1[m.queryIdx].pt, K );
        // pts_3d = x, y, depth真实的keypoints_1中的x,y,depth
        pts_3d.push_back ( Point3f ( p1.x*dd, p1.y*dd, dd ) );
        // pts_2d = x, y是相机的keypoints_2中的x,y
        pts_2d.push_back ( keypoints_2[m.trainIdx].pt );
    }

    cout<<"3d-2d pairs: "<<pts_3d.size() <<endl;


    chrono::steady_clock::time_point s1 = chrono::steady_clock::now();

    Mat r, t, inliers;
    // solvePnPRansac();// 将外点（错误的匹配点）去掉，使用正确的点将Ｒ,ｔ计算出来
    solvePnPRansac ( pts_3d, pts_2d, K, Mat(), r, t, false, 100, 4.0, 0.99, inliers);// 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    int num_inliers_ = inliers.rows;  // number of inlier features in pnp
    cout<<"pnp inliers: "<<num_inliers_<<endl;

    chrono::steady_clock::time_point s2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( s2-s1 );
    cout<<"EPNP_ITERATIVE costs time: "<<time_used.count()*1000 <<" ms."<<endl;

    Mat R;
    cv::Rodrigues ( r, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵
    cout << R << endl;

    cout<<"calling bundle adjustment"<<endl;

    // ground_truth
    Mat truth_t1 = ( Mat_<double> ( 3,1 ) <<  1.3405, 0.6266, 1.6575 );
    Mat truth_t2;
//    truth_t2 = ( Mat_<double> ( 3,1 ) <<  1.3303, 0.6256, 1.6464  );//0.0151079
//    truth_t2 = ( Mat_<double> ( 3,1 ) <<  1.2757, 0.6256, 1.5853  );//0.09702m
//    truth_t2 = ( Mat_<double> ( 3,1 ) <<  1.2089, 0.6245, 1.5116  );//0.196494
//    truth_t2 = ( Mat_<double> ( 3,1 ) <<  1.1501, 0.6226, 1.4343  );//0.293405
    truth_t2 = ( Mat_<double> ( 3,1 ) <<  1.0733, 0.6329, 1.3610  );//0.399184


    // 根据一组3D点和第二个图像中的2D投影,估计第二个相机的位姿
    Mat truth_t =  truth_t2 - truth_t1; // 由第一个点(3D点)到第二个点(2D相机平面)的水平位移

    Mat R_, t_;


    s1 = chrono::steady_clock::now();
    bundleAdjustment ( pts_3d, pts_2d, K, R, t, R_, t_ );
    s2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>> ( s2-s1 );
    cout<<"RANSAC_EPNP_Bundle costs time: "<<time_used.count()*1000 <<" ms."<<endl;

    Mat t1 = R  * truth_t1 + t;
    Mat t2 = R_ * truth_t1 + t_;

    // mat.t()矩阵的转置
    double truth_l = norm(truth_t, NORM_L2);
    cout << "truth:             " << truth_t.t() << " " << norm(truth_t, NORM_L2) << endl;
    cout << "RANSAC_EPNP:       " << "t.t()  << " " << norm(t, NORM_L2)  << " " << norm(truth_t, t , NORM_L2)/truth_l*100 <<  " << abs(norm(t , NORM_L2)-truth_l) << " " << abs(norm(t , NORM_L2)-truth_l)/truth_l*100 << endl;
    cout << "RANSAC_EPNP_Bundle:" << "t_.t() << " " << norm(t_, NORM_L2) << " " << norm(truth_t, t_, NORM_L2)/truth_l*100 <<  " << abs(norm(t_, NORM_L2)-truth_l) << " " << abs(norm(t_ , NORM_L2)-truth_l)/truth_l*100 << endl;
    cout << "truth:             " << truth_t2.t() << endl;
    cout << "RANSAC_EPNP:       " << "t1.t()" << " " << norm(truth_t2, t1, NORM_L2)/norm(truth_t1, NORM_L2)*100 << " " << abs(norm(t1 , NORM_L2)-truth_l)/truth_l*100 << endl;
    cout << "RANSAC_EPNP_Bundle:" << "t2.t()" << " " << norm(truth_t2, t2, NORM_L2)/norm(truth_t1, NORM_L2)*100 << " " << abs(norm(t2 , NORM_L2)-truth_l)/truth_l*100 << endl;
}

void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3

    Ptr<FeatureDetector> detector = ORB::create(500, 1.2f, 4);
    Ptr<DescriptorExtractor> descriptor = ORB::create(500, 1.2f, 4);

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    cv::FlannBasedMatcher matcher( new cv::flann::LshIndexParams ( 5,10,2 ) );
    matcher.match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    //printf ( "-- Max dist : %f \n", max_dist );
    //printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}

Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
                   // (x - cx)/fx
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
                       // (y - cy)/fy
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}

void bundleAdjustment (
    const vector< Point3f > points_3d, // 第一帧的3D点
    const vector< Point2f > points_2d, // 第二帧的2D点
    const Mat& K,// 相机内参矩阵
    Mat& R, Mat& t, // 待优化的旋转矩阵R和平移矩阵t
    Mat& R_, Mat& t_)
{
    // 初始化g2o,
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose位姿维度为 6, landmark路标维度为 3
    // 线性方程求解器LinearSolverCSparse
    // g2o允许使用不同的优化求解器（然而实际效果似乎差别不大）。可选csparse, pcg, cholmod等等。这里使用csparse为例啦。
    //Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton ( solver_ptr );
    g2o::SparseOptimizer optimizer; //图模式
    optimizer.setAlgorithm ( solver ); // 设置求解器

    // vertex添加顶点,即  估计第二个相机的位姿pose
    // VertexSE3Expmap李代数位姿
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat << // 旋转矩阵
               R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
               R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
               R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );

    // sets the id of the node in the graph be sure that the graph keeps consistent after changing the id
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat ( // 构建位姿:旋转矩阵+平移矩阵
                            R_mat,// 旋转矩阵
                            // 平移矩阵
                            Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
                        ) );// 这是设置初始值,若将其设置为旋转部分为单位阵,平移部分为0矩阵g2o::SE3Quat(),则可以看出优化前后较大的差别
    optimizer.addVertex ( pose );

    int index = 1;
    for ( const Point3f p:points_3d )   // landmarks一组3D点,第一帧的3D点坐标XYZ
    {
        // VertexSBAPointXYZ空间点位置
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId ( index++ );
        // 3D点的x,y,z坐标
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) ); // 设置初始值,第一帧3D点的XYZ坐标值
        point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
        // 上面这句,若只是优化pose,用的点是对的,应该加上point->setFixed(true);
        optimizer.addVertex ( point ); // 添加顶点
    }

    // parameter: camera intrinsics相机内参
    // CameraParameters(double focal_length, const Vector2D & principle_point, double baseline)
    g2o::CameraParameters* camera = new g2o::CameraParameters (
        K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
    );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // edges边
    index = 1;
    for ( const Point2f p:points_2d )
    {
        // 每个pose和point之间构建edges边,纯投影误差
        // g2o::EdgeProjectXYZ2UV投影方程边
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );// 每条边的ID
        // 设置连接的顶点
        // 边的一边连接着3D point,即optimizer中的第index个vertex
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose ); // 边的一边连接着pose位姿
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );// 观测数值
        edge->setParameterId ( 0,0 );//???
        edge->setInformation ( Eigen::Matrix2d::Identity() );// 信息矩阵：协方差矩阵之逆
        optimizer.addEdge ( edge ); // 添加边
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    optimizer.setVerbose ( false ); // true打开调试输出
    optimizer.initializeOptimization();// 初始化图优化
    optimizer.optimize ( 100 );// 开始进行图优化求解,10表示最大的迭代次数

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;

    Eigen::Matrix<double, 4, 4> T = Eigen::Isometry3d ( pose->estimate() ).matrix();

    R_ = ( Mat_<double> ( 3,3 ) <<  T(0,0), T(0,1), T(0,2),
                                    T(1,0), T(1,1), T(1,2),
                                    T(2,0), T(2,1), T(2,2) );

    t_ = ( Mat_<double> ( 3,1 ) <<  T(0,3), T(1,3), T(2,3) );

}
