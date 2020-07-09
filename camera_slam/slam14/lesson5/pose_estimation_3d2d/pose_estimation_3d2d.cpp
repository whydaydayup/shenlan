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
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>
#include <g2o/core/optimization_algorithm_gauss_newton.h>


using namespace std;
using namespace cv;
//特征点匹配，传入两张图像，两张图像对应的特征点，最后生成的匹配存入matches数组
void find_feature_matches (
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches );

// 像素坐标转相机归一化坐标
Point2d pixel2cam ( const Point2d& p, const Mat& K );
//BA优化
void bundleAdjustment (
    const vector<Point3f> points_3d,
    const vector<Point2f> points_2d,
    const Mat& K,
    Mat& R, Mat& t
);

int main ( int argc, char** argv )
{
    if ( argc != 5 )
    {
        cout<<"usage: pose_estimation_3d2d img1 img2 depth1 depth2"<<endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    // 得到匹配点后，在第一个图的深度图中寻找他们的深度，并求出空间位置
    // 以此空间位置为3D点，再以第二张图的像素位置为2D点，调用EPnP求解PnP问题
    // 建立3D点
    Mat d1 = imread ( argv[3], CV_LOAD_IMAGE_UNCHANGED );       // 深度图为16位无符号数，单通道图像
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );// 相机内参
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    for ( DMatch m:matches )
    {
        // OpenCV 的 DMatch 结构,queryIdx 为第一图的特征 ID,trainIdx 为第二个图的特征 ID。
        //这一步应该是取得匹配点对应的第一幅图的深度，queryIdx查询描述子索引，pt关键点的坐标
        ushort d = d1.ptr<unsigned short> (int ( keypoints_1[m.queryIdx].pt.y )) [ int ( keypoints_1[m.queryIdx].pt.x ) ];
        if ( d == 0 )   // bad depth
            continue;
        // 正确的depth scale是5000.0而不是书中的1000.0
        float dd = d/5000.0;// 深度图depth/尺度
        // 像素坐标u,v转相机归一化坐标X/Z, Y/Z, Z
        Point2d p1 = pixel2cam ( keypoints_1[m.queryIdx].pt, K );
        // 相机归一化坐标X/Z, Y/Z, Z 转换成 3D坐标 X, Y, Z
        pts_3d.push_back ( Point3f ( p1.x*dd, p1.y*dd, dd ) );
        // 取得第二幅图的 像素坐标u, v，trainIdx 为第二个图的特征 ID，pt关键点的坐标
        pts_2d.push_back ( keypoints_2[m.trainIdx].pt );
    }

    cout<<"3d-2d pairs: "<<pts_3d.size() <<endl;

    Mat r, t;
    solvePnP ( pts_3d, pts_2d, K, Mat(), r, t, false ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    // solvePnPRansac();// 将外点（错误的匹配点）去掉，使用正确的点将Ｒ,ｔ计算出来
    Mat R;
    cv::Rodrigues ( r, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵

    // 与之前2D-2D情况下求解的R,t, R几乎相同，t相差很多，这是由于引入了新的深度信息所致
    // 由于 Kinect 所采的深度图本身有一些误差，所有这里的3D点也不是很准确，希望将位姿R,t和所有三维点P同时优化
    cout<<"R="<<endl<<R<<endl;
    cout<<"t="<<endl<<t<<endl;

    cout<<"calling bundle adjustment"<<endl;

    bundleAdjustment ( pts_3d, pts_2d, K, R, t );
}

void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

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
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}

void bundleAdjustment (
    const vector< Point3f > points_3d,
    const vector< Point2f > points_2d,
    const Mat& K,
    Mat& R, Mat& t )
{
    // ui - 1/si * K * exp(Xi^) * Pi
    // ui像素坐标(观测到的投影位置)，si尺度，K相机内参，exp(Xi^)相机位姿Rt的李代数，Pi某空间坐标
    // 像素坐标（观测到的投影位置） 和 3D点按照当前估计的位姿投影得到的位置相比较得到的误差-->重投影误差
    // 初始化g2o
    // 构建图优化，先设定g2o
    // 矩阵块：每个误差项优化变量相机位姿pose(用李代数表示)维度为3，误差值(空间3D点landmark)维度为3
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    // Optimization Algorithm拥有一个Solver
    // 包含两部分，一个是 SparseBlockMatrix,用于计算稀疏矩阵的雅克比和海塞；一个用来计算迭代中最关键的一步
    // 1、选择一个线性方程求解器，从 PCG, CSparse, Choldmod中选，实际则来自 g2o/solvers 文件夹中定义的东东
    // 用于计算迭代过程中最关键的一步 H * dx = -b 线性方程
    // 线性方程求解器：稠密的增量方程
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
    // 2、选择一个 BlockSolver，用于计算稀疏矩阵块的雅克比和海塞；
    Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
    // 3、选择一个迭代策略，从GN, LM, Doglog中选。
    // SparseOptimizer 拥有一个 Optimization Algorithm，
    // 继承自Gauss-Newton, Levernberg-Marquardt, Powell's dogleg 三者之一（常用GN或LM）。
    // 梯度下降法，从GN高斯牛顿、LM、DogLeg中选
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton ( solver_ptr );
    g2o::SparseOptimizer optimizer; // 图模式
    optimizer.setAlgorithm ( solver ); // 设置求解器

    // 一个 SparseOptimizer 含有很多个顶点（继承自 Base Vertex）,用 SparseOptimizer.addVertex 向一个图中添加顶点
    // 向图中增加顶点vertex，这里的顶点有一个相机位姿和许多路标
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose 相机位姿SE3
    Eigen::Matrix3d R_mat;
    R_mat << // R为PnP求解出的旋转矩阵
          R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
          R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
          R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
    //添加一个相机位姿camera pose。给这个pose设置Id为0：pose->setId(0);
    pose->setId ( 0 );
    // 设置顶点的初始估计值，PnP求解出来的相机的位姿（旋转矩阵R,平移向量t）
    pose->setEstimate ( g2o::SE3Quat (
                            R_mat,
                            // t为PnP求解出来的平移向量
                            Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
                        ) );
    // 向求解器中添加一个顶点vertex，即一个相机位姿pose
    optimizer.addVertex ( pose );

    //添加许多路标点landmarks,每个路标点设置一个ID。
    int index = 1;
    // points_3d匹配点在第一个图的深度图中的深度，得到匹配点的空间位置
    // 遍历第一张图求解出来的3D空间点
    for ( const Point3f p:points_3d )   // landmarks 3D空间点XYZ
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        //添加许多路标点landmarks,每个路标点设置一个ID。
        point->setId ( index++ );
        // 设置顶点初始估计值，深度图直接得到的匹配点的空间位置，即观测得到的空间点坐标XYZ
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        // 如果只是优化pose，用的点是对的，加上了landmark point以后需要set Fixed(true)!!
        point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
        // 向求解器添加顶点vertex，即观测得到的空间点landmark坐标XYZ
        optimizer.addVertex ( point );
    }

    // K = fx,  0,  Cx,
    //      0, fy,  Cy,
    //      0,  0,   1,
    // 添加相机的内参parameter: camera intrinsics 相机的内参
    g2o::CameraParameters* camera = new g2o::CameraParameters (
        // fx: focal_length焦距  (Cx, Cy):principle_point主点, 0:baseline双目基线
        K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
    );
    camera->setId ( 0 );
    // 向求解器添加相机的内参
    optimizer.addParameter ( camera );

    // 一个 SparseOptimizer 含有很多个边（继承自 BaseUnaryEdge, BaseBinaryEdge或BaseMultiEdge），用SparseOptimizer.addEdge 向一个图中添加边
    // 向图中增加加许多边edges
    index = 1;
    // 以第二张图的像素位置为2D点，
    for ( const Point2f p:points_2d )
    {
        // 每个pose和point之间构建一个edges，重投影误差
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        // 设置连接的顶点 set the ith vertex on the hyper-edge to the pointer supplied
        // void setVertex(size_t i, Vertex* v) { assert(i < _vertices.size() && "index out of bounds"); _vertices[i]=v;}
        // 0--> index=1...dynamic --> points_3d第一张图求解出来的3D空间点
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        // pose --> SE3(R,t)相机的位姿
        edge->setVertex ( 1, pose );
        // // 观测数值, 与实际测量值之间的误差
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
        edge->setParameterId ( 0,0 );
        // edge->computeError(); // cout << edge->chi2() << endl: ????
        // 信息矩阵：协方差矩阵之逆
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose ( true ); // 打开调试输出，调试信息输出
    optimizer.initializeOptimization();
    // 最后调用 SparseOptimizer.optimize 完成优化
    optimizer.optimize ( 100 );
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;
}
