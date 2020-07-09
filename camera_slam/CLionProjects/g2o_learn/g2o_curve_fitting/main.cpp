#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>
using namespace std; 

// 曲线模型的顶点(待优化的量)，模板参数：优化变量维度(3)和数据类型(Eigen::Vector3d)
// 定义优化的变量,即a,b,c三个变量放在一个向量中
class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 虚函数,顶点重置函数,把估计值置0
    virtual void setToOriginImpl() // 重置
    {
        _estimate << 0,0,0;// 估计出的量,先设为0,0,0
    }
    // 顶点更新函数,优化中最重要的是增量delta_x的计算,该函数处理的是x_k+1 = x_k + delta_x的过程
    // 在曲线拟合中,由于优化变量(曲线参数)本身位于向量空间中,这个更新计算就是简单的加法,
    // 但当优化变量不存在于向量空间中的时,例如x是相机的位姿,本身不一定有加法运算,这时需要重新定义增量如何加到现有的估计上的行为,可以使用左乘或右乘更新,而不是直接的加法
    virtual void oplusImpl( const double* update ) // 虚函数,顶点的更新函数
    {
        _estimate += Eigen::Vector3d(update);
    }
    // 存盘和读盘函数read,write, 因为不想进行读写操作,所以留空
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {}
};

// 误差模型 模板参数：观测值维度(1)，类型(double)，连接顶点类型(CurveFittingVertex)
// Edge边, UnaryEdge一元边(这里是一元边),误差项,在这个问题只估计一个顶点,所有边是一条边,Unary Edge只关联一个顶点a,b,c
// Edge可能连接1/2/多个顶点,一个优化问题可以由很多个误差项组成,一个误差项可能关联几个变量
// 一个优化问题可以由很多个顶点和很多个边组成,用图优化/图论解决,
// 一个机器位置是一个顶点,一个路标是顶点,机器人看到路标构成一条边,边会产生误差,估计出来数据跟实际不一样
class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge( double x ): BaseUnaryEdge(), _x(x) {}
    // 计算曲线模型误差
    // 边的误差计算函数,computeError()取出边所连接的顶点的当前估计值,根据曲线模型,与它的观测值进行比较,这个最小二乘问题中的误差模型是一致的
    void computeError()
    {
        const CurveFittingVertex* v = static_cast<const CurveFittingVertex*> (_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        // 测量值y-理论值y
        _error(0,0) = _measurement - std::exp( abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0) ) ;
    }
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {}
public:
    double _x;  // x 值， y 值为 _measurement
};
// 还可提供导数的实现,本身g2o提供自动求导,也可以不提供导数实现,若提供简洁导数实现,可稍微加速计算
// g2o只提供数值求导,ceres提供基于模板元的自动求导和运行时的数值求导
// 对于大多数问题,若能够推导出雅克比矩阵的解析形式并告诉优化库,就可以避免数值求导中的诸多问题

int main( int argc, char** argv )
{
    double a=1.0, b=2.0, c=1.0;         // 真实参数值  exp(a*x^2+b*x+c)+rng
    int N=100;                          // 数据点个数,100个数据点
    double w_sigma=1.0;                 // 噪声Sigma值
    cv::RNG rng;                        // OpenCV随机数产生器
    double abc[3] = {0,0,0};            // abc参数的估计值

    vector<double> x_data, y_data;      // 数据
    
    cout<<"generating data: "<<endl;
    for ( int i=0; i<N; i++ )
    {
        double x = i/100.0;
        x_data.push_back ( x );
        y_data.push_back (
            exp ( a*x*x + b*x + c ) + rng.gaussian ( w_sigma )
        );
        cout<<x_data[i]<<" "<<y_data[i]<<endl;
    }

    // 曲线拟合问题-->图优化问题, 节点(待观测的参数)为优化变量,边(观测数据)为误差项
    // 曲线拟合问题中,只有一个顶点:曲线模型的参数a,b,c;  带噪声的数据点,构成一个个误差项,即图优化的边
    // 这里的边是一元边Unary Edge,即只连接一个顶点--因为整个图优化中只有一个顶点
    // 图优化中一条边可连接一个,两个或多个顶点,主要反映每个误差与多少个优化变量有关

    // 构建图优化，先设定g2o
    // 矩阵块: 每个误差项优化变量维度为3，误差值维度为1
    // Traints英文是特性,
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > Block;
    // 线性方程求解器.稠密的增量方程, SolverDense稠密矩阵求解器
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
    // 梯度下降方法，从GN, LM, DogLeg 中选
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm( solver );   // 设置求解器
    optimizer.setVerbose( true );       // true打开调试输出, false关闭调试输出
    
    // 往图中增加顶点
    CurveFittingVertex* v = new CurveFittingVertex();
    v->setEstimate( Eigen::Vector3d(0,0,0) );
    v->setId(0);
    optimizer.addVertex( v );
    
    // 往图中增加边
    for ( int i=0; i<N; i++ )
    {
        CurveFittingEdge* edge = new CurveFittingEdge( x_data[i] );
        edge->setId(i);
        edge->setVertex( 0, v );                // 设置连接的顶点
        edge->setMeasurement( y_data[i] );      // 观测数值
        edge->setInformation( Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma) ); // 信息矩阵：协方差矩阵之逆
        optimizer.addEdge( edge );
    }
    
    // 执行优化
    cout<<"start optimization"<<endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    optimizer.initializeOptimization(); // 初始化图优化
    optimizer.optimize(100); // 开始进行图优化求解,100表示最大的迭代次数

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;
    
    // 输出优化值
    Eigen::Vector3d abc_estimate = v->estimate();
    cout << "estimated model: "<< abc_estimate.transpose() <<endl;
    
    return 0;
}