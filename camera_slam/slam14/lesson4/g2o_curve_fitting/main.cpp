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

// 用顶点来描述待优化变量
// 曲线模型的顶点， 模板参数：优化变量维度和数据类型
class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 顶点重置函数，把估计值置为零
    virtual void setToOriginImpl(){ //重置 optimizable_graph.h line333
        _estimate << 0,0,0;
    }

    // 顶点更新函数，xk+1= xk + dx
    // 优化变量（曲线参数）本身在向量空间中，更新计算就是简单的加法
    // 优化变量不在向量空间中，x为相机位姿，本身不一定有加法运算，
    // 需要重新定义增量如何加到现有的估计上，需要使用左乘或右乘更新，而不是直接的加法
    virtual void oplusImpl( const double* update )//更新
    {
        _estimate += Eigen::Vector3d(update);
    }
    // 存盘和读盘，不进行读写操作，留空
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {}
};

// 一个优化问题可以由很多顶点和很多边，用图优化的问题解决
// 一个机器的位置是顶点、一个路标是顶点，机器人看到路边就构成一条边，边会产生误差，估计出来数据就会跟实际不一样
// 一个优化问题由很多个误差项组成，用边来表示误差项
// 一个误差项可能关联几个变量（1、2、多个），边Edge可以连接1、2、多个顶点
// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge: public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
// 这个问题只估计一个顶点，所有边都是一条边，Unary Edge一元边，只关联一个顶点a,b,c
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge( double x ): BaseUnaryEdge(), _x(x) {}
    // 计算曲线模型误差
    void computeError()
    {
        // 1、取出边所连接的顶点的当前估计值
        const CurveFittingVertex* v = static_cast<const CurveFittingVertex*>( _vertices[0] );
        const Eigen::Vector3d abc = v->estimate();
        // 2、根据曲线模型，与它的观测值进行比较。与最小二乘模型中的误差模型一致
        // _error误差值 = _measurement测量值 - 理论模型的值
        _error(0, 0) = _measurement - std::exp(abc(0, 0)*_x*_x + abc(1, 0)*_x + abc(2, 0));
    }
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {} // const !!!!一定不能忘记
public:
    double _x; // x值，  y值为 _measurement
};
// Ceres提供基于模板元的自动求导和运行时的数值求导
// g2o只提供运行时的数值求导
// 本身g2o提供自动求导功能，可以不提供导数实现
// 还可以提供导数的实现,提供简洁导数实现可以稍微加速计算
int main( int argc, char** argv )
{
    double a = 1.0, b = 2.0, c = 1.0;// 真实参数值
    int N = 100; // 数据点个数
    double w_sigma = 1.0; // 噪声sigma值
    cv::RNG rng; // OpenCV随机数产生器
    double abc[3] = {0, 0, 0}; // abc参数估计值

    vector<double> x_data, y_data; // 数据

    cout << "generating data: " << endl;
    for( int i= 0; i<N; i++ )
    {
        double x = i/100.0;
        x_data.push_back(x);
        y_data.push_back( exp(a*x*x + b*x + c) + rng.gaussian(w_sigma) );
        cout << x_data[i] << " " << y_data[i] << endl;
    }
    // 构建图优化，先设定g2o
    // 矩阵块：每个误差项优化变量维度为3，误差值维度为1
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > Block;
    // Optimization Algorithm 拥有一个Solver，
    // 它含有两个部分。一个是 SparseBlockMatrix ，用于计算稀疏的雅可比和海塞； 一个是用于计算迭代过程中最关键的一步
    // 1、选择一个线性方程求解器，从 PCG, CSparse, Choldmod中选，实际则来自 g2o/solvers 文件夹中定义的东东
    // 用于计算迭代过程中最关键的一步 H * dx = -b 线性方程
    // 线性方程求解器：稠密的增量方程
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();// 稠密矩阵求解器，ceres中使用QR分解
    // 2、选择一个 BlockSolver, 用于计算稀疏块矩阵SparseBlockMatrix稀疏的雅可比和海塞
    Block* solver_ptr = new Block(linearSolver); //矩阵块求解器,ceres中使用QR分解
    // 3、选择一个迭代策略，从GN, LM, Doglog中选。
    // SparseOptimizer 拥有一个 Optimization Algorithm，
    // 继承自Gauss-Newton, Levernberg-Marquardt, Powell's dogleg 三者之一（常用GN或LM）。
    // 梯度下降法，从GN高斯牛顿、LM、DogLeg中选
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg(solver_ptr);
    g2o::SparseOptimizer optimizer; // 图模式
    optimizer.setAlgorithm( solver ); // 设置求解器
    optimizer.setVerbose( true ); // 打开调试输出


    // 一个 SparseOptimizer 含有很多个顶点（继承自 Base Vertex）,用 SparseOptimizer.addVertex 向一个图中添加顶点
    // 向图中增加顶点
    CurveFittingVertex* v = new CurveFittingVertex();
    v->setEstimate( Eigen::Vector3d(0, 0, 0) );
    v->setId(0);
    optimizer.addVertex( v );

    // 一个 SparseOptimizer 含有很多个边（继承自 BaseUnaryEdge, BaseBinaryEdge或BaseMultiEdge），用SparseOptimizer.addEdge 向一个图中添加边
    // 向图中增加边
    for (int i=0; i<N; i++)
    {
        CurveFittingEdge* edge = new CurveFittingEdge( x_data[i] );
        edge->setId(i);
        edge->setVertex( 0, v ); // 设置连接的顶点
        edge->setMeasurement( y_data[i] ); // 观测数值
        // 信息矩阵：协方差矩阵之逆
        edge->setInformation( Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma) ); 
        optimizer.addEdge( edge ); 
    }
    // 这些 Base Vertex 和 Base Edge 都是抽象的基类，而实际用的顶点和边，都是它们的派生类。

    // 执行优化
    cout << "start optimization" << endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    // 最后调用 SparseOptimizer.optimize 完成优化
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast< chrono::duration<double> > (t2-t1) ;
    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    // 输出优化值
    Eigen::Vector3d abc_estimate = v->estimate();
    cout << "estimated model: " << abc_estimate.transpose() << endl;

    return 0;
}