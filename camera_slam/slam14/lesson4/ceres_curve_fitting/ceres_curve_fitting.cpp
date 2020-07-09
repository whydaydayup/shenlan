#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

// 定义Cost Function模型
struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y){}
    // 定义一个带模板参数的()运算符，该类成为拟函数（Functor，C++术语）
    // 这种方式使得Ceres可以像调用函数一样，对该类的某个对象（比如a）调用a<double>()方法，使对象具有像函数一样的行为
    // 残差计算
    template <typename T>
    bool operator()(
            const T* const abc, // 模型参数，有3维
            T*residual) const // 残差
    {
        // y-exp(ax^2+bx+c)
        residual[0] = T( _y ) - ceres::exp( abc[0]*T(_x) *T(_x) + abc[1]*T(_x) + abc[2]);
        return true;
    }

    const double _x, _y;
};

int main( int argc, char* argv[] )
{
    double a=1.0, b=2.0, c=1.0; // 真实参数值
    int N=100; //数据点数量
    double w_sigma=1.0; //噪声Sigma值
    cv::RNG rng; //OpenCV 随机数产生器
    double abc[3] = {0, 0, 0}; // abc参数估计值

    vector<double> x_data, y_data; // 数据

    cout << "generating data: " << endl;
    for( int i = 0; i < N; i++ )
    {
        double x = i/100.0;
        x_data.push_back( x );
        y_data.push_back( exp( a*x*x + b*x + c ) + rng.gaussian( w_sigma ) );
        cout << x_data[i] << " " << y_data[i] << endl;
    }

    // 构建最小二乘问题
    ceres::Problem problem;
    for( int i=0; i<N; i++ )
    {
        // 调用AddResidualBlock将误差项添加到目标函数中。
        // 由于优化需要梯度，有若干种选择：
        // 1、使用Ceres自动求导（Auto Diff），编码最方便
        // 2、使用数值求导（Numeric Diff）
        // 3、自行推导解析的导数形式，提供给Ceres
        problem.AddResidualBlock(
        // Ceres提供基于模板元的自动求导和运行时的数值求导
        // g2o只提供运行时的数值求导
        // Ceres自动求导是通过模板元实现，在编译时期可以完成自动求导工作，不过仍然是数值求导
        // 使用自动求导， 模板参数：误差类型，输出维度，输入维度，数值参照前面的struct中写法
        // 误差项，优化变量的维度，误差是标量，维度为1，优化abc三个量，维度为3
        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                new CURVE_FITTING_COST(x_data[i], y_data[i])
                ),
                nullptr,   //核函数，这里不使用，为空
                abc // 待估计参数
             );
    }
    // 配置求解器，可以选择Line Search还是Trust Region，迭代次数，步长等
    ceres::Solver::Options options; //这里有很多配置项可以填写
    options.linear_solver_type = ceres::DENSE_QR; // 增量方程如何求解
    options.minimizer_progress_to_stdout = true; //输出到cout

    ceres::Solver::Summary summary; // 优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve( options, &problem, &summary ); //开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2 - t1 );
    cout << "solve time cost = " << time_used.count() << " seconds." << endl;

    //输出结果
    cout << summary.BriefReport() << endl;
    cout << "estimated a,b,c = ";
    for( auto a: abc ) cout << a << " " ;
    cout << endl;


}