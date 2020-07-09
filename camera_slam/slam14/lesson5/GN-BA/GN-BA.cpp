//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

// std::vector  Eigen::Vector3d
typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "./p3d.txt";
string p2d_file = "./p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    // K = fx,  0,  Cx,
    //      0, fy,  Cy,
    //      0,  0,   1,
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    ifstream ifp3d, ifp2d;

    ifp3d.open( p3d_file );
    // ifp3d.open( p3d_file, ios::in );
    if ( !ifp3d.is_open() )
    // if ( !ifp3d )
    {
        cout << "Open p3d.txt fail!" << endl;
        return 1;
    }
    else
    {
        // another method
       /* string sP3d;
        // getline默认的换行分隔符号就是'\n',可以不写
        while( getline(ifp3d, sP3d) && !sP3d.empty() )
        {
            istringstream issP3d(sP3d);
            Vector3d vP3d; // Eigen::Vector3d
            issP3d >> vP3d[0] >> vP3d[1] >> vP3d[2];
            p3d.push_back( vP3d ); // std::vector
        }*/

        while ( !ifp3d.eof() )
        {
            string line;
            getline( ifp3d, line, '\n' );
            istringstream line_string(line);
            double tmp1, tmp2, tmp3;
            line_string >> tmp1 >> tmp2 >> tmp3;
            Vector3d p3d_tmp{tmp1, tmp2, tmp3}; // Eigen::Vector3d
            p3d.push_back( p3d_tmp ); // std::vector
        }
    }

    ifp2d.open( p2d_file );
    if ( !ifp2d.is_open() )
    {
        cout << "Open p2d.txt fail!" << endl;
        return 1;
    }
    else
    {
        while ( !ifp2d.eof() )
        {
            double tmp1, tmp2;
            string line;
            getline( ifp2d, line, '\n' );
            istringstream line_string(line);
            line_string >> tmp1 >> tmp2;
            Vector2d p2d_tmp{ tmp1, tmp2 }; // Eigen::Vector2d
            p2d.push_back( p2d_tmp ); // std::vector
        }
    }
    // END YOUR CODE HERE

    // assert宏的原型定义在<assert.h>中
    // #include <assert.h>
    // void assert( int expression );
    // assert的作用是现计算表达式 expression ，如果其值为假（即为0），
    // 那么它先向stderr打印一条出错信息, 然后通过调用 abort 来终止程序运行。
    assert(p3d.size() == p2d.size());

    int iterations = 100; // 迭代次数
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    // 初始化估计为T0 = I 单位矩阵
    // T 4x4 = R 3x3, t 3x1
    //         0 1x3, 1 1x1
    Eigen::Vector3d t_I{0, 0, 0};
    Eigen::Matrix3d R_I = Matrix3d::Identity();
    Sophus::SE3 T_esti(R_I, t_I); // estimated pose
    // Sophus::SE3 T_esti( Matrix3d::Identity(), Vector3d::Zero() ); // 初始化估计为T0 = I 单位矩阵
    // Sophus::SE3 T_esti; // 默认没有带参数的定义，自动初始化为单位矩阵

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE
            // 像素点坐标误差e = 观测值u,v - 相机模型值u, v
            // 像素点坐标e初始化为0
            Vector2d e = Vector2d::Zero();
            // 第i个相机像素观测值u, v
            Vector2d p2d_tmp = p2d[i];


            // Pw 3x1 --> Pw_4 4x1 非齐次坐标(Xw,Yw,Zw)转换为齐次坐标(Xw,Yw,Zw,1)
            Vector3d Pw = p3d[i];
            Vector4d Pw_4{ p3d[i][0], p3d[i][1], p3d[i][2], 1 };
            // Pw --> Pc 世界坐标系 --> 相机坐标系
            // Pc=R*Pw+t=T*Pw
            Vector4d Pc_4 = T_esti.matrix() * Pw_4;
            // Pc_4 4x1 --> Pc 3x1 齐次坐标(Xc,Yc,Zc,1)转换为非齐次坐标(Xc,Yc,Zc)
            Vector3d Pc{ Pc_4[0], Pc_4[1], Pc_4[2] };;

            // u = fx*Xc/Zc+cx, v = fy*Yc/Zc+cy
            Vector2d p3d_tmp{ fx*Pc[0]/Pc[2] + cx, fy*Pc[1]/Pc[2] + cy };

            // 像素坐标误差e = 相机像素观测值u,v
            //      - 3D坐标按照当前估计的位姿投影得到的相机像素位置(通过相机模型得到的)u, v
            e = p2d_tmp - p3d_tmp;
        // END YOUR CODE HERE

        /*
            // another method
        // compute cost for p3d[I] and p2d[I]
        // START YOUR CODE HERE
            // Pc=R*Pw+t=T*Pw
            Vector3d Pc = T_esti * p3d[i];
            if(DEBUG) cout << "K * Pc: \n" << K*Pc << endl;
            Vector3d v3_e = Vector3d( p2d[i][0], p2d[i][1], 1 ) - K * Pc / Pc[2];
            e[0] = v3_e[0];
            e[1] = v3_e[1];
            if(DEBUG) cout << "v3_e: " << v3_e.transpose() << " e: " << e.transpose() << " Pc: " << Pc.transpose() << endl;
	    // END YOUR CODE HERE
        */

	    // compute jacobian
            Matrix<double, 2, 6> J = Matrix<double, 2, 6>::Zero(); // 需要进行初始化!!!!!
            // START YOUR CODE HERE
            // 相机坐标系下的空间点P'(Pc)坐标x,y,z
            double x = Pc[0];
            double y = Pc[1];
            double z = Pc[2];
            double z_2 = z*z;

            J(0, 0) = -fx / z;
            J(0, 1) = 0;
            J(0, 2) = fx * x / z_2;
            J(0, 3) = fx * x * y / z_2;
            J(0, 4) = -( fx + fx * x * x / z_2);
            J(0, 5) = fx * y/z;

            J(1, 0) = 0;
            J(1, 1) = -fy / z;
            J(1, 2) = fy * y / z_2;
            J(1, 3) = fy + fy * y * y / z_2;
            J(1, 4) = -fy * x * y / z_2;
            J(1, 5) = -fy * x / z;
            // if(DEBUG) cout << "J: \n" << J << endl;
	    // END YOUR CODE HERE

	    // H 6x6 * dx 6x1 = b 6x1
	    // J.transpose() 6x2 *  J 2x6 --> H 6x6
            H += J.transpose() * J;
        // -J.transpose() 6x2 * e 2x1 --> b 6x1
            b += -J.transpose() * e;

            // cost += 1/2 * ( e[0]*e[0] + e[1]*e[1] );

            cost += 0.5 * e.transpose() * e;
            // if(DEBUG) cout << "cost: " << cost << endl;

            /*
            double cost_squaredNorm = e.squaredNorm();
            cost += cost_squaredNorm/2; // 二范数平方的1/2
            double cost_norm = e.norm(); // 返回值是 return sqrt( squaredNorm() )
             // 以下两句也可以求得二范数
            MatrixXd e_e = e * e.transpose(); // 2x2
            cout << cost_squaredNorm << endl;
            cout << e_e.eigenvalues() << endl; // 输出的是2x2的矩阵，(0,0)位置是最大特征值，也是二范数squardNorm
             */
        }
        // if(DEBUG) cout << "H: \n" << H << "\n b: \n" << b << endl;

	// solve dx, dx: se3: Xi = (rho平移, phi旋转).transpose() --> T = exp(Xi^)
        Vector6d dx;

        // START YOUR CODE HERE
        dx = H.ldlt().solve(b);
        cout << "iter: " << iter << "dx: " << dx.transpose() << endl;
        // END YOUR CODE HERE

        if (std::isnan(dx[0])) { // std::isnan需要加上std::作用域，不然找不到对应函数
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE

        /*
        // method 1
        Vector6d T_origin = T_esti.log();
        Vector6d T_se = T_origin + dx; // se3可以继续加法运算
        T_esti = Sophus::SE3::exp( T_se );
         */

        // method 2
        // SE3 T update by se3 dx 左乘
        T_esti = Sophus::SE3::exp(dx)*T_esti; // SE3只能进行乘法运算
        // END YOUR CODE HERE
        
        lastCost = cost;

        // cout.precision(12)控制输出精度
        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}