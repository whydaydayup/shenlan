#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"

int main( int argc, char** argv )
{
    // 沿着Z轴旋转90度的旋转矩阵
    Eigen::AngleAxisd Ax( M_PI/2, Eigen::Vector3d(0, 0, 1) );
    Eigen::Matrix3d R = Ax.matrix();

    Sophus::SO3 SO3_R(R); // Sophus::SO(3)可以直接从旋转矩阵构造
    Sophus::SO3 SO3_v(0, 0, M_PI/2); // 可以从旋转向量构造
    Sophus::SO3 SO3_v3((Ax.axis()*Ax.angle())(0), (Ax.axis()*Ax.angle())(1),(Ax.axis()*Ax.angle())(2));
    Eigen::Vector3d v1(0, 0, M_PI/2);
    Sophus::SO3 SO3_v2 = Sophus::SO3::exp(v1);
    Eigen::Quaterniond q(R);
    Sophus::SO3 SO3_q( q ); // 可以从四元数构造
    // 上述等式等价
    // 输出SO(3)时，以so(3)形式输出
    cout << "SO(3) from matrix: \n" << SO3_R.matrix() << endl;
    //cout << "SO(3) from vector: " << SO3_v << endl;
    //cout << "SO(3) from quaternion: " << SO3_q << endl;

    // 使用对数映射获得它的李代数
    Eigen::Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;
    // 使用指数映射获得它的李代数
    Sophus::SO3 SO3_final= Sophus::SO3::exp(so3);
    // hat()得到反对称矩阵
    Eigen::Matrix3d so3_hat = Sophus::SO3::hat(so3);
    Eigen::Vector3d so3_hat_vee = Sophus::SO3::vee(Sophus::SO3::hat(so3));
    cout << endl << Sophus::SO3::exp(so3_hat_vee);

    //增量模型的更新
    Eigen::Vector3d update_so3(1e-4, 0, 0);
    Eigen::Matrix3d updated_matrix = (Sophus::SO3::exp(update_so3)*SO3_R).matrix();
    Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3) * SO3_R;
    cout << "SO3 updated: \n" << SO3_updated << endl;
    cout << "SO3 updated: \n" << SO3_updated.matrix() << endl;

    /**********************************************************************/
    cout << "/*************************************************************/" << endl;
    // 对SE(3)进行操作
    Eigen::Vector3d t(1, 0, 0);
    Sophus::SE3 SE3_Rt(R, t);
    Sophus::SE3 SE3_qt(q, t);
    Sophus::SE3 SE3_t(SO3_R, t);
    cout << "SE3 from R,t = \n" << SE3_Rt << endl;
    cout << "SE3 from q,t = \n" << SE3_qt << endl;

    Eigen::Matrix<double, 6, 1> se3 = SE3_Rt.log();
    cout << "se3 = \n" << se3 << endl;

    cout << "se3 hat = \n" << Sophus::SE3::hat(se3) << endl;
    cout << "se3 hat vee = \n" << Sophus::SE3::vee( Sophus::SE3::hat(se3) ) << endl;

    Eigen::Matrix<double, 6, 1> update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;
    Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3)*SE3_Rt;
    cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;


}

