#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
// Eigen的几何模块
#include <Eigen/Geometry>
int main(int argc, char* argv[])
{
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    // cout << rotation_matrix << endl;
    // 沿着Z轴旋转45度
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0, 0, 1));
    // cout << endl << Eigen::Vector3d::UnitZ();
    // cout.precision(3);
    // 将旋转向量(n,theta)转换成旋转矩阵R
    // 用.matrix()转换
    cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;
    // 直接赋值转换.toRotationMatrix()转换
    rotation_matrix = rotation_vector.toRotationMatrix();

    Eigen::Vector3d v(1, 0, 0);
    cout << v << endl;
    // 用旋转向量
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << endl;
    // 用旋转矩阵
    v_rotated = rotation_matrix * v;
    cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << endl;

    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

    // SE(3)
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate( rotation_vector );
    T.pretranslate( Eigen::Vector3d(1, 3, 4)); //将平移向量设成（1， 3， 4）
    cout << "Transform matrix = \n" << T.matrix() << endl;

    // SE(3)变换矩阵
    Eigen::Vector3d v_transformed = T*v;
    cout << "v Transformed = " << v_transformed.transpose() << endl;
    cout << "v Transformed = \n" << rotation_matrix * v + Eigen::Vector3d(1, 3, 4) << endl;

    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
    cout << "quaterniod = \n" << q.coeffs() << endl; // x,y,z,w

    q = Eigen::Quaterniond(rotation_matrix);
    cout << "quaterniod = \n" << q.coeffs() << endl; // x,y,z,w

    v_rotated = q*v;
    cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << endl;



}