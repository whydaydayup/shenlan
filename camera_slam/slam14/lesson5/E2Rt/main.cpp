//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

using namespace Eigen;

// 头文件不是so3.hpp !!!!!
// #include <sophus/so3.hpp> 错误!!!!!!!
#include <sophus/so3.h>

#include <iostream>

using namespace std;

void swap(double &x, double &y)
{
    double tmp;
    tmp = x;
    x = y;
    y = tmp;
}

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values // SVD分解和奇异值的处理
    // START YOUR CODE HERE
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(E, ComputeThinU | ComputeThinV);
    Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();

    // one answer
    Vector3d S = svd.singularValues(); // singularValues()得出奇异值
    double a = S[0], b = S[1], c = S[2];
    if( a<b ) swap(a, b);
    if( b<c ) swap(b, c);
    if( a<b ) swap(a, b);
    Matrix3d T = Eigen::Matrix3d::Identity();
    T(0, 0) = (a+b)/2;
    T(1, 1) = (a+b)/2;
    T(2, 2) = 0;
    cout << T << endl;

    // another answer
    Matrix3d Sigma = U.inverse() * E * V.transpose().inverse();
    vector<double> tao = { Sigma(0, 0), Sigma(1, 1), Sigma(2, 2) };
    sort(tao.begin(), tao.end()); // sort从小到大排列
    cout << tao[0] << tao[1] << tao[2];
    double tao_mean = (tao[1] + tao[2])/2;
    Matrix3d SigmaX = Matrix3d::Zero();
    SigmaX(0, 0) = SigmaX(1, 1) = tao_mean;

    Vector3d SigmaVector{ (S[0]+S[1])/2, (S[0]+S[1])/2, 0 };
    Matrix3d SigmaMatrix = SigmaVector.asDiagonal(); // 将向量转换成对角矩阵!!!!!!

    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d Rz1 = AngleAxisd( M_PI/2, Vector3d(0, 0, 1) ).toRotationMatrix();
    // Matrix3d Rz1 = AngleAxisd( M_PI/2, Vector3d(0, 0, 1) ).matrix(); // 也可以直接使用.matrix()转换成旋转矩阵
    Matrix3d Rz2 = AngleAxisd( -M_PI/2, Vector3d(0, 0, 1) ).toRotationMatrix();
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;
    t_wedge1 = U * Rz1 * T * U.transpose();
    t_wedge2 = U * Rz2 * T * U.transpose();

    Matrix3d R1;
    Matrix3d R2;
    R1 = U * Rz1.transpose() * V.transpose();
    R2 = U * Rz2.transpose() * V.transpose();
    // END YOUR CODE HERE

    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "t1 = " << Sophus::SO3::vee(t_wedge1) << endl;
    cout << "t2 = " << Sophus::SO3::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << tR << endl;

    return 0;
}