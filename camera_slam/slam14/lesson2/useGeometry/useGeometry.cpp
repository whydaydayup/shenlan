#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char* argv[])
{
    // Quaterniond --> double 、 Quternionf --> float
    Eigen::Quaterniond q1(0.55, 0.3, 0.2, 0.2); //初始化时 (w, x, y, z)
    cout << q1.coeffs() << endl;// coeffs 输出 (x, y, z, w)
    q1.normalize();
    Eigen::Vector3d t1( 0.7, 1.1, 0.2);

    Eigen::Quaterniond q2(-0.1, 0.3,-0.7, 0.2); //初始化时 (w, x, y, z)
    cout << q2.coeffs() << endl;// coeffs 输出 (x, y, z, w)
    q2.normalize(); // Normalizes the quaternion *this
    // q2 = q2.normalized(); // a normalized copy of *this
    cout << q2.coeffs() << endl;
    Eigen::Vector3d t2(-0.1, 0.4, 0.8);

    // Calculate Tcw1 Transform Matrix
    Eigen::Isometry3d Tcw1 = Eigen::Isometry3d::Identity(); // 4x4
    Tcw1.rotate(q1);
    // Quaternion --> Rotation Matrix
    // Eigen::Matrix3d rotation_matrix1 = q1.toRotationMatrix();
    // Tcw1.rotate(rotation_matrix1);
    Tcw1.pretranslate(t1);
    cout << Tcw1.matrix() << endl;

    // Pc1= Tcw1*Pw, Pw=Tcw1.inverse()*Pc1
    Eigen::Vector3d pc1(0.5, -0.1, 0.2);
    Eigen::Vector3d pw = Tcw1.inverse() * pc1;

    // Calculate Tcw2 Transform Matrix
    Eigen::Isometry3d Tcw2 = Eigen::Isometry3d::Identity(); // 4x4
    Tcw2.rotate(q2);
    Tcw2.pretranslate(t2);
    cout << Tcw2.matrix() << endl;
    // Pc2= Tcw2*Pw
    Eigen::Vector3d pc2 = Tcw2 * pw;
    cout << pc2.transpose() << endl;

    Eigen::Matrix4d m = Eigen::Matrix4d::Zero();
    m.block<3,3>(0,0).setIdentity();
    cout << m << endl;


    cout << "START.." << endl;
    Eigen::Isometry3d T_C1_W = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond q_C1_W(0.55, 0.3, 0.2, 0.2);
    cout << "q_C1_W: " << q_C1_W.coeffs().transpose();
    T_C1_W.rotate(q_C1_W.normalized().toRotationMatrix());
    T_C1_W.pretranslate(Eigen::Vector3d(0.7, 1.1, 0.2));
    cout << "T_C1_W:\n" << T_C1_W.matrix() << endl;

    Eigen::Isometry3d T_C2_W = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond q_C2_W(-0.1, 0.3, -0.7, 0.2);
    T_C2_W.rotate(q_C2_W.normalized());
    T_C2_W.pretranslate(Eigen::Vector3d(-0.1, 0.4, 0.8));

    Eigen::Vector3d p_C1(0.5, -0.1, 0.2);
    Eigen::Vector3d p_C2 = T_C2_W * ( T_C1_W.inverse() * p_C1 );
    cout << "p_C2: " << p_C2.transpose() << endl;



    return 0;
}