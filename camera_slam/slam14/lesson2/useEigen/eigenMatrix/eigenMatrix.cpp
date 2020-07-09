#include <iostream>
#include <ctime>
using namespace std;

// Eigen部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆、特征值等）
#include <Eigen/Dense>

#define MATRIX_SIZE 50

int main(int argc, char* argv[])
{
    Eigen::Matrix<float, 2, 3> matrix_23;
    Eigen::Vector3d v_3d;
    Eigen::Matrix<double, 3, 1> matrix_31;
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();// 初始化为0
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > matrix_dynamic;
    Eigen::MatrixXd matrix_x;

    matrix_23 << 1, 2, 3, 4, 5, 6;
    cout << matrix_23 << endl << endl;

    for (int i=0; i<2; i++ )
        for (int j=0; j<3; j++ )
            cout << matrix_23(i, j) << endl;
    cout << endl;

    v_3d << 3, 2, 1;
    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    cout << result << endl;

    matrix_33 = Eigen::Matrix3d::Random();
    cout << matrix_33 << endl << endl;

    cout << matrix_33.transpose() << endl;
    cout << matrix_33.sum() << endl;
    cout << matrix_33.trace() << endl;
    cout << 10*matrix_33 << endl;
    cout << matrix_33.inverse() << endl;
    cout << matrix_33.determinant() << endl;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver( matrix_33.transpose() * matrix_33 );
    cout << endl << eigen_solver.eigenvalues() << endl;
    cout << endl << eigen_solver.eigenvalues()[0] << endl;

    // 求解matrix_NN * x = v_Nd
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd;
    v_Nd = Eigen::VectorXd::Random(MATRIX_SIZE);

    clock_t time_stt = clock(); // clock start
    //直接求逆矩阵
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "time use in normal inverse is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
    cout << endl << CLOCKS_PER_SEC << endl;

    //通常使用矩阵分解来求解，例如QR分解，速度快很多
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "time use in QR composition is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
    bool a_solution_exists = (matrix_NN*x).isApprox(v_Nd, 10e-10);
    cout << endl << a_solution_exists << endl;

    return 0;
}
