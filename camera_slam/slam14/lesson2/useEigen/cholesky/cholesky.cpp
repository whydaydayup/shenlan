#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
using namespace std;

#define MATRIX_SIZE 100

int main(int argc, char* argv[])
{
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_A = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_A = matrix_A.array().abs().matrix();
    for(int i = 0; i < MATRIX_SIZE; i++ )
        for(int j = 0; j < MATRIX_SIZE; j++ )
            if(i>j)
                matrix_A(i,j) = matrix_A(j,i);
    // cout << matrix_A;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> vector_b = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> x = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);


    clock_t start = clock();
    x = matrix_A.inverse() * vector_b;
    cout << x.transpose() << endl;
    cout << "time use in Direct comosition is " << 1000* (clock() - start) / (double)CLOCKS_PER_SEC << "ms" << endl;

    start = clock();
    x = matrix_A.fullPivHouseholderQr().solve(vector_b);
    cout << x.transpose() << endl;
    cout << "time use in QR comosition is " << 1000* (clock() - start) / (double)CLOCKS_PER_SEC << "ms" << endl;

    start = clock();
    x = matrix_A.llt().solve(vector_b) ;
    cout << x.transpose() << endl;
    cout << "time use in llt comosition is " << 1000* (clock() - start) / (double)CLOCKS_PER_SEC << "ms" << endl;

    start = clock();
    x = matrix_A.ldlt().solve(vector_b) ;
    cout << x.transpose() << endl;
    cout << "time use in ldlt comosition is " << 1000* (clock() - start) / (double)CLOCKS_PER_SEC << "ms" << endl;

    return 0;
}
