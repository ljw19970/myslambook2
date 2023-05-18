#include <iostream>
using namespace std;

#include <ctime>
#include <Eigen/Core>   // Eigen核心
#include <Eigen/Dense>  // Eigen稠密矩阵的代数运算(逆, 特征值等)
using namespace Eigen;  // TODO: 了解using namespace的含义

#define MATRIX_SIZE 50

int main (int argc, char **agrv) {
    /*******************************************
     * 矩阵变量声明
     *******************************************/
    // 声明一个2*3的float类型矩阵
    Matrix<float, 2, 3> matrix_23;

    /* Eigen有一些内置类型，但底层还是Eigen::Matrix */
    // Vector3d本质上是 Eigen::Matrix<double, 3, 1>
    Vector3d v_3d;

    // Matrix3d本质上是 Eigen::Matrix<double, 3, 3>
    Matrix3d m_3d = Matrix3d::Zero();

    /* 可以使用动态大小的矩阵 */
    // - Method 1
    Matrix<double, Dynamic, Dynamic> matrix_dynamic;
    // - Method 2
    MatrixXd m_xd;

    /*******************************************
     * 矩阵基础操作
     *******************************************/
    // 给矩阵赋值
    matrix_23 << 1, 2, 3, 4, 5, 6;
    m_3d = Matrix3d::Random();  // 还可以使用Matrix3d::Zero()

    // 访问矩阵
    // - 直接访问整个矩阵的值
    cout << "matrix_23: \n" << matrix_23 << endl;

    // - 用()访问矩阵中的元素: matrix(i, j)
    cout << "print matrix 2*3: \n" << matrix_23 << endl;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            cout << matrix_23(i, j) << "\t";
        }
        cout << endl;
    }

    Matrix<float, 2, 2> m_22;
    m_22 << 0,0,0,0;
    matrix_23.block<2,2>(0,0) = m_22;
    cout << "matrix_23: \n" << matrix_23 << endl;

    /*******************************************
     * 矩阵基础运算
     *******************************************/
    // 乘法(向量和矩阵的乘法本质上也是矩阵乘法)
    v_3d << 3, 2, 1;        // 数据类型为double
    Matrix<float, 3, 1> m_31;   // 数据类型为float
    m_31 << 4, 5, 6;

    Matrix<float, 2, 1> result;
    result = matrix_23 * m_31;  // 正确, 因为两个数据类型都是float
    cout << "[1,2,3;4,5,6] * [4,5,6] = \n" << result << endl;

    // 显式转换数据类型
    // result = matrix_23 * v_3d; // 错误, 因为数据类型分别为float和double
    result = matrix_23 * v_3d.cast<float>(); // 显式转换v_3d的数据类型
    cout << "[1,2,3;4,5,6] * [1,2,3] = \n" << result << endl;

    // 迹 trace
    cout << "trace of matrix 3*3 = \n" << m_3d.trace() << endl;

    // 转置 transpose
    cout << "transpose of vector 3*1: \n" << v_3d.transpose() << endl;

    // 求和 sum
    cout << "sum of matrix 2*3: \n" << matrix_23.sum() << endl;

    // 数乘
    cout << "matrix_23 * 10 = \n" << 10 * matrix_23 << endl;
    cout << "matrix_23 / 10= \n" << matrix_23 / 10 << endl;

    // 逆 inverse
    cout << "inverse of matrix 3*3 = \n" << m_3d.inverse() << endl;

    // 行列式 determinant
    cout << "det of matrix 3*3 = \n" << m_3d.determinant() << endl;

    // 特征值 eigenvalue
    // 实对称矩阵能保证对角化成功，实对称矩阵 A^T * A
    SelfAdjointEigenSolver<Matrix3d> eigen_solver(m_3d.transpose() * m_3d); 
    cout << "Eigen values  = \n" << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << endl;

    /*******************************************
     * 解方程 (matrix_NN * x = v_Nd)
     *******************************************/
    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN 
        = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE); // 注意MatrixXd的使用
    matrix_NN = matrix_NN * matrix_NN.transpose(); // 保证半正定（保证对角化成功）
    Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);
    Matrix<double, MATRIX_SIZE, 1> x;

    // Method1: 直接求逆
    clock_t time_stt = clock(); // start time

    x = matrix_NN.inverse() * v_Nd;
    cout << "Method 1 execution time: "
         << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x^T = " << x.transpose() << endl;

    // Method2: QR分解
    time_stt = clock();

    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "Method 2 execution time: "
         << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x^T = " << x.transpose() << endl;

    // Method3: cholesky分解
    time_stt = clock();

    x = matrix_NN.ldlt().solve(v_Nd);
    cout << "Method 3 execution time: "
         << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x^T = " << x.transpose() << endl ;

    return 0;
}