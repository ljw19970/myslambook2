#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

int main (int argc, char **argv) {
    /*******************************************
     * SO(3)的操作
     *******************************************/
    cout << "*******************************************\n"
         << "SO(3) operation\n"
         << "*******************************************\n";
    /// 从旋转矩阵和四元数构造 SO(3) 李群
    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));
    Matrix3d m3d = rotation_vector.toRotationMatrix();
    Quaterniond q(rotation_vector);

    Sophus::SO3d SO3_m(m3d);    // 旋转矩阵 --> SO3 
    Sophus::SO3d SO3_q(q);      // 四元数 --> SO3
    // cout << "SO3_m = \n" << SO3_m.matrix() << endl;

    /// 从 SO(3) 李群获取对应的李代数(so3)
    Vector3d so3 = SO3_m.log();
    cout.precision(3);
    cout << "so3 = " << so3.transpose() << endl;
    cout << "SO3 from so3 = \n" << Sophus::SO3d::exp(so3).matrix() << endl;

    /// 使用 hat 和 vee 操作进行李代数与对应反对称矩阵的转换
    Matrix3d so3_hat = Sophus::SO3d::hat(so3);
    cout << "so3_hat = \n" << so3_hat << endl;
    cout << "so3_hat vee = \n" << Sophus::SO3d::vee(so3_hat).transpose() << endl;

    /// 模拟了一个小的旋转更新量，并使用指数映射(exp()) 将其转化为 SO(3)
    /// 李群，然后左乘到原来的 SO(3)李群上，从而实现了 SO(3) 的更新
    Vector3d delta_so3(1e-4, 0, 0);       // ΔΦ
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(delta_so3) * SO3_m;   // 左乘一个扰动
    cout << "SO3_updated = \n" << SO3_updated.matrix() << endl;


    /*******************************************
     * SE(3)的操作
     *******************************************/
    cout << "*******************************************\n"
         << "SE(3) operation\n"
         << "*******************************************\n";
    /// 用 Sophus::SE3d 类从旋转矩阵/四元数和平移向量构造 SE(3) 李群
    Sophus::SE3d SE3_m(m3d, Vector3d(5, 4, 1));
    Sophus::SE3d SE3_q(q, Vector3d(5, 4, 1));
    Sophus::SE3d SE3_so3(SO3_m, Vector3d(5, 4, 1));
    // cout << "SE3_m = \n " << SE3_m.matrix() << endl;
    // cout << "SE3_q = \n " << SE3_q.matrix() << endl;
    // cout << "SE3_so3 = \n " << SE3_so3.matrix() << endl;

    /// 从 SE(3) 李群获取对应的李代数(se3)
    Sophus::Vector6d se3 = SE3_m.log();
    cout << "se3 = \n" << se3.transpose() << endl;
    cout << "SE3 from se3 = \n" << Sophus::SE3d::exp(se3).matrix() << endl;

    // 使用 hat 和 vee 操作
    Sophus::Matrix4d se3_hat = Sophus::SE3d::hat(se3);
    cout << "se3 hat = \n" << se3_hat << endl;
    cout << "se3 hat vee = \n" << Sophus::SE3d::vee(se3_hat).transpose() << endl;

    /// 模拟了一个小的平移更新量，并使用指数映射将其转化为
    /// SE(3)李群，然后左乘到原来的 SE(3) 李群上，从而实现了 SE(3) 的更新
    Sophus::Vector6d delta_se3;     // 不能直接创建的时候就赋值，因为没有对应的构造函数
    delta_se3.setZero();
    delta_se3(0) = delta_se3(1) = 1e-3;
    // cout << "delta_se3 = \n" << delta_se3.transpose() << endl;

    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(delta_se3) * SE3_m;
    cout << "SE3_updated = \n" << SE3_updated.matrix() << endl;     // 扰动太小可能看不出来

    return 0;
}
