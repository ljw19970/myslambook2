#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

#define DATA_NUM    100     // 数据数量
#define ITER_CNT    100     // 最多迭代次数

int main (int agrc, char **argv) {
    double ar = 1.0, br = 2.0, cr = 1.0;     // 参数真实值
    double ae = 2.0, be = -1.0, ce = 5.0;    // 参数估计值  
    vector<double> x_data, y_data;      // 数据对
    double lastCost = 0.0;

    /*******************************************
     * 生成100对带高斯噪声的数据
     *******************************************/
    // 使用cv库创建高斯噪声 (RNG: random number generator)
    cv::RNG rng;
    double w_sigma = 1.0;
    double inv_sigma = 1.0 / w_sigma;

    // 计算得到带噪声的观测数据y，并将数据对存入vector中
    for (int i = 0; i < DATA_NUM; i++) {
        double x = (double)i / 100; // 不加(double)强制转换的话, 所有x的类型为int
        // double x = i / 100.0    // 使用这个方法也行
        double y = exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma);

        x_data.push_back(x);
        y_data.push_back(y);
    }

    // 打印生成的数据
    // for (int i = 0; i < DATA_NUM; i++) {
    //     cout << "x = " << x_data[i] << "\ty = " << y_data[i] << endl;
    // }

    // 使用chrono库获取初始时间
    chrono::steady_clock::time_point t_stt = chrono::steady_clock::now();

    /*******************************************
     * 高斯牛顿法迭代更新参数 abc
     *******************************************/
    for (int iter = 0; iter < ITER_CNT; iter++) {
        Matrix3d H = Matrix3d::Zero();     // Hessian Matrix
        Vector3d g = Vector3d::Zero();     // error_sum
        
        double cost = 0.0;

        // 获得增量方程的参数H和g
        for (int i = 0; i < DATA_NUM; i++) {
            double x = x_data[i], y = y_data[i];
            double y_e = exp(ae * x * x + be * x + ce);
            double error = y - y_e;     // 计算误差

            // 计算各个参数的Jacobian
            Vector3d J; // Jacobian Matrix
            J[0] = -x * x * y_e;        // de/da
            J[1] = -x * y_e;            // de/db
            J[2] = -y_e;                // de/dc

            H += inv_sigma * inv_sigma * J * J.transpose();
            g += -inv_sigma * inv_sigma * error * J;

            cost += error * error;
        }

        // 求解增量方程 H dx = g
        Vector3d dx = H.ldlt().solve(g);        
        if (isnan(dx[0])) {
            cerr << "dx is nand" << endl;
            break;
        }

        // 判断是否达到最优值
        if (iter > 0 && cost >= lastCost) {   // 表示已经达到最优值
            cout << "cost >= lastCost, reached optimum value" << endl;
            break;
        }

        // 更新cost
        lastCost = cost;

        // 更新x
        ae += dx[0];
        be += dx[1];
        ce += dx[2];
        
        cout << "cost = " << cost << "\t\tupdate = " << dx.transpose()
             << "\t\t estimated abc = " << ae << " " << be << " " << ce
             << endl;
    }

    // 获取结束时间并计算所用时间
    chrono::steady_clock::time_point t_end = chrono::steady_clock::now();
    chrono::duration<double> t_dur =
        chrono::duration_cast<chrono::duration<double>>(t_end - t_stt);
    cout << "execution time = " << t_dur.count() << " seconds" << endl;

    // 打印最终的参数估计值
    cout << "abc true      = " << ar << " " << br << " " << cr << endl;
    cout << "abc estimated = " << ae << " " << be << " " << ce << endl;

    return 0;
}