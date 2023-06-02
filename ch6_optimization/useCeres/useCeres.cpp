#include <iostream>
#include <ceres/ceres.h>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace Eigen;

#define DATA_NUM    100
#define ITER_CNT    100

struct CURVE_FITTING_COST {
    // 构造函数: 接受两个参数 x 和 y, 初始化成员变量_x, _y
    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}        

    // 用模板函数 重载()运算符, 整个结构体当作拟函数使用
    template<typename T>    
    bool operator()(const T *const abc, T *residual) const {    // 第一个const表示指针abc指向常量, 第二个const表示指针T本身为常量, 第三个const表示函数内不允许修改成员变量
        // error = y - exp(a*x^2 + b*x + c)
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
        return true;
    }

    const double _x, _y;    // x, y 在初始化时就确定不再变更
};


int main (int argc, char **argv) {
    /*******************************************
     * 生成100对带高斯噪声的数据
     *******************************************/
    double ar = 1.0, br = 2.0, cr = 1.0;
    double ae = 2.0, be = -1.0, ce = 5.0;

    vector<double> x_data, y_data;
    cv::RNG rng;
    double w_sigma = 1.0;
    double inv_sigma = 1 / w_sigma;

    for (int i = 0; i < DATA_NUM; i++) {
        double x = i / 100.0;
        double y = exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma);

        x_data.push_back(x);
        y_data.push_back(y);
    }

    /*******************************************
     * 配置Ceres
     *******************************************/
    // 构建最小二乘问题(配置Problem)
    ceres::Problem problem;
    double abc[3] = {ae, be, ce};   // 参数块
    
    for (int i = 0; i < DATA_NUM; i++) {
        // AutoDiffCostFunction: 指定残差的计算方式
        // <>内参数: 误差类型, 输出维度(残差块维度), 输入维度(参数块维度)
        // ()内参数: functor
        ceres::CostFunction *costFunc =
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                new CURVE_FITTING_COST(x_data[i], y_data[i]));  // functor

        problem.AddResidualBlock(costFunc, // cost function
                                 nullptr,  // loss function
                                 abc);     // parameter block
    }

    // 配置求解器Solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;      // 增量方程Hx = g如何求解
    options.minimizer_progress_to_stdout = true;    // 输出到cout, 如果这里设置为false, summary中不会有优化信息
    ceres::Solver::Summary summary;     // 实例化ceres优化信息

    // 使用chrono计算运行时间
    chrono::steady_clock::time_point t_str = chrono::steady_clock::now();

    // 使用ceres求解问题
    ceres::Solve(options, &problem, &summary);

    // 使用chrono计算运行时间并打印
    chrono::steady_clock::time_point t_end = chrono::steady_clock::now();
    chrono::duration<double> t_dur =
        chrono::duration_cast<chrono::duration<double>>(t_end - t_str);

    // 输出ceres优化信息（需要options.minimizer_progress_to_stdout = true）
    cout << summary.BriefReport() << endl;

    cout << "execution time = " << t_dur.count() << " seconds" << endl;
    // 输出最终估计参数和真实参数
    cout << "abc true      = " << ar << " " << br << " " << cr << endl;
    cout << "abc estimated = ";
    for (auto xi : abc) {cout << xi << " ";}    // 基于范围的for循环
    cout << endl;

    return 0;
}
