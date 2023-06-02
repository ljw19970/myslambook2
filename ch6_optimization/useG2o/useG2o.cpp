#include <chrono>
#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/eigen_types.h>

#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

using namespace std;
using namespace Eigen;

#define DATA_NUM    100
#define ITER_CNT    100

/// 定义Vertex
// 继承自BaseVertex: 顶点/优化参数_estimate最小维度3, 顶点数据类型
class CurveFittingVertex : public g2o::BaseVertex<3, Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // 内存对齐

    // 顶点更新函数
    virtual void oplusImpl(const double *update) {
        _estimate += Vector3d(update);
    }

    // 顶点重置函数，设定被优化变量的原始值
    virtual void setToOriginImpl() {
        _estimate << 0, 0, 0;
    }

    // 分别是读盘、存盘函数，一般情况下不需要进行读/写操作的话，返回true即可
    virtual bool read(std::istream &is) {return true;}
    virtual bool write(std::ostream &os) const {return true;}
};

/// 定义Edge
// 继承自BaseUnaryEdge: 测量值维度, 类型, 顶点数据类型
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;    // 内存对齐

    CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}  // 构造函数

    // 使用当前顶点值计算的测量值与真实测量值之间的误差 (_error, de)
    virtual void computeError() override {
        // 获得顶点/优化参数_estimate当前值
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Vector3d abc = v->estimate();
        // error = y - exp(ae*x^2 + be*x + ce)
        // _error(0,0) = _measurement - exp(abc(0,0) * _x * _x + abc(1,0) * _x + abc(2,0));
        _error[0] = _measurement - exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
    }

    // 计算雅可比矩阵Jacobian (de / dxi)
    virtual void linearizeOplus() override {
        // staic_cast: 显示类型转换, 类似于C中的(const CurveFittingVertex *)_vertices[0];
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Vector3d abc = v->estimate();

        double ye = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
        _jacobianOplusXi[0] = -_x * _x * ye;    // de / da
        _jacobianOplusXi[1] = -_x * ye;         // de / db
        _jacobianOplusXi[2] = - ye;             // de / dc
    }

    // 分别是读盘、存盘函数，一般情况下不需要进行读/写操作的话，返回true即可
    virtual bool read(std::istream &is) {return true;}
    virtual bool write(std::ostream &os) const {return true;}

public:
    double _x;  // y用_measurement表示, 从Base继承
};

int main (int argc, char **argv) {
    double ar = 1.0, br = 2.0, cr = 1.0;
    double ae = 2.0, be = -1.0, ce = 5.0;

    vector<double> x_data, y_data;

    /*******************************************
     * 生成100对带高斯噪声的数据
     *******************************************/
    cv::RNG rng;
    double w_sigma = 1.0;

    for (int i = 0; i < DATA_NUM; i++) {
        double xi = i / 100.0;
        double yi = exp(ar * xi * xi + br * xi + cr) + rng.gaussian(w_sigma);

        x_data.push_back(xi);
        y_data.push_back(yi);
    }
    
    /*******************************************
     * 使用g2o优化库求解
     *******************************************/
    // 定义BlockSolver和LinearSolver类型
    using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
    using LinearSolverType = g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

    /// 第1步：创建一个线性求解器LinearSolver (求解HΔx = b)
    unique_ptr<BlockSolverType::LinearSolverType> linearSolver(
        new g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>());

    /// 第2步：创建块求解器BlockSolver (求解J和H)
    // 使用线性求解器LinearSolver初始化
    unique_ptr<BlockSolverType> solver_ptr(
        new BlockSolverType(std::move(linearSolver)));
    
    /// 第3步：创建总求解器Solver
    //  迭代方法从GN, LM, DogLeg中选一个, 并用块求解器BlockSolver初始化
    // g2o::OptimizationAlgorithmLevenberg *solver =
    //     new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));  // LM
    g2o::OptimizationAlgorithmGaussNewton *solver =
        new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));   // GN
    // g2o::OptimizationAlgorithmDogleg *solver =
        // new g2o::OptimizationAlgorithmDogleg(std::move(solver_ptr));        // Dogleg
    
    /// 第4步：创建图优化的核心：稀疏优化器SparseOptimizer
    g2o::SparseOptimizer optimizer; // 图模型
    optimizer.setAlgorithm(solver); // 使用solver设置求解方法
    optimizer.setVerbose(true);     // 打开调试输出

    /// 第5步：定义图的顶点和边, 并添加到SparseOptimizer中
    // 向图中增加顶点
    CurveFittingVertex *v = new CurveFittingVertex();
    // 设置顶点属性
    v->setEstimate(Vector3d(ae, be, ce));
    v->setId(0);
    // 向SparseOptimizer添加顶点
    optimizer.addVertex(v); 

    // 向图中增加边
    for (int i = 0; i < DATA_NUM; i++) {
        CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
        // 设置边属性
        edge->setMeasurement(y_data[i]);
        edge->setInformation(Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma));
        edge->setVertex(0, v);
        edge->setId(i);
        // 向SparseOptimizer添加边
        optimizer.addEdge(edge);
    }

    // 使用chrono计算运行时间
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    /// 第6步：设置优化参数，开始执行优化
    optimizer.initializeOptimization();
    optimizer.optimize(10);     // 10: 最多迭代次数

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> t_dur = chrono::duration_cast <chrono::duration<double>>(t2 - t1);
    cout << "execution time = " << t_dur.count() << " seconds" << endl;

    cout << "abc true      = " << ar << " " << br << " " << cr << " " << endl;
    cout << "abc estimated = " << v->estimate().transpose() << endl;
    
    return 0;
}