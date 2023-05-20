#include <iostream>
#include <fstream>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>

#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

const string groundtruth_file = "../data/groundtruth.txt";
const string estimated_file = "../data/estimated.txt";

// 绘制gt和esti路径; 不修改变量的值加入const; 使用引用避免变量的复制
void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);

// 读取文件中的轨迹并保存
TrajectoryType ReadTrajectory(const string &filepath);


int main (int argv, char **argc) {
    /// 从文件中读取轨迹
    TrajectoryType gt_trajectory = ReadTrajectory(groundtruth_file);
    TrajectoryType esti_trajectory = ReadTrajectory(estimated_file);

    // assert的作用: 在debug模式下, 如果assert中的表达式不相等, 则程序停止运行，并输出错误信息
    assert(!gt_trajectory.empty() && !esti_trajectory.empty());
    assert(gt_trajectory.size() == esti_trajectory.size());

    // cout.precision(3);
    // cout << "size = " << gt_trajectory.size() << endl;
    // cout << gt_trajectory[1].matrix() << endl;

    /// 根据两条轨迹计算rmse, rmse_all = sqrt (1/N * se3_error.norm^2)
    double rmse_all = 0, rmse_trans = 0;
    for (int i = 0; i < gt_trajectory.size(); i++) {
        Sophus::SE3d SE3_error = gt_trajectory[i].inverse() * esti_trajectory[i];
        // 计算位姿李代数的rmse
        double error_all = SE3_error.log().norm();      
        rmse_all += error_all * error_all;

        // 计算trans部分的rmse
        double error_trans = SE3_error.translation().norm();
        rmse_trans += error_trans * error_trans;
    }
    
    rmse_all = sqrt(rmse_all / double(gt_trajectory.size()));
    cout << "rmse_all = " << rmse_all << endl;

    rmse_trans = sqrt(rmse_trans / double(gt_trajectory.size()));
    cout << "rmse_trans = " << rmse_trans << endl;

    /// 绘制两条轨迹
    DrawTrajectory(gt_trajectory, esti_trajectory);

    return 0;
}

TrajectoryType ReadTrajectory(const string &filepath) {
    TrajectoryType trajectory;
    ifstream fin(filepath);

    // 从文件中读取数据并存储到trajectory
    if (!fin) {
        cout << "cannot find trajectory file at " << filepath << endl;
        return trajectory;
    }

    while (!fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

        Sophus::SE3d SE3_read(Quaterniond(qx, qy, qz, qw), Vector3d(tx, ty, tz)); // 通过四元数+平移量初始化SE3
        trajectory.push_back(SE3_read);
    }

    return trajectory;
}

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti) {
    // 创建一个名为 "Trajectory Viewer" 的窗口
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    // 启用深度测试和混合功能
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 定义了 OpenGL 渲染状态，包括投影矩阵和相机视角
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    // 创建一个视图对象 d_cam，指定其位置、大小和处理器
    pangolin::View &d_cam =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    // 绘制图像
    while (pangolin::ShouldQuit() == false) {
        // 清空颜色缓冲区和深度缓冲区
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // 激活视图，并设置渲染状态
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);
        
        // 画出各个位姿之间的连线
        for (int i = 0; i < gt.size(); i++) {
            // 画gt连线
            glBegin(GL_LINES);
            glColor3f(0.0, 0.0, 1.0);
            auto p1 = gt[i], p2 = gt[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();

            // 画esti连线
            glBegin(GL_LINES);
            glColor3f(1.0, 0.0, 0.0);
            p1 = esti[i], p2 = esti[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();

        // 延迟 5 毫秒，控制帧率
        usleep(5000);
    }
}