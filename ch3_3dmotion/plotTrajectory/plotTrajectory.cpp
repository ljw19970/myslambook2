#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;

#define TRAJECTORY_FILE_PATH "../trajectory.txt"

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);

int main(int argc, char **argv) {
  vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;

  // 从文件中读取数据并存储到poses
  ifstream fin(TRAJECTORY_FILE_PATH);
  if (!fin) {
    cout << "cannot find trajectory file at " << TRAJECTORY_FILE_PATH << endl;
    return 1;
  }

  while (!fin.eof()) {
    double time, tx, ty, tz, qx, qy, qz, qw;
    // 使用输入运算符 >>
    // 进行数据读取时，它会自动跳过空白字符，直到找到下一个非空白字符作为数据的开始
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

    // 将数据转换为T
    Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
    Twr.pretranslate(Vector3d(tx, ty, tz));

    // 将数据存储到poses中
    poses.push_back(Twr);
  }

  cout << "read total " << poses.size() << " pose entires" << endl;

  // 绘制轨迹
  DrawTrajectory(poses);

  return 0;
}

/*******************************************************************************************/
void DrawTrajectory(
    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses) {
  // 创建一个名为 "Trajectory Viewer" 的窗口，并设置窗口的宽度为 1024，高度为
  // 768
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
  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false) {
    // 清空颜色缓冲区和深度缓冲区
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // 激活视图，并设置渲染状态
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);

    // 画每个位姿的三个坐标轴
    for (size_t i = 0; i < poses.size(); i++) {
      Vector3d Ow = poses[i].translation(); // 获取T中的t
      // 下面的乘法运算符应该进行了重载(poses[i]是4*4的，Vector是3*1的)
      Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
      Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
      Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));

      // 画坐标轴
      glBegin(GL_LINES);
      glColor3f(1.0, 0.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Xw[0], Xw[1], Xw[2]);
      glColor3f(0.0, 1.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Yw[0], Yw[1], Yw[2]);
      glColor3f(0.0, 0.0, 1.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Zw[0], Zw[1], Zw[2]);
      glEnd();
    }
    // 画出各个位姿之间的连线
    for (size_t i = 0; i < poses.size(); i++) {
      glColor3f(0.0, 0.0, 0.0);
      glBegin(GL_LINES);
      auto p1 = poses[i], p2 = poses[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();

    // 延迟 5 毫秒，控制帧率
    usleep(5000);
  }
}