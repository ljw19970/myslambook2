#include <cmath>
#include <iostream>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

int main(int argc, char **argv) {
  /*******************************************
   * 创建3D旋转矩阵rotation_matrix，并用旋转向量rotation_vector对其赋值
   * 创建向量v，分别用旋转矩阵和旋转向量计算旋转后的向量v'
   *******************************************/
  cout << "------------------------------------\n"
       << "Rotation matrix & Rotation vector\n"
       << "------------------------------------\n";
  Matrix3d rotation_matrix = Matrix3d::Identity();

  // 创建旋转向量 (绕z轴旋转45°)
  // AngleAxis的底层不是Matrix，但因为运算进行了重载，所以运算时可以当作矩阵
  AngleAxisd rotation_vector(
      M_PI / 4, Vector3d(0, 0, 1)); // 变量定义后加()，直接对变量进行赋值

  cout.precision(3); // 保留三位小数
  cout << "rotation_vector axis: \n"
       << rotation_vector.axis().transpose() << endl;
  cout << "rotation_vector angle (rad): \n" << rotation_vector.angle() << endl;

  // 用rotation_vector对rotation_matrix进行赋值
  rotation_matrix = rotation_vector.toRotationMatrix();
  cout << "rotation_matrix: \n" << rotation_matrix << endl;

  // 分别用旋转矩阵和旋转向量计算旋转后的向量v'
  Vector3d v(0, 1, 0); // 创建向量v
  cout << "v' (rotation matrix) : \n"
       << (rotation_matrix * v).transpose() << endl;
  cout << "v' (rotation vector) : \n"
       << (rotation_vector * v).transpose() << endl;

  /*******************************************
   * 创建欧拉角euler_angles，并用旋转矩阵对其赋值
   * 查看欧拉角角度
   *******************************************/
  cout << "------------------------------------\n"
       << "Euler angles\n"
       << "------------------------------------\n";
  // 创建欧拉角euler_angles，并用旋转矩阵对其赋值
  Vector3d euler_angles =
      rotation_matrix.eulerAngles(2, 1, 0); // 2,1,0表示ZYX(rpy)
  // 查看欧拉角角度
  cout << "euler_angles (yaw, pitch, roll) (rad): \n"
       << euler_angles.transpose() << endl;

  /*******************************************
   * 创建欧式变换T，并用旋转向量和平移向量对其赋值
   * 用欧式变换计算旋转后的向量v'
   *******************************************/
  cout << "------------------------------------\n"
       << "Euclidean transformation\n"
       << "------------------------------------\n";
  // 创建欧式变换T，虽然是3D但实际上是4*4矩阵
  Isometry3d T = Isometry3d::Identity();

  // 分别用旋转向量和平移向量对其赋值
  T.rotate(rotation_matrix);
  cout << "T: \n" << T.matrix() << endl;
  T = T.Identity();
  T.rotate(rotation_vector);
  cout << "T: \n" << T.matrix() << endl;
  // 平移(1,2,3)，需要注意pretranslate和translate的区别
  T.pretranslate(Vector3d(1, 2, 3)); // 平移向量为(1,2,3)

  // 用旋转矩阵和旋转向量计算旋转后的向量v' (R*v + t)
  cout << "v'(transformation matrix) : \n" << (T * v).transpose() << endl;

  /*******************************************
   * 创建四元数q，分别用旋转向量和旋转矩阵对其赋值
   * 用四元数计算旋转后的向量v'
   *******************************************/
  cout << "------------------------------------\n"
       << "Quaternion\n"
       << "------------------------------------\n";
  // 用旋转向量和旋转矩阵对其赋值
  Quaterniond q = Quaterniond(rotation_matrix);
  cout << "q using rotation matrix (x,y,z,w): \n"
       << q.coeffs().transpose() << endl; // 注意用coeffs打印实部在最后
  q = Quaterniond(rotation_vector);
  cout << "q using rotation vector (x,y,z,w): \n"
       << q.coeffs().transpose() << endl; // 注意用coeffs打印实部在最后

  // 用四元数计算旋转后的向量v'
  cout << "v' (quaternion) : \n"
       << (q * v).transpose() << endl; // 乘法符号会重载, 对应qvq^-1

  // 用数学定义计算旋转后的向量v'
  cout << "v' (quaternion using math) : \n"
       << (q * Quaterniond(0, 0, 1, 0) * q.inverse())
              .coeffs()
              .transpose() // 注意四元数定义时也是(x,y,z,w)
       << endl;

  return 0;
}