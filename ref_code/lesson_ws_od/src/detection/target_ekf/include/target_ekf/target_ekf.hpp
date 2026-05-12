#pragma once
#include <Eigen/Geometry>

/**
 * @brief 欧拉角转四元数，输出为四元数q
 * 
 * @param euler 输入参数，欧拉角
 * @return Eigen::Quaterniond 输出四元数
 */
Eigen::Quaterniond euler2quaternion(const Eigen::Vector3d& euler) {
  double cr = cos(euler(0) / 2);
  double sr = sin(euler(0) / 2);
  double cp = cos(euler(1) / 2);
  double sp = sin(euler(1) / 2);
  double cy = cos(euler(2) / 2);
  double sy = sin(euler(2) / 2);
  Eigen::Quaterniond q;
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;
  return q;
}

/**
 * @brief 四元数转欧拉角
 * 
 * @param q 输入参数，四元数
 * @return Eigen::Vector3d 输出欧拉角
 */
Eigen::Vector3d quaternion2euler(const Eigen::Quaterniond& q) {
  Eigen::Matrix3d m = q.toRotationMatrix();
  Eigen::Vector3d rpy;
  rpy.x() = atan2(m(2, 1), m(2, 2));
  rpy.y() = asin(-m(2, 0));
  rpy.z() = atan2(m(1, 0), m(0, 0));
  return rpy;
}

struct Ekf {
  double dt;
  Eigen::MatrixXd A, B, C;
  Eigen::MatrixXd Qt, Rt;
  Eigen::MatrixXd Sigma, K;
  Eigen::VectorXd x;//列向量

  // states: x, y, z, vx, vy, vz, roll, pitch, yaw

  Ekf(double _dt) : dt(_dt) {
    A.setIdentity(9, 9);//设置A为9×9的单位矩阵
    Sigma.setZero(9, 9);//设置Sigma为9×9的零矩阵
    B.setZero(9, 6);//设置B为9×6的零矩阵
    C.setZero(6, 9);//设置C为6×9的零矩阵
    A(0, 3) = dt;//这里就是1/20
    A(1, 4) = dt;
    A(2, 5) = dt;
    double t2 = dt * dt / 2;//这里是200
    B(0, 0) = t2;
    B(1, 1) = t2;
    B(2, 2) = t2;
    B(3, 0) = dt;
    B(4, 1) = dt;
    B(5, 2) = dt;
    B(6, 3) = dt;
    B(7, 4) = dt;
    B(8, 5) = dt;
    C(0, 0) = 1;
    C(1, 1) = 1;
    C(2, 2) = 1;
    C(3, 6) = 1;
    C(4, 7) = 1;
    C(5, 8) = 1;
    K = C;
    Qt.setIdentity(6, 6);//设置Qt为6×6的单位矩阵
    Rt.setIdentity(6, 6);//设置Rt为6×6的单位矩阵
    Qt(0, 0) = 4;    // x
    Qt(1, 1) = 4;    // y
    Qt(2, 2) = 1;    // z
    Qt(3, 3) = 1;    // roll
    Qt(4, 4) = 1;    // pitch
    Qt(5, 5) = 0.1;  // yaw
    Rt(0, 0) = 0.1;
    Rt(1, 1) = 0.1;
    Rt(2, 2) = 0.1;
    Rt(3, 3) = 0.01;
    Rt(4, 4) = 0.01;
    Rt(5, 5) = 0.01;
    x.setZero(9);//设置为9行1列
  }
  inline void predict() {
    x = A * x;//得到原规模的vecotr  即预测在下一秒的位置是 位置=原来位置+1/20*v  速度不变  欧拉角不变  x_(k+1) = x_(k) + delta(t) * v
    Sigma = A * Sigma * A.transpose() + B * Qt * B.transpose();//这里左侧Sigma就是P'   P' = F * P * F^T + Q    这里A就是F矩阵 
    return;
  }
  inline void reset(const Eigen::Vector3d& z, const Eigen::Vector3d& z_rpy) {
    x.setZero();
    x.head(3) = z;//head操作，取一列元素的前3个，这里代表目标无人机位置
    x.tail(3) = z_rpy;//tail操作，取一列元素的后3个，这里代表目标无人机欧拉角
    Sigma.setZero();
  }
  inline bool update(const Eigen::Vector3d& z, const Eigen::Vector3d& z_rqp) {
    K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();//K是代表卡尔曼增益。K = P'H^T S^-1  所以Sigma是P' C是H  
    Eigen::VectorXd zz(6);
    zz.head(3) = z;
    zz.tail(3) = z_rqp;
    Eigen::VectorXd x_tmp = x + K * (zz - C * x);//计算先验状态和协方差  x = x'+ K * y   y = z - h * x
    // NOTE check valid
    static double vmax = 4;
    if (x_tmp.middleRows(3, 3).norm() > vmax) {
      return false;
    }
    Eigen::Vector3d d_rpy = x.tail(3) - z_rqp;
    x.tail(3).x() = d_rpy.x() > M_PI ? x.tail(3).x() - 2 * M_PI : x.tail(3).x();//完成odom状态的更新。
    x.tail(3).y() = d_rpy.y() > M_PI ? x.tail(3).y() - 2 * M_PI : x.tail(3).y();
    x.tail(3).z() = d_rpy.z() > M_PI ? x.tail(3).z() - 2 * M_PI : x.tail(3).z();
    x.tail(3).x() = d_rpy.x() < -M_PI ? x.tail(3).x() + 2 * M_PI : x.tail(3).x();
    x.tail(3).y() = d_rpy.y() < -M_PI ? x.tail(3).y() + 2 * M_PI : x.tail(3).y();
    x.tail(3).z() = d_rpy.z() < -M_PI ? x.tail(3).z() + 2 * M_PI : x.tail(3).z();
    x = x + K * (zz - C * x);
    Sigma = Sigma - K * C * Sigma;  //完成协方差的更新，左侧Sigma就是协方差P  P = P' - K * H * P' 
    return true;
  }
  /**
   * @brief 返回位置
   * 
   * @return const Eigen::Vector3d 
   */
  inline const Eigen::Vector3d pos() const {
    return x.head(3);
  }
  /**
   * @brief 返回速度
   * 
   * @return const Eigen::Vector3d 
   */
  inline const Eigen::Vector3d vel() const {
    return x.middleRows(3, 3);
  }
  /**
   * @brief 返回欧拉角
   * 
   * @return const Eigen::Vector3d 
   */
  inline const Eigen::Vector3d rpy() const {
    return x.tail(3);
  }
};
