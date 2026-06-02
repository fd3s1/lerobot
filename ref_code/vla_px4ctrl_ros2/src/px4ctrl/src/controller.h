#ifndef PX4CTRL_CONTROLLER_H
#define PX4CTRL_CONTROLLER_H

#include <array>
#include <queue>
#include <utility>

#include <Eigen/Dense>

#include "PX4CtrlParam.h"
#include "input.h"

struct Desired_State_t
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};
  Eigen::Vector3d j{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  double yaw{0.0};
  double yaw_rate{0.0};

  Desired_State_t() = default;
  explicit Desired_State_t(const Odom_Data_t &odom)
  {
    p = odom.p;
    v.setZero();
    a.setZero();
    j.setZero();
    q = odom.q;
    yaw = uav_utils::normalize_angle(uav_utils::get_yaw_from_quaternion(odom.q));
    yaw_rate = 0.0;
  }
};

struct Controller_Output_t
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d bodyrates{Eigen::Vector3d::Zero()};
  double thrust{0.0};
};

class LinearControl
{
public:
  explicit LinearControl(Parameter_t &param);
  void resetControlState();
  void resetThrustMapping();
  bool estimateThrustModel(const Eigen::Vector3d &est_a, const rclcpp::Time &now);
  Controller_Output_t calculateControl(
    const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu,
    const rclcpp::Time &now);

private:
  Parameter_t &param_;
  Eigen::Vector3d integral_u0_{Eigen::Vector3d::Zero()};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  bool control_initialized_{false};
  double thr2acc_{0.0};
  double P_{1e6};
  std::queue<std::pair<rclcpp::Time, double>> timed_thrust_;

  static constexpr double kAlmostZeroValueThreshold = 1e-3;
  static constexpr double kAlmostZeroThrustThreshold = 1e-2;

  Eigen::Vector3d diagVector(const std::array<double, 3> &value) const;
  Eigen::Vector3d clampVectorByAxis(
    const Eigen::Vector3d &value,
    const std::array<double, 3> &limits) const;
  Eigen::Vector3d computeLimitedTotalAcc(const Eigen::Vector3d &ref_acc) const;
  bool normalizeWithGrad(
    const Eigen::Vector3d &x,
    const Eigen::Vector3d &xd,
    Eigen::Vector3d &x_normalized,
    Eigen::Vector3d &x_normalized_dot) const;
  bool computeFlatInput(
    const Eigen::Vector3d &thrust_acc,
    const Eigen::Vector3d &jerk,
    double yaw,
    double yaw_rate,
    const Eigen::Quaterniond &att_est,
    Eigen::Quaterniond &att,
    Eigen::Vector3d &bodyrates_ff) const;
  Eigen::Vector3d computeFeedBackControlBodyrates(
    const Eigen::Quaterniond &des_q,
    const Eigen::Quaterniond &est_q) const;
  double computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &thrust_acc,
    const Odom_Data_t &odom) const;
};

#endif
