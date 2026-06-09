#include "controller.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxThr2AccStepRatio = 0.05;

double finite_or(double value, double fallback)
{
  return std::isfinite(value) ? value : fallback;
}

double clamp_symmetric(double value, double limit)
{
  const double abs_limit = std::abs(limit);
  if (abs_limit <= 0.0 || !std::isfinite(abs_limit)) {
    return 0.0;
  }
  return std::clamp(value, -abs_limit, abs_limit);
}

}  // namespace

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
  resetThrustMapping();
  resetControlState();
}

void LinearControl::resetControlState()
{
  integral_u0_.setZero();
  last_control_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  control_initialized_ = false;
  while (!timed_thrust_.empty()) {
    timed_thrust_.pop();
  }
}

void LinearControl::resetThrustMapping()
{
  const double hover_thrust =
    std::clamp(param_.thrust_model.hover_thrust, 0.05, 0.95);
  const double nominal_thr2acc =
    finite_or(param_.controller.gravity, 9.81) / hover_thrust;
  thr2acc_ = std::clamp(
    nominal_thr2acc,
    param_.thrust_model.min_thr2acc,
    param_.thrust_model.max_thr2acc);
  P_ = 1e6;
}

bool LinearControl::estimateThrustModel(
  const Eigen::Vector3d &est_a,
  const rclcpp::Time &now)
{
  if (!param_.thrust_model.enable_estimation || !est_a.allFinite()) {
    return false;
  }

  while (!timed_thrust_.empty()) {
    const auto sample = timed_thrust_.front();
    const double time_passed = (now - sample.first).seconds();
    if (time_passed > 0.045) {
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) {
      return false;
    }

    const double thrust = sample.second;
    timed_thrust_.pop();
    const double measured_acc_z = est_a.z();
    if (!std::isfinite(thrust) || thrust < kAlmostZeroThrustThreshold ||
        thrust < param_.controller.min_thrust || thrust > param_.controller.max_thrust ||
        !std::isfinite(measured_acc_z)) {
      return false;
    }

    const double sample_thr2acc = measured_acc_z / thrust;
    if (!std::isfinite(sample_thr2acc) ||
        sample_thr2acc < param_.thrust_model.min_thr2acc ||
        sample_thr2acc > param_.thrust_model.max_thr2acc) {
      return false;
    }

    const double rho2 = std::clamp(param_.thrust_model.rho2, 0.90, 0.9999);
    const double gamma = 1.0 / (rho2 + thrust * P_ * thrust);
    const double gain = gamma * P_ * thrust;
    const double updated_thr2acc = thr2acc_ + gain * (measured_acc_z - thrust * thr2acc_);
    if (std::isfinite(updated_thr2acc)) {
      const double max_step =
        std::max(0.1, std::abs(thr2acc_) * kMaxThr2AccStepRatio);
      const double bounded_thr2acc =
        thr2acc_ + std::clamp(updated_thr2acc - thr2acc_, -max_step, max_step);
      thr2acc_ = std::clamp(
        bounded_thr2acc,
        param_.thrust_model.min_thr2acc,
        param_.thrust_model.max_thr2acc);
    }
    P_ = std::clamp((1.0 - gain * thrust) * P_ / rho2, 1e-6, 1e8);

    if (param_.thrust_model.print_value) {
      RCLCPP_INFO(
        rclcpp::get_logger("px4ctrl"),
        "[px4ctrl] thr2acc=%.3f gamma=%.3f K=%.3f P=%.3f",
        thr2acc_,
        gamma,
        gain,
        P_);
    }
    return true;
  }
  return false;
}

Controller_Output_t LinearControl::calculateControl(
  const Desired_State_t &des,
  const Odom_Data_t &odom,
  const Imu_Data_t &imu,
  const rclcpp::Time &now,
  Controller_Debug_t *debug)
{
  Controller_Output_t u;
  u.q = odom.q;

  double dt = 0.0;
  if (control_initialized_) {
    dt = (now - last_control_time_).seconds();
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.2) {
      resetControlState();
      dt = 0.0;
    }
  }
  if (!control_initialized_) {
    control_initialized_ = true;
    dt = 0.0;
  }
  last_control_time_ = now;

  const Eigen::Vector3d Kp = diagVector(param_.ude.Kp_diag);
  const Eigen::Vector3d Kd = diagVector(param_.ude.Kd_diag);
  const Eigen::Vector3d T = diagVector(param_.ude.T_diag);
  const Eigen::Vector3d e = des.p - odom.p;
  const Eigen::Vector3d e_dot = des.v - odom.v;
  const Eigen::Vector3d u0 = Kp.asDiagonal() * e + Kd.asDiagonal() * e_dot;
  if (dt > 0.0 && u0.allFinite()) {
    integral_u0_ += u0 * dt;
  }

  Eigen::Vector3d f_hat = Eigen::Vector3d::Zero();
  for (int i = 0; i < 3; ++i) {
    const double t_i = std::max(std::abs(T(i)), 1e-3);
    f_hat(i) = (odom.v(i) - integral_u0_(i)) / t_i;
  }
  f_hat = clampVectorByAxis(f_hat, param_.ude.max_f_hat);

  Eigen::Vector3d u_acc = u0 - f_hat;
  u_acc = clampVectorByAxis(u_acc, param_.ude.max_u_acc);

  Eigen::Vector3d thrust_acc =
    u_acc + des.a + Eigen::Vector3d(0.0, 0.0, finite_or(param_.controller.gravity, 9.81));
  thrust_acc = computeLimitedTotalAcc(thrust_acc);

  Eigen::Quaterniond desired_attitude = odom.q;
  Eigen::Vector3d feedforward_bodyrates = Eigen::Vector3d::Zero();
  if (!computeFlatInput(
      thrust_acc,
      des.j,
      uav_utils::normalize_angle(des.yaw),
      des.yaw_rate,
      odom.q,
      desired_attitude,
      feedforward_bodyrates)) {
    desired_attitude = odom.q;
    feedforward_bodyrates.setZero();
  }

  const Eigen::Vector3d feedback_bodyrates =
    computeFeedBackControlBodyrates(desired_attitude, odom.q);
  u.bodyrates = feedforward_bodyrates + feedback_bodyrates;
  u.bodyrates.x() = clamp_symmetric(u.bodyrates.x(), param_.controller.max_bodyrate_x);
  u.bodyrates.y() = clamp_symmetric(u.bodyrates.y(), param_.controller.max_bodyrate_y);
  u.bodyrates.z() = clamp_symmetric(u.bodyrates.z(), param_.controller.max_bodyrate_z);

  u.thrust = computeDesiredCollectiveThrustSignal(thrust_acc, odom);
  if (!std::isfinite(u.thrust)) {
    u.thrust = param_.controller.min_thrust;
  }
  u.thrust = std::clamp(u.thrust, param_.controller.min_thrust, param_.controller.max_thrust);

  if (debug) {
    debug->des_p = des.p;
    debug->odom_p = odom.p;
    debug->e = e;
    debug->des_v = des.v;
    debug->odom_v = odom.v;
    debug->e_dot = e_dot;
    debug->u0 = u0;
    debug->integral_u0 = integral_u0_;
    debug->f_hat = f_hat;
    debug->u_acc = u_acc;
    debug->thrust_acc_limited = thrust_acc;
    debug->bodyrates_ff = feedforward_bodyrates;
    debug->bodyrates_fb = feedback_bodyrates;
    debug->bodyrates_cmd = u.bodyrates;
    debug->thrust = u.thrust;
    debug->yaw_des = uav_utils::normalize_angle(des.yaw);
    debug->yaw_odom = uav_utils::normalize_angle(uav_utils::get_yaw_from_quaternion(odom.q));
    debug->yaw_error = uav_utils::normalize_angle(debug->yaw_des - debug->yaw_odom);
    debug->dt = dt;
  }

  u.q = imu.q * odom.q.inverse() * desired_attitude;
  if (u.q.norm() > 1e-6) {
    u.q.normalize();
  } else {
    u.q = desired_attitude;
  }

  timed_thrust_.push({now, u.thrust});
  while (timed_thrust_.size() > 100) {
    timed_thrust_.pop();
  }
  return u;
}

Eigen::Vector3d LinearControl::diagVector(const std::array<double, 3> &value) const
{
  return Eigen::Vector3d(value[0], value[1], value[2]);
}

Eigen::Vector3d LinearControl::clampVectorByAxis(
  const Eigen::Vector3d &value,
  const std::array<double, 3> &limits) const
{
  Eigen::Vector3d clamped;
  for (int i = 0; i < 3; ++i) {
    clamped(i) = clamp_symmetric(value(i), limits[static_cast<std::size_t>(i)]);
  }
  return clamped;
}

Eigen::Vector3d LinearControl::computeLimitedTotalAcc(const Eigen::Vector3d &ref_acc) const
{
  if (!ref_acc.allFinite()) {
    return Eigen::Vector3d(0.0, 0.0, finite_or(param_.controller.gravity, 9.81));
  }

  const double max_angle =
    std::max(0.0, param_.controller.max_angle_deg) * kPi / 180.0;
  if (max_angle <= 0.0) {
    return ref_acc;
  }

  const Eigen::Vector3d world_z = Eigen::Vector3d::UnitZ();
  const double ref_norm = ref_acc.norm();
  const double min_collective_acc =
    std::max(param_.controller.min_thrust * std::max(thr2acc_, 1e-3), 0.1);
  if (ref_norm < min_collective_acc) {
    return min_collective_acc * world_z;
  }

  const Eigen::Vector3d desired_z = ref_acc / ref_norm;
  const double dot_z = std::clamp(world_z.dot(desired_z), -1.0, 1.0);
  const double angle = std::acos(dot_z);
  if (angle <= max_angle) {
    return ref_acc;
  }

  const Eigen::Vector3d rot_axis = world_z.cross(desired_z);
  if (rot_axis.norm() < kAlmostZeroValueThreshold) {
    return ref_acc.z() >= 0.0 ? ref_acc : min_collective_acc * world_z;
  }

  const Eigen::Vector3d limited_z =
    Eigen::AngleAxisd(max_angle, rot_axis.normalized()) * world_z;
  const double vertical_acc = std::max(ref_acc.dot(world_z), min_collective_acc);
  const double cos_max = std::max(std::cos(max_angle), 1e-3);
  return (vertical_acc / cos_max) * limited_z;
}

bool LinearControl::normalizeWithGrad(
  const Eigen::Vector3d &x,
  const Eigen::Vector3d &xd,
  Eigen::Vector3d &x_normalized,
  Eigen::Vector3d &x_normalized_dot) const
{
  const double x_sqr_norm = x.squaredNorm();
  if (x_sqr_norm < kAlmostZeroValueThreshold * kAlmostZeroValueThreshold) {
    x_normalized.setZero();
    x_normalized_dot.setZero();
    return false;
  }
  const double x_norm = std::sqrt(x_sqr_norm);
  x_normalized = x / x_norm;
  x_normalized_dot = (xd - x * (x.dot(xd) / x_sqr_norm)) / x_norm;
  return x_normalized.allFinite() && x_normalized_dot.allFinite();
}

bool LinearControl::computeFlatInput(
  const Eigen::Vector3d &thrust_acc,
  const Eigen::Vector3d &jerk,
  double yaw,
  double yaw_rate,
  const Eigen::Quaterniond &att_est,
  Eigen::Quaterniond &att,
  Eigen::Vector3d &bodyrates_ff) const
{
  Eigen::Vector3d zb;
  Eigen::Vector3d zbd;
  if (!normalizeWithGrad(thrust_acc, jerk, zb, zbd)) {
    att = att_est;
    bodyrates_ff.setZero();
    return false;
  }

  const double sin_yaw = std::sin(yaw);
  const double cos_yaw = std::cos(yaw);
  const Eigen::Vector3d xc(cos_yaw, sin_yaw, 0.0);
  const Eigen::Vector3d xcd(-sin_yaw * yaw_rate, cos_yaw * yaw_rate, 0.0);
  const Eigen::Vector3d yc = zb.cross(xc);
  const Eigen::Vector3d ycd = zbd.cross(xc) + zb.cross(xcd);

  Eigen::Vector3d yb;
  Eigen::Vector3d ybd;
  if (!normalizeWithGrad(yc, ycd, yb, ybd)) {
    att = att_est;
    bodyrates_ff.setZero();
    return false;
  }

  const Eigen::Vector3d xb = yb.cross(zb);
  const Eigen::Vector3d xbd = ybd.cross(zb) + yb.cross(zbd);
  bodyrates_ff.x() = (zb.dot(ybd) - yb.dot(zbd)) / 2.0;
  bodyrates_ff.y() = (xb.dot(zbd) - zb.dot(xbd)) / 2.0;
  bodyrates_ff.z() = (yb.dot(xbd) - xb.dot(ybd)) / 2.0;

  Eigen::Matrix3d rot;
  rot.col(0) = xb;
  rot.col(1) = yb;
  rot.col(2) = zb;
  att = Eigen::Quaterniond(rot);
  if (att.norm() > 1e-6) {
    att.normalize();
  } else {
    att = att_est;
    bodyrates_ff.setZero();
    return false;
  }
  return bodyrates_ff.allFinite();
}

Eigen::Vector3d LinearControl::computeFeedBackControlBodyrates(
  const Eigen::Quaterniond &des_q,
  const Eigen::Quaterniond &est_q) const
{
  Eigen::Quaterniond q_error = est_q.inverse() * des_q;
  if (q_error.norm() > 1e-6) {
    q_error.normalize();
  } else {
    return Eigen::Vector3d::Zero();
  }

  const Eigen::Vector3d KAng = diagVector(param_.attitude.KAng_diag);
  const double sign = q_error.w() >= 0.0 ? 1.0 : -1.0;
  return Eigen::Vector3d(
    sign * 2.0 * KAng.x() * q_error.x(),
    sign * 2.0 * KAng.y() * q_error.y(),
    sign * 2.0 * KAng.z() * q_error.z());
}

double LinearControl::computeDesiredCollectiveThrustSignal(
  const Eigen::Vector3d &thrust_acc,
  const Odom_Data_t &odom) const
{
  if (!thrust_acc.allFinite() || thr2acc_ < 1e-3) {
    return param_.controller.min_thrust;
  }
  Eigen::Vector3d body_z = odom.q * Eigen::Vector3d::UnitZ();
  if (body_z.norm() < 1e-6 || !body_z.allFinite()) {
    body_z = Eigen::Vector3d::UnitZ();
  } else {
    body_z.normalize();
  }
  return thrust_acc.dot(body_z) / thr2acc_;
}
