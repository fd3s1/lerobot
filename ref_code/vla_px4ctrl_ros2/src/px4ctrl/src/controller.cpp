#include "controller.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxThr2AccStepRatio = 0.05;
constexpr std::array<double, 3> kMaxBodyrateDot{120.0, 120.0, 60.0};

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
  filtered_velocity_.setZero();
  last_control_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  control_initialized_ = false;
  velocity_filter_initialized_ = false;
  yaw_rate_lpf_ = 0.0;
  yaw_rate_lpf_initialized_ = false;
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
  const Eigen::Vector3d velocity_lpf_tau = diagVector(param_.ude.velocity_lpf_tau_s);
  if (!velocity_filter_initialized_ || dt <= 0.0 || !filtered_velocity_.allFinite()) {
    filtered_velocity_ = odom.v;
    velocity_filter_initialized_ = true;
  } else {
    for (int i = 0; i < 3; ++i) {
      const double tau = velocity_lpf_tau(i);
      if (std::isfinite(tau) && tau > 0.0) {
        const double alpha = std::clamp(1.0 - std::exp(-dt / tau), 0.0, 1.0);
        filtered_velocity_(i) += alpha * (odom.v(i) - filtered_velocity_(i));
      } else {
        filtered_velocity_(i) = odom.v(i);
      }
    }
  }
  const Eigen::Vector3d e = des.p - odom.p;
  const Eigen::Vector3d e_dot = des.v - filtered_velocity_;
  const Eigen::Vector3d u_d = des.a;
  const Eigen::Vector3d u0 = u_d + Kp.asDiagonal() * e + Kd.asDiagonal() * e_dot;
  if (param_.ude.enable && dt > 0.0 && u0.allFinite()) {
    integral_u0_ += u0 * dt;
  }

  Eigen::Vector3d f_hat = Eigen::Vector3d::Zero();
  if (param_.ude.enable) {
    for (int i = 0; i < 3; ++i) {
      const double t_i = std::max(std::abs(T(i)), 1e-3);
      f_hat(i) = (filtered_velocity_(i) - integral_u0_(i)) / t_i;
    }
    f_hat = clampVectorByAxis(f_hat, param_.ude.max_f_hat);
  }

  Eigen::Vector3d u_acc = param_.ude.enable ? (u0 - f_hat) : u0;
  u_acc = clampVectorByAxis(u_acc, param_.ude.max_u_acc);

  Eigen::Vector3d thrust_acc =
    u_acc + Eigen::Vector3d(0.0, 0.0, finite_or(param_.controller.gravity, 9.81));
  thrust_acc = computeLimitedTotalAcc(thrust_acc);
  u.thrust_acc = thrust_acc;

  Eigen::Quaterniond desired_attitude = odom.q;
  Eigen::Vector3d feedforward_bodyrates = Eigen::Vector3d::Zero();
  Eigen::Vector3d feedforward_bodyrates_dot = Eigen::Vector3d::Zero();
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

  if (param_.physical_control.angular_acceleration_feedforward_scale > 1e-6) {
    Eigen::Quaterniond second_order_attitude = desired_attitude;
    Eigen::Vector3d second_order_bodyrates = Eigen::Vector3d::Zero();
    if (!computeFlatInputSecondOrder(
        thrust_acc,
        des.j,
        des.snap,
        uav_utils::normalize_angle(des.yaw),
        des.yaw_rate,
        des.yaw_acceleration,
        odom.q,
        second_order_attitude,
        second_order_bodyrates,
        feedforward_bodyrates_dot)) {
      feedforward_bodyrates_dot.setZero();
    }
  }

  const double yaw_odom =
    uav_utils::normalize_angle(uav_utils::get_yaw_from_quaternion(odom.q));
  const double yaw_error =
    uav_utils::normalize_angle(uav_utils::normalize_angle(des.yaw) - yaw_odom);
  double yaw_error_deadbanded = yaw_error;
  double yaw_rate_cmd_raw = des.yaw_rate;
  Eigen::Vector3d feedback_bodyrates = Eigen::Vector3d::Zero();
  if (param_.attitude.feedback_mode == "full_quaternion") {
    feedback_bodyrates = computeFeedBackControlBodyrates(desired_attitude, odom.q);
    yaw_error_deadbanded = yaw_error;
    yaw_rate_cmd_raw = feedforward_bodyrates.z() + feedback_bodyrates.z();
    yaw_rate_lpf_ = 0.0;
    yaw_rate_lpf_initialized_ = false;
  } else {
    const double independent_yaw_rate =
      computeIndependentYawRate(
        yaw_error,
        des.yaw_rate,
        dt,
        yaw_error_deadbanded,
        yaw_rate_cmd_raw);
    feedback_bodyrates =
      computeReducedAttitudeFeedbackBodyrates(thrust_acc, odom.q);
    feedback_bodyrates.z() = independent_yaw_rate;
    feedforward_bodyrates.z() = 0.0;
  }
  feedforward_bodyrates.x() =
    clamp_symmetric(feedforward_bodyrates.x(), param_.controller.max_bodyrate_x);
  feedforward_bodyrates.y() =
    clamp_symmetric(feedforward_bodyrates.y(), param_.controller.max_bodyrate_y);
  feedforward_bodyrates.z() =
    clamp_symmetric(feedforward_bodyrates.z(), param_.controller.max_bodyrate_z);
  u.bodyrates_ff = feedforward_bodyrates;
  u.bodyrates_dot_ff = clampVectorByAxis(feedforward_bodyrates_dot, kMaxBodyrateDot);
  u.bodyrates = feedforward_bodyrates + feedback_bodyrates;
  u.bodyrates.x() = clamp_symmetric(u.bodyrates.x(), param_.controller.max_bodyrate_x);
  u.bodyrates.y() = clamp_symmetric(u.bodyrates.y(), param_.controller.max_bodyrate_y);
  u.bodyrates.z() = clamp_symmetric(u.bodyrates.z(), param_.controller.max_bodyrate_z);

  u.thrust = computeDesiredCollectiveThrustSignal(thrust_acc, odom);
  if (!std::isfinite(u.thrust)) {
    u.thrust = param_.controller.min_thrust;
  }
  u.thrust = std::clamp(u.thrust, param_.controller.min_thrust, param_.controller.max_thrust);

  Eigen::Vector3d body_z = odom.q * Eigen::Vector3d::UnitZ();
  if (body_z.allFinite() && body_z.norm() > 1e-6 &&
      std::isfinite(param_.physical_control.mass_kg) && param_.physical_control.mass_kg > 0.0) {
    body_z.normalize();
    const double collective_acc = std::max(0.0, thrust_acc.dot(body_z));
    u.total_thrust_n = std::clamp(
      param_.physical_control.mass_kg * collective_acc,
      0.0,
      param_.physical_control.max_total_thrust_n);
    u.physical_setpoint_valid =
      param_.physical_control.enable && std::isfinite(u.total_thrust_n);
  }

  if (debug) {
    debug->des_p = des.p;
    debug->odom_p = odom.p;
    debug->e = e;
    debug->des_v = des.v;
    debug->odom_v = filtered_velocity_;
    debug->e_dot = e_dot;
    debug->u0 = u0;
    debug->integral_u0 = integral_u0_;
    debug->f_hat = f_hat;
    debug->u_acc = u_acc;
    debug->thrust_acc_limited = thrust_acc;
    debug->bodyrates_ff = feedforward_bodyrates;
    debug->bodyrates_dot_ff = u.bodyrates_dot_ff;
    debug->bodyrates_fb = feedback_bodyrates;
    debug->bodyrates_cmd = u.bodyrates;
    debug->thrust = u.thrust;
    debug->yaw_des = uav_utils::normalize_angle(des.yaw);
    debug->yaw_odom = yaw_odom;
    debug->yaw_error = yaw_error;
    debug->yaw_error_deadbanded = yaw_error_deadbanded;
    debug->yaw_rate_cmd_raw = yaw_rate_cmd_raw;
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
  const double gravity = finite_or(param_.controller.gravity, 9.81);
  const double nominal_hover_thrust =
    std::clamp(param_.thrust_model.hover_thrust, 0.05, 0.95);
  const double nominal_thr2acc = gravity / nominal_hover_thrust;
  const double min_collective_acc =
    std::max(param_.controller.min_thrust * nominal_thr2acc, 0.1);
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

bool LinearControl::normalizeWithSecondGrad(
  const Eigen::Vector3d &x,
  const Eigen::Vector3d &xd,
  const Eigen::Vector3d &xdd,
  Eigen::Vector3d &x_normalized,
  Eigen::Vector3d &x_normalized_dot,
  Eigen::Vector3d &x_normalized_ddot) const
{
  const double x_norm = x.norm();
  if (!std::isfinite(x_norm) || x_norm < kAlmostZeroValueThreshold) {
    x_normalized.setZero();
    x_normalized_dot.setZero();
    x_normalized_ddot.setZero();
    return false;
  }

  x_normalized = x / x_norm;
  const double norm_dot = x_normalized.dot(xd);
  x_normalized_dot =
    (xd - x_normalized * norm_dot) / x_norm;
  x_normalized_ddot =
    (xdd - x_normalized * x_normalized.dot(xdd)) / x_norm -
    (2.0 * norm_dot / x_norm) * x_normalized_dot -
    x_normalized * x_normalized_dot.squaredNorm();
  return
    x_normalized.allFinite() &&
    x_normalized_dot.allFinite() &&
    x_normalized_ddot.allFinite();
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

bool LinearControl::computeFlatInputSecondOrder(
  const Eigen::Vector3d &thrust_acc,
  const Eigen::Vector3d &jerk,
  const Eigen::Vector3d &snap,
  double yaw,
  double yaw_rate,
  double yaw_acceleration,
  const Eigen::Quaterniond &att_est,
  Eigen::Quaterniond &att,
  Eigen::Vector3d &bodyrates_ff,
  Eigen::Vector3d &bodyrates_dot_ff) const
{
  Eigen::Vector3d zb;
  Eigen::Vector3d zbd;
  Eigen::Vector3d zbdd;
  if (!normalizeWithSecondGrad(thrust_acc, jerk, snap, zb, zbd, zbdd)) {
    att = att_est;
    bodyrates_ff.setZero();
    bodyrates_dot_ff.setZero();
    return false;
  }

  const double sin_yaw = std::sin(yaw);
  const double cos_yaw = std::cos(yaw);
  const Eigen::Vector3d xc(cos_yaw, sin_yaw, 0.0);
  const Eigen::Vector3d xcd(-sin_yaw * yaw_rate, cos_yaw * yaw_rate, 0.0);
  const Eigen::Vector3d xcdd(
    -cos_yaw * yaw_rate * yaw_rate - sin_yaw * yaw_acceleration,
    -sin_yaw * yaw_rate * yaw_rate + cos_yaw * yaw_acceleration,
    0.0);
  const Eigen::Vector3d yc = zb.cross(xc);
  const Eigen::Vector3d ycd = zbd.cross(xc) + zb.cross(xcd);
  const Eigen::Vector3d ycdd =
    zbdd.cross(xc) + 2.0 * zbd.cross(xcd) + zb.cross(xcdd);

  Eigen::Vector3d yb;
  Eigen::Vector3d ybd;
  Eigen::Vector3d ybdd;
  if (!normalizeWithSecondGrad(yc, ycd, ycdd, yb, ybd, ybdd)) {
    att = att_est;
    bodyrates_ff.setZero();
    bodyrates_dot_ff.setZero();
    return false;
  }

  const Eigen::Vector3d xb = yb.cross(zb);
  const Eigen::Vector3d xbd = ybd.cross(zb) + yb.cross(zbd);
  const Eigen::Vector3d xbdd =
    ybdd.cross(zb) + 2.0 * ybd.cross(zbd) + yb.cross(zbdd);

  Eigen::Matrix3d rot;
  rot.col(0) = xb;
  rot.col(1) = yb;
  rot.col(2) = zb;
  Eigen::Matrix3d rot_dot;
  rot_dot.col(0) = xbd;
  rot_dot.col(1) = ybd;
  rot_dot.col(2) = zbd;
  Eigen::Matrix3d rot_ddot;
  rot_ddot.col(0) = xbdd;
  rot_ddot.col(1) = ybdd;
  rot_ddot.col(2) = zbdd;

  const Eigen::Matrix3d omega_hat_raw = rot.transpose() * rot_dot;
  const Eigen::Matrix3d omega_hat =
    0.5 * (omega_hat_raw - omega_hat_raw.transpose());
  const Eigen::Matrix3d omega_dot_hat_raw =
    rot.transpose() * rot_ddot - omega_hat_raw * omega_hat_raw;
  const Eigen::Matrix3d omega_dot_hat =
    0.5 * (omega_dot_hat_raw - omega_dot_hat_raw.transpose());
  bodyrates_ff = Eigen::Vector3d(
    omega_hat(2, 1),
    omega_hat(0, 2),
    omega_hat(1, 0));
  bodyrates_dot_ff = Eigen::Vector3d(
    omega_dot_hat(2, 1),
    omega_dot_hat(0, 2),
    omega_dot_hat(1, 0));

  att = Eigen::Quaterniond(rot);
  if (att.norm() > 1e-6) {
    att.normalize();
  } else {
    att = att_est;
    bodyrates_ff.setZero();
    bodyrates_dot_ff.setZero();
    return false;
  }
  return bodyrates_ff.allFinite() && bodyrates_dot_ff.allFinite();
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

Eigen::Vector3d LinearControl::computeReducedAttitudeFeedbackBodyrates(
  const Eigen::Vector3d &thrust_acc,
  const Eigen::Quaterniond &est_q) const
{
  Eigen::Vector3d z_des = thrust_acc;
  if (z_des.norm() <= kAlmostZeroValueThreshold || !z_des.allFinite()) {
    return Eigen::Vector3d::Zero();
  }
  z_des.normalize();

  Eigen::Quaterniond q = est_q;
  if (q.norm() <= 1e-6 || !q.coeffs().allFinite()) {
    return Eigen::Vector3d::Zero();
  }
  q.normalize();

  Eigen::Vector3d z_cur = q * Eigen::Vector3d::UnitZ();
  if (z_cur.norm() <= kAlmostZeroValueThreshold || !z_cur.allFinite()) {
    return Eigen::Vector3d::Zero();
  }
  z_cur.normalize();

  const Eigen::Vector3d tilt_error_world = z_cur.cross(z_des);
  const Eigen::Vector3d tilt_error_body = q.inverse() * tilt_error_world;
  const Eigen::Vector3d KAng = diagVector(param_.attitude.KAng_diag);
  return Eigen::Vector3d(
    KAng.x() * tilt_error_body.x(),
    KAng.y() * tilt_error_body.y(),
    0.0);
}

double LinearControl::computeIndependentYawRate(
  double yaw_error,
  double yaw_rate,
  double dt,
  double &yaw_error_deadbanded,
  double &yaw_rate_cmd_raw)
{
  const double deadband = std::max(0.0, param_.attitude.yaw_deadband_rad);
  if (std::abs(yaw_error) <= deadband) {
    yaw_error_deadbanded = 0.0;
  } else {
    yaw_error_deadbanded = yaw_error - std::copysign(deadband, yaw_error);
  }

  const Eigen::Vector3d KAng = diagVector(param_.attitude.KAng_diag);
  yaw_rate_cmd_raw = yaw_rate + KAng.z() * yaw_error_deadbanded;

  double yaw_rate_cmd = yaw_rate_cmd_raw;
  if (param_.attitude.yaw_rate_limit > 0.0) {
    yaw_rate_cmd = clamp_symmetric(yaw_rate_cmd, param_.attitude.yaw_rate_limit);
  }

  const double tau = param_.attitude.yaw_lpf_tau_s;
  if (tau > 0.0 && std::isfinite(tau)) {
    if (!yaw_rate_lpf_initialized_ || dt <= 0.0 || !std::isfinite(dt)) {
      yaw_rate_lpf_ = yaw_rate_cmd;
      yaw_rate_lpf_initialized_ = true;
    } else {
      const double alpha = std::clamp(dt / (tau + dt), 0.0, 1.0);
      yaw_rate_lpf_ += alpha * (yaw_rate_cmd - yaw_rate_lpf_);
    }
    yaw_rate_cmd = yaw_rate_lpf_;
  } else {
    yaw_rate_lpf_ = yaw_rate_cmd;
    yaw_rate_lpf_initialized_ = true;
  }

  return std::isfinite(yaw_rate_cmd) ? yaw_rate_cmd : 0.0;
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
