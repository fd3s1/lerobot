#include "input.h"

#include <algorithm>
#include <cmath>

namespace {

double rc_axis_from_pwm(double pwm)
{
  double value = (pwm - 1500.0) / 500.0;
  value = std::clamp(value, -1.0, 1.0);
  if (value > RC_Data_t::DEAD_ZONE) {
    return (value - RC_Data_t::DEAD_ZONE) / (1.0 - RC_Data_t::DEAD_ZONE);
  }
  if (value < -RC_Data_t::DEAD_ZONE) {
    return (value + RC_Data_t::DEAD_ZONE) / (1.0 - RC_Data_t::DEAD_ZONE);
  }
  return 0.0;
}

double switch_from_pwm(double pwm)
{
  return std::clamp((pwm - 1000.0) / 1000.0, -0.2, 1.2);
}

bool stamp_is_zero(const builtin_interfaces::msg::Time &stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0;
}

rclcpp::Time message_stamp_or_receive_time(
  const builtin_interfaces::msg::Time &stamp,
  const rclcpp::Time &receive_time,
  bool &used_receive_time)
{
  used_receive_time = stamp_is_zero(stamp);
  if (used_receive_time) {
    return receive_time;
  }
  return rclcpp::Time(stamp, RCL_ROS_TIME);
}

}  // namespace

void RC_Data_t::feed(const mavros_msgs::msg::RCIn::SharedPtr pMsg, const rclcpp::Time &now)
{
  msg = *pMsg;
  rcv_stamp = now;
  received = true;

  for (int i = 0; i < 4; ++i) {
    ch[i] = rc_axis_from_pwm(channel_pwm(static_cast<std::size_t>(i + 1)));
  }

  mode = switch_from_pwm(channel_pwm(5));
  gear = switch_from_pwm(channel_pwm(6));
  reboot_cmd = switch_from_pwm(channel_pwm(8));

  check_validity();

  if (!have_init_last_mode) {
    have_init_last_mode = true;
    last_mode = mode;
  }
  if (!have_init_last_gear) {
    have_init_last_gear = true;
    last_gear = gear;
  }
  if (!have_init_last_reboot_cmd) {
    have_init_last_reboot_cmd = true;
    last_reboot_cmd = reboot_cmd;
  }

  enter_hover_mode = last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE;
  is_hover_mode = mode > API_MODE_THRESHOLD_VALUE;

  if (is_hover_mode) {
    if (last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE) {
      enter_command_mode = true;
    } else if (gear < GEAR_SHIFT_VALUE) {
      enter_command_mode = false;
    }

    is_command_mode = gear > GEAR_SHIFT_VALUE;
  }

  if (!is_hover_mode && !is_command_mode) {
    toggle_reboot = last_reboot_cmd < REBOOT_THRESHOLD_VALUE && reboot_cmd > REBOOT_THRESHOLD_VALUE;
  } else {
    toggle_reboot = false;
  }

  last_mode = mode;
  last_gear = gear;
  last_reboot_cmd = reboot_cmd;
}

void RC_Data_t::check_validity() const
{
  if (mode < -1.1 || mode > 1.1 || gear < -1.1 || gear > 1.1 ||
      reboot_cmd < -1.1 || reboot_cmd > 1.1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("px4ctrl"),
      "RC data validity check failed. mode=%f, gear=%f, reboot_cmd=%f",
      mode,
      gear,
      reboot_cmd);
  }
}

bool RC_Data_t::check_centered() const
{
  return std::abs(ch[0]) < 1e-5 && std::abs(ch[1]) < 1e-5 &&
         std::abs(ch[2]) < 1e-5 && std::abs(ch[3]) < 1e-5;
}

bool RC_Data_t::check_takeoff_sticks(double throttle_max_pwm) const
{
  const bool attitude_sticks_centered =
    std::abs(ch[0]) < 1e-5 && std::abs(ch[1]) < 1e-5 && std::abs(ch[3]) < 1e-5;
  const bool throttle_check_disabled = throttle_max_pwm <= 0.0;
  const bool throttle_low = throttle_check_disabled || channel_pwm(3) <= throttle_max_pwm;
  return attitude_sticks_centered && throttle_low;
}

bool RC_Data_t::is_received(const rclcpp::Time &now_time, double timeout_s) const
{
  return received && (now_time - rcv_stamp).seconds() < timeout_s;
}

double RC_Data_t::channel_pwm(std::size_t one_based_channel, double default_value) const
{
  if (one_based_channel == 0) {
    return default_value;
  }
  const std::size_t index = one_based_channel - 1;
  if (index >= msg.channels.size()) {
    return default_value;
  }
  return static_cast<double>(msg.channels[index]);
}

void Odom_Data_t::feed(
  const nav_msgs::msg::Odometry::SharedPtr pMsg,
  const rclcpp::Time &now)
{
  msg = *pMsg;
  msg_stamp = message_stamp_or_receive_time(msg.header.stamp, now, stamp_from_receive_time);
  uav_utils::extract_odometry(msg, p, v, q, w);
  if (q.norm() > 1e-6) {
    q.normalize();
  } else {
    q.setIdentity();
  }

  // Odometry twist is expressed in child_frame_id; use world/ENU velocity downstream.
  v = q * v;

  rcv_stamp = now;
  recv_new_msg = true;
  received = true;
}

bool Odom_Data_t::is_received(const rclcpp::Time &now_time, double timeout_s) const
{
  return received && (now_time - rcv_stamp).seconds() < timeout_s;
}

void MocapPose_Data_t::feed(
  const geometry_msgs::msg::PoseStamped::SharedPtr pMsg,
  const rclcpp::Time &now)
{
  msg = *pMsg;
  msg_stamp = message_stamp_or_receive_time(msg.header.stamp, now, stamp_from_receive_time);
  p = Eigen::Vector3d(
    msg.pose.position.x,
    msg.pose.position.y,
    msg.pose.position.z);
  rcv_stamp = now;
  received = true;
}

bool MocapPose_Data_t::is_received(const rclcpp::Time &now_time, double timeout_s) const
{
  return received && (now_time - rcv_stamp).seconds() < timeout_s;
}

void MocapTwist_Data_t::feed(
  const geometry_msgs::msg::TwistStamped::SharedPtr pMsg,
  const rclcpp::Time &now)
{
  msg = *pMsg;
  msg_stamp = message_stamp_or_receive_time(msg.header.stamp, now, stamp_from_receive_time);
  v = Eigen::Vector3d(
    msg.twist.linear.x,
    msg.twist.linear.y,
    msg.twist.linear.z);
  rcv_stamp = now;
  received = true;
}

bool MocapTwist_Data_t::is_received(const rclcpp::Time &now_time, double timeout_s) const
{
  return received && (now_time - rcv_stamp).seconds() < timeout_s;
}

void State_Data_t::feed(const mavros_msgs::msg::State::SharedPtr pMsg)
{
  current_state = *pMsg;
}

void ExtendedState_Data_t::feed(const mavros_msgs::msg::ExtendedState::SharedPtr pMsg)
{
  current_extended_state = *pMsg;
}

void Imu_Data_t::feed(
  const sensor_msgs::msg::Imu::SharedPtr pMsg,
  const rclcpp::Time &now)
{
  msg = *pMsg;
  rcv_stamp = now;
  received = true;

  q.w() = msg.orientation.w;
  q.x() = msg.orientation.x;
  q.y() = msg.orientation.y;
  q.z() = msg.orientation.z;
  if (q.norm() > 1e-6) {
    q.normalize();
  } else {
    q.setIdentity();
  }

  w = Eigen::Vector3d(
    msg.angular_velocity.x,
    msg.angular_velocity.y,
    msg.angular_velocity.z);
  a = Eigen::Vector3d(
    msg.linear_acceleration.x,
    msg.linear_acceleration.y,
    msg.linear_acceleration.z);
}

bool Imu_Data_t::is_received(const rclcpp::Time &now_time, double timeout_s) const
{
  return received && (now_time - rcv_stamp).seconds() < timeout_s;
}

void Command_Data_t::feed(
  const geometry_msgs::msg::PoseStamped::SharedPtr pMsg,
  const rclcpp::Time &now)
{
  const bool had_previous = received;
  const Eigen::Vector3d previous_p = p;
  const Eigen::Vector3d previous_v = v;
  const rclcpp::Time previous_stamp = rcv_stamp;

  msg = *pMsg;
  uav_utils::extract_odometry(msg, p, q);
  yaw = uav_utils::normalize_angle(uav_utils::get_yaw_from_quaternion(q));
  yaw_rate = 0.0;
  yaw_acceleration = 0.0;
  j.setZero();
  snap.setZero();

  if (had_previous) {
    const double dt = (now - previous_stamp).seconds();
    if (dt > 1e-3 && dt < 1.0) {
      v = (p - previous_p) / dt;
      a = (v - previous_v) / dt;
    } else {
      v.setZero();
      a.setZero();
    }
  } else {
    v.setZero();
    a.setZero();
  }
  last_p = previous_p;
  last_v = previous_v;
  rcv_stamp = now;
  received = true;
}

void Command_Data_t::feed(
  const quadrotor_msgs::msg::PositionCommand::SharedPtr pMsg,
  const rclcpp::Time &now)
{
  const Eigen::Vector3d previous_p = p;
  const Eigen::Vector3d previous_v = v;

  traj_msg = *pMsg;
  p = Eigen::Vector3d(
    pMsg->position.x,
    pMsg->position.y,
    pMsg->position.z);
  v = Eigen::Vector3d(
    pMsg->velocity.x,
    pMsg->velocity.y,
    pMsg->velocity.z);
  a = Eigen::Vector3d(
    pMsg->acceleration.x,
    pMsg->acceleration.y,
    pMsg->acceleration.z);
  j = Eigen::Vector3d(
    pMsg->jerk.x,
    pMsg->jerk.y,
    pMsg->jerk.z);
  snap = Eigen::Vector3d(
    pMsg->snap.x,
    pMsg->snap.y,
    pMsg->snap.z);
  yaw = uav_utils::normalize_angle(pMsg->yaw);
  yaw_rate = pMsg->yaw_dot;
  yaw_acceleration = pMsg->yaw_ddot;
  q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  last_p = previous_p;
  last_v = previous_v;

  msg.header = pMsg->header;
  msg.pose.position.x = p.x();
  msg.pose.position.y = p.y();
  msg.pose.position.z = p.z();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();

  rcv_stamp = now;
  received = true;
}

bool Command_Data_t::is_received(const rclcpp::Time &now_time, double timeout_s) const
{
  return received && (now_time - rcv_stamp).seconds() < timeout_s;
}

void Battery_Data_t::feed(
  const sensor_msgs::msg::BatteryState::SharedPtr pMsg,
  const rclcpp::Time &now)
{
  msg = *pMsg;
  rcv_stamp = now;
  received = true;

  double voltage = 0.0;
  for (const auto cell_voltage : pMsg->cell_voltage) {
    voltage += cell_voltage;
  }
  if (voltage <= 0.0) {
    voltage = pMsg->voltage;
  }
  volt = 0.8 * volt + 0.2 * voltage;
  percentage = pMsg->percentage;
}

bool Battery_Data_t::is_received(const rclcpp::Time &now_time, double timeout_s) const
{
  return received && (now_time - rcv_stamp).seconds() < timeout_s;
}

void Takeoff_Land_Data_t::feed(
  const quadrotor_msgs::msg::TakeoffLand::SharedPtr pMsg,
  const rclcpp::Time &now)
{
  msg = *pMsg;
  rcv_stamp = now;
  triggered = true;
  takeoff_land_cmd = pMsg->takeoff_land_cmd;
}
