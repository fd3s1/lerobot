#ifndef PX4CTRL_INPUT_H
#define PX4CTRL_INPUT_H

#include <cstddef>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/takeoff_land.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <uav_utils/utils.h>

#include "PX4CtrlParam.h"

class RC_Data_t
{
public:
  double mode{-1.0};
  double gear{-1.0};
  double reboot_cmd{-1.0};
  double last_mode{-1.0};
  double last_gear{-1.0};
  double last_reboot_cmd{-1.0};
  bool have_init_last_mode{false};
  bool have_init_last_gear{false};
  bool have_init_last_reboot_cmd{false};
  double ch[4]{0.0, 0.0, 0.0, 0.0};

  mavros_msgs::msg::RCIn msg;
  rclcpp::Time rcv_stamp{0, 0, RCL_ROS_TIME};
  bool received{false};

  bool is_command_mode{true};
  bool enter_command_mode{false};
  bool is_hover_mode{true};
  bool enter_hover_mode{false};
  bool toggle_reboot{false};

  static constexpr double GEAR_SHIFT_VALUE = 0.75;
  static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;
  static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;
  static constexpr double DEAD_ZONE = 0.25;

  void check_validity() const;
  bool check_centered() const;
  bool check_takeoff_sticks(double throttle_max_pwm) const;
  void feed(const mavros_msgs::msg::RCIn::SharedPtr pMsg, const rclcpp::Time &now);
  bool is_received(const rclcpp::Time &now_time, double timeout_s) const;
  double channel_pwm(std::size_t one_based_channel, double default_value = 1500.0) const;
};

class Odom_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Vector3d w{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};

  nav_msgs::msg::Odometry msg;
  rclcpp::Time rcv_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time msg_stamp{0, 0, RCL_ROS_TIME};
  bool stamp_from_receive_time{false};
  bool recv_new_msg{false};
  bool received{false};

  void feed(const nav_msgs::msg::Odometry::SharedPtr pMsg, const rclcpp::Time &now);
  bool is_received(const rclcpp::Time &now_time, double timeout_s) const;
};

class MocapPose_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p{Eigen::Vector3d::Zero()};

  geometry_msgs::msg::PoseStamped msg;
  rclcpp::Time rcv_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time msg_stamp{0, 0, RCL_ROS_TIME};
  bool stamp_from_receive_time{false};
  bool received{false};

  void feed(const geometry_msgs::msg::PoseStamped::SharedPtr pMsg, const rclcpp::Time &now);
  bool is_received(const rclcpp::Time &now_time, double timeout_s) const;
};

class MocapTwist_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d v{Eigen::Vector3d::Zero()};

  geometry_msgs::msg::TwistStamped msg;
  rclcpp::Time rcv_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time msg_stamp{0, 0, RCL_ROS_TIME};
  bool stamp_from_receive_time{false};
  bool received{false};

  void feed(const geometry_msgs::msg::TwistStamped::SharedPtr pMsg, const rclcpp::Time &now);
  bool is_received(const rclcpp::Time &now_time, double timeout_s) const;
};

class State_Data_t
{
public:
  mavros_msgs::msg::State current_state;
  mavros_msgs::msg::State state_before_offboard;

  void feed(const mavros_msgs::msg::State::SharedPtr pMsg);
};

class ExtendedState_Data_t
{
public:
  mavros_msgs::msg::ExtendedState current_extended_state;

  void feed(const mavros_msgs::msg::ExtendedState::SharedPtr pMsg);
};

class Imu_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d w{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};

  sensor_msgs::msg::Imu msg;
  rclcpp::Time rcv_stamp{0, 0, RCL_ROS_TIME};
  bool received{false};

  void feed(const sensor_msgs::msg::Imu::SharedPtr pMsg, const rclcpp::Time &now);
  bool is_received(const rclcpp::Time &now_time, double timeout_s) const;
};

class Command_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};
  Eigen::Vector3d j{Eigen::Vector3d::Zero()};
  Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_v{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  double yaw{0.0};
  double yaw_rate{0.0};
  double yaw_acceleration{0.0};

  geometry_msgs::msg::PoseStamped msg;
  quadrotor_msgs::msg::PositionCommand traj_msg;
  rclcpp::Time rcv_stamp{0, 0, RCL_ROS_TIME};
  bool received{false};

  void feed(const geometry_msgs::msg::PoseStamped::SharedPtr pMsg, const rclcpp::Time &now);
  void feed(const quadrotor_msgs::msg::PositionCommand::SharedPtr pMsg, const rclcpp::Time &now);
  bool is_received(const rclcpp::Time &now_time, double timeout_s) const;
};

class Battery_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double volt{0.0};
  double percentage{0.0};

  sensor_msgs::msg::BatteryState msg;
  rclcpp::Time rcv_stamp{0, 0, RCL_ROS_TIME};
  bool received{false};

  void feed(const sensor_msgs::msg::BatteryState::SharedPtr pMsg, const rclcpp::Time &now);
  bool is_received(const rclcpp::Time &now_time, double timeout_s) const;
};

class Takeoff_Land_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool triggered{false};
  uint8_t takeoff_land_cmd{0};

  quadrotor_msgs::msg::TakeoffLand msg;
  rclcpp::Time rcv_stamp{0, 0, RCL_ROS_TIME};

  void feed(const quadrotor_msgs::msg::TakeoffLand::SharedPtr pMsg, const rclcpp::Time &now);
};

#endif
