#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

double deg2rad(double deg)
{
  return deg * kPi / 180.0;
}

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double clamp_symmetric(double value, double limit)
{
  return std::clamp(value, -std::abs(limit), std::abs(limit));
}

double slew_toward(double current, double target, double max_delta)
{
  if (target > current + max_delta) {
    return current + max_delta;
  }
  if (target < current - max_delta) {
    return current - max_delta;
  }
  return target;
}

Eigen::Quaterniond eigen_quaternion_from_msg(const geometry_msgs::msg::Quaternion &q_msg)
{
  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  if (q.norm() > 1e-6) {
    q.normalize();
  } else {
    q.setIdentity();
  }
  return q;
}

Eigen::Vector3d rpy_from_quaternion(const Eigen::Quaterniond &q)
{
  const double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
  const double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  const double pitch = std::abs(sinp) >= 1.0 ?
    std::copysign(kPi / 2.0, sinp) :
    std::asin(sinp);

  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  const double yaw = std::atan2(siny_cosp, cosy_cosp);
  return Eigen::Vector3d(roll, pitch, normalize_angle(yaw));
}

Eigen::Vector3d rpy_from_quaternion(const geometry_msgs::msg::Quaternion &q_msg)
{
  return rpy_from_quaternion(eigen_quaternion_from_msg(q_msg));
}

Eigen::Quaterniond eigen_quaternion_from_rpy(const Eigen::Vector3d &rpy)
{
  const Eigen::AngleAxisd roll_angle(rpy.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(rpy.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(rpy.z(), Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
  q.normalize();
  return q;
}

geometry_msgs::msg::Quaternion quaternion_msg_from_eigen(const Eigen::Quaterniond &q_in)
{
  Eigen::Quaterniond q = q_in;
  if (q.norm() > 1e-6) {
    q.normalize();
  } else {
    q.setIdentity();
  }
  geometry_msgs::msg::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

Eigen::Quaterniond integrate_bodyrate(
  const Eigen::Quaterniond &q_current,
  const Eigen::Vector3d &bodyrate,
  double dt)
{
  const double angle = bodyrate.norm() * std::max(0.0, dt);
  if (angle < 1e-9) {
    return q_current.normalized();
  }
  const Eigen::Vector3d axis = bodyrate.normalized();
  Eigen::Quaterniond q_next = q_current * Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
  q_next.normalize();
  return q_next;
}

double channel_pwm(const mavros_msgs::msg::RCIn &rc, int one_based_channel, double default_value)
{
  if (one_based_channel <= 0) {
    return default_value;
  }
  const std::size_t index = static_cast<std::size_t>(one_based_channel - 1);
  if (index >= rc.channels.size()) {
    return default_value;
  }
  return static_cast<double>(rc.channels[index]);
}

double switch_from_pwm(double pwm)
{
  return std::clamp((pwm - 1000.0) / 1000.0, -0.2, 1.2);
}

double raw_axis_from_pwm(double pwm, bool reverse)
{
  double value = std::clamp((pwm - 1500.0) / 500.0, -1.0, 1.0);
  if (reverse) {
    value = -value;
  }
  return value;
}

double shape_axis(double raw, double deadzone, double expo)
{
  const double dz = std::clamp(deadzone, 0.0, 0.95);
  const double exponent = std::max(0.1, expo);
  const double magnitude = std::abs(raw);
  if (magnitude <= dz) {
    return 0.0;
  }
  const double normalized = std::clamp((magnitude - dz) / (1.0 - dz), 0.0, 1.0);
  return std::copysign(std::pow(normalized, exponent), raw);
}

geometry_msgs::msg::Vector3Stamped make_vector_msg(
  const rclcpp::Time &stamp,
  const std::string &frame_id,
  const Eigen::Vector3d &value)
{
  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.vector.x = value.x();
  msg.vector.y = value.y();
  msg.vector.z = value.z();
  return msg;
}

}  // namespace

class TestAttitudeModeNode : public rclcpp::Node
{
public:
  TestAttitudeModeNode()
  : Node("test_attitude_mode_node")
  {
    rc_topic_ = declare_parameter<std::string>("rc_topic", "/mavros/rc/in");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/mavros/imu/data");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/mavros/local_position/odom");
    state_topic_ = declare_parameter<std::string>("state_topic", "/mavros/state");
    setpoint_topic_ =
      declare_parameter<std::string>("setpoint_topic", "/mavros/setpoint_raw/attitude");
    set_mode_service_ = declare_parameter<std::string>("set_mode_service", "/mavros/set_mode");
    arming_service_ = declare_parameter<std::string>("arming_service", "/mavros/cmd/arming");

    reference_rpy_topic_ =
      declare_parameter<std::string>("reference_rpy_topic", "/test_att/reference_rpy");
    desired_rpy_topic_ = declare_parameter<std::string>("desired_rpy_topic", "/test_att/desired_rpy");
    actual_rpy_topic_ = declare_parameter<std::string>("actual_rpy_topic", "/test_att/actual_rpy");
    error_rpy_topic_ = declare_parameter<std::string>("error_rpy_topic", "/test_att/error_rpy");
    reference_bodyrate_topic_ =
      declare_parameter<std::string>("reference_bodyrate_topic", "/test_att/reference_bodyrate");
    actual_bodyrate_topic_ =
      declare_parameter<std::string>("actual_bodyrate_topic", "/test_att/actual_bodyrate");
    error_bodyrate_topic_ =
      declare_parameter<std::string>("error_bodyrate_topic", "/test_att/error_bodyrate");
    thrust_topic_ = declare_parameter<std::string>("thrust_topic", "/test_att/thrust");
    status_topic_ = declare_parameter<std::string>("status_topic", "/test_att/status");

    rate_hz_ = declare_parameter<double>("rate_hz", 100.0);
    test_att_channel_ = declare_parameter<int>("test_att_channel", 11);
    active_threshold_ = declare_parameter<double>("active_threshold", 0.75);
    rc_timeout_s_ = declare_parameter<double>("rc_timeout_s", 0.3);
    imu_timeout_s_ = declare_parameter<double>("imu_timeout_s", 0.3);
    odom_timeout_s_ = declare_parameter<double>("odom_timeout_s", 0.3);
    state_timeout_s_ = declare_parameter<double>("state_timeout_s", 1.0);
    auto_arm_enable_ = declare_parameter<bool>("auto_arm_enable", true);
    auto_offboard_enable_ = declare_parameter<bool>("auto_offboard_enable", true);
    require_offboard_for_auto_arm_ = declare_parameter<bool>("require_offboard_for_auto_arm", true);
    restore_mode_on_inactive_ = declare_parameter<bool>("restore_mode_on_inactive", true);
    disarm_on_inactive_ = declare_parameter<bool>("disarm_on_inactive", false);
    enter_confirm_enable_ = declare_parameter<bool>("enter_confirm_enable", true);
    arm_request_delay_s_ = declare_parameter<double>("arm_request_delay_s", 0.2);
    arm_request_period_s_ = declare_parameter<double>("arm_request_period_s", 1.0);
    offboard_request_delay_s_ = declare_parameter<double>("offboard_request_delay_s", 0.5);
    offboard_request_period_s_ = declare_parameter<double>("offboard_request_period_s", 1.0);

    stick_deadzone_ = declare_parameter<double>("stick_deadzone", 0.08);
    stick_expo_ = declare_parameter<double>("stick_expo", 1.7);
    roll_reverse_ = declare_parameter<bool>("roll_reverse", false);
    pitch_reverse_ = declare_parameter<bool>("pitch_reverse", false);
    yaw_reverse_ = declare_parameter<bool>("yaw_reverse", true);
    throttle_reverse_ = declare_parameter<bool>("throttle_reverse", false);

    max_roll_rate_ = deg2rad(declare_parameter<double>("max_roll_rate_dps", 45.0));
    max_pitch_rate_ = deg2rad(declare_parameter<double>("max_pitch_rate_dps", 45.0));
    max_yaw_rate_ = deg2rad(declare_parameter<double>("max_yaw_rate_dps", 60.0));
    max_roll_ = deg2rad(declare_parameter<double>("max_roll_deg", 35.0));
    max_pitch_ = deg2rad(declare_parameter<double>("max_pitch_deg", 35.0));
    attitude_kang_ = Eigen::Vector3d(
      declare_parameter<double>("attitude_kang_roll", 6.0),
      declare_parameter<double>("attitude_kang_pitch", 6.0),
      declare_parameter<double>("attitude_kang_yaw", 3.0));
    max_bodyrate_cmd_ = Eigen::Vector3d(
      deg2rad(declare_parameter<double>("max_bodyrate_roll_dps", 143.0)),
      deg2rad(declare_parameter<double>("max_bodyrate_pitch_dps", 143.0)),
      deg2rad(declare_parameter<double>("max_bodyrate_yaw_dps", 86.0)));

    thrust_base_ = declare_parameter<double>("thrust_base", 0.35);
    thrust_min_ = declare_parameter<double>("thrust_min", 0.20);
    thrust_max_ = declare_parameter<double>("thrust_max", 0.45);
    thrust_ramp_per_s_ = declare_parameter<double>("thrust_ramp_per_s", 0.05);
    thrust_slew_per_s_ = declare_parameter<double>("thrust_slew_per_s", 0.20);
    require_armed_for_thrust_ramp_ = declare_parameter<bool>("require_armed_for_thrust_ramp", true);
    require_offboard_for_thrust_ramp_ =
      declare_parameter<bool>("require_offboard_for_thrust_ramp", true);
    setpoint_output_mode_ =
      declare_parameter<std::string>("setpoint_output_mode", "attitude");
    setpoint_alignment_mode_ =
      declare_parameter<std::string>("setpoint_alignment_mode", "direct_imu");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    sanitize_parameters();

    rc_sub_ = create_subscription<mavros_msgs::msg::RCIn>(
      rc_topic_,
      rclcpp::SensorDataQoS(),
      [this](const mavros_msgs::msg::RCIn::SharedPtr msg) {
        rc_msg_ = *msg;
        rc_stamp_ = now();
        have_rc_ = true;
      });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_msg_ = *msg;
        imu_stamp_ = now();
        have_imu_ = true;
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_msg_ = *msg;
        odom_stamp_ = now();
        have_odom_ = true;
      });

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      state_topic_,
      rclcpp::SensorDataQoS(),
      [this](const mavros_msgs::msg::State::SharedPtr msg) {
        state_msg_ = *msg;
        state_stamp_ = now();
        have_state_ = true;
      });

    setpoint_pub_ = create_publisher<mavros_msgs::msg::AttitudeTarget>(setpoint_topic_, 10);
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>(set_mode_service_);
    arming_client_ = create_client<mavros_msgs::srv::CommandBool>(arming_service_);
    reference_rpy_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(reference_rpy_topic_, 10);
    desired_rpy_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(desired_rpy_topic_, 10);
    actual_rpy_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(actual_rpy_topic_, 10);
    error_rpy_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(error_rpy_topic_, 10);
    reference_bodyrate_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(reference_bodyrate_topic_, 10);
    actual_bodyrate_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(actual_bodyrate_topic_, 10);
    error_bodyrate_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(error_bodyrate_topic_, 10);
    thrust_pub_ = create_publisher<std_msgs::msg::Float64>(thrust_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    current_thrust_ = thrust_min_;
    nominal_thrust_ = thrust_min_;
    if (enter_confirm_enable_) {
      start_enter_confirm_thread();
    } else {
      enter_confirmed_.store(true);
    }
    const double period_s = 1.0 / std::max(1.0, rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_s)),
      std::bind(&TestAttitudeModeNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "test_attitude_mode_node started. CH%d high + Enter enables setpoint/OFFBOARD/arm. rc=%s imu=%s odom=%s state=%s setpoint=%s set_mode=%s arming=%s",
      test_att_channel_,
      rc_topic_.c_str(),
      imu_topic_.c_str(),
      odom_topic_.c_str(),
      state_topic_.c_str(),
      setpoint_topic_.c_str(),
      set_mode_service_.c_str(),
      arming_service_.c_str());
  }

private:
  void start_enter_confirm_thread()
  {
    std::thread([this]() {
      while (rclcpp::ok()) {
        RCLCPP_WARN(
          get_logger(),
          "CH%d high is armed by terminal confirmation. Press Enter to allow test_att automation.",
          test_att_channel_);
        std::string line;
        if (!std::getline(std::cin, line)) {
          RCLCPP_WARN(get_logger(), "stdin closed; Enter confirmation disabled for this run.");
          return;
        }
        enter_confirmed_.store(true);
        RCLCPP_WARN(get_logger(), "Enter confirmed. Raise CH%d to start test_att.", test_att_channel_);
        while (rclcpp::ok() && enter_confirmed_.load()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    }).detach();
  }

  void sanitize_parameters()
  {
    if (setpoint_output_mode_ != "attitude" &&
        setpoint_output_mode_ != "bodyrate" &&
        setpoint_output_mode_ != "attitude_bodyrate") {
      RCLCPP_WARN(
        get_logger(),
        "unknown setpoint_output_mode='%s'; using attitude",
        setpoint_output_mode_.c_str());
      setpoint_output_mode_ = "attitude";
    }
    if (setpoint_alignment_mode_ != "direct_imu" &&
        setpoint_alignment_mode_ != "px4ctrl_align" &&
        setpoint_alignment_mode_ != "direct_odom") {
      RCLCPP_WARN(
        get_logger(),
        "unknown setpoint_alignment_mode='%s'; using direct_imu",
        setpoint_alignment_mode_.c_str());
      setpoint_alignment_mode_ = "direct_imu";
    }
    if (thrust_min_ < 0.0) {
      RCLCPP_WARN(get_logger(), "thrust_min %.3f < 0.0; clamping to 0.0", thrust_min_);
      thrust_min_ = 0.0;
    }
    if (thrust_max_ > 1.0) {
      RCLCPP_WARN(get_logger(), "thrust_max %.3f > 1.0; clamping to 1.0", thrust_max_);
      thrust_max_ = 1.0;
    }
    if (thrust_min_ > thrust_max_) {
      RCLCPP_WARN(get_logger(), "thrust_min > thrust_max; resetting to [0.20, 0.45]");
      thrust_min_ = 0.20;
      thrust_max_ = 0.45;
    }
    if (thrust_base_ < thrust_min_ || thrust_base_ > thrust_max_) {
      RCLCPP_WARN(
        get_logger(),
        "thrust_base %.3f outside [%.3f, %.3f]; clamping",
        thrust_base_,
        thrust_min_,
        thrust_max_);
      thrust_base_ = std::clamp(thrust_base_, thrust_min_, thrust_max_);
    }
    rate_hz_ = std::max(1.0, rate_hz_);
    rc_timeout_s_ = std::max(0.05, rc_timeout_s_);
    imu_timeout_s_ = std::max(0.05, imu_timeout_s_);
    odom_timeout_s_ = std::max(0.05, odom_timeout_s_);
    state_timeout_s_ = std::max(0.05, state_timeout_s_);
    arm_request_delay_s_ = std::max(0.0, arm_request_delay_s_);
    arm_request_period_s_ = std::max(0.1, arm_request_period_s_);
    offboard_request_delay_s_ = std::max(0.0, offboard_request_delay_s_);
    offboard_request_period_s_ = std::max(0.1, offboard_request_period_s_);
    thrust_ramp_per_s_ = std::max(0.0, thrust_ramp_per_s_);
    thrust_slew_per_s_ = std::max(0.0, thrust_slew_per_s_);
  }

  bool rc_fresh(const rclcpp::Time &stamp) const
  {
    return have_rc_ && (stamp - rc_stamp_).seconds() <= rc_timeout_s_;
  }

  bool imu_fresh(const rclcpp::Time &stamp) const
  {
    return have_imu_ && (stamp - imu_stamp_).seconds() <= imu_timeout_s_;
  }

  bool odom_fresh(const rclcpp::Time &stamp) const
  {
    return have_odom_ && (stamp - odom_stamp_).seconds() <= odom_timeout_s_;
  }

  bool odom_required_for_setpoint() const
  {
    if (setpoint_output_mode_ == "bodyrate" ||
        (setpoint_output_mode_ == "attitude_bodyrate" &&
         setpoint_alignment_mode_ == "direct_imu")) {
      return false;
    }
    return setpoint_alignment_mode_ == "px4ctrl_align" ||
           setpoint_alignment_mode_ == "direct_odom";
  }

  bool state_fresh(const rclcpp::Time &stamp) const
  {
    return have_state_ && (stamp - state_stamp_).seconds() <= state_timeout_s_;
  }

  double rc_age(const rclcpp::Time &stamp) const
  {
    return have_rc_ ? (stamp - rc_stamp_).seconds() : -1.0;
  }

  double imu_age(const rclcpp::Time &stamp) const
  {
    return have_imu_ ? (stamp - imu_stamp_).seconds() : -1.0;
  }

  double odom_age(const rclcpp::Time &stamp) const
  {
    return have_odom_ ? (stamp - odom_stamp_).seconds() : -1.0;
  }

  double state_age(const rclcpp::Time &stamp) const
  {
    return have_state_ ? (stamp - state_stamp_).seconds() : -1.0;
  }

  bool is_offboard() const
  {
    return have_state_ && state_msg_.mode == "OFFBOARD";
  }

  void request_mode(const std::string &mode, const std::string &reason)
  {
    if (!set_mode_client_ || !set_mode_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "set_mode service is not ready; cannot request %s (%s)",
        mode.c_str(),
        reason.c_str());
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    set_mode_client_->async_send_request(request);
    RCLCPP_WARN(
      get_logger(),
      "requested FCU mode %s (%s)",
      mode.c_str(),
      reason.c_str());
  }

  void request_arm(bool arm, const std::string &reason)
  {
    if (!arming_client_ || !arming_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "arming service is not ready; cannot request armed=%s (%s)",
        arm ? "true" : "false",
        reason.c_str());
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    arming_client_->async_send_request(request);
    RCLCPP_WARN(
      get_logger(),
      "requested FCU armed=%s (%s)",
      arm ? "true" : "false",
      reason.c_str());
  }

  void maybe_request_offboard(const rclcpp::Time &stamp, bool active, bool state_ok)
  {
    if (!auto_offboard_enable_ || !active || !state_ok || is_offboard()) {
      return;
    }

    if (!have_active_start_) {
      active_start_stamp_ = stamp;
      have_active_start_ = true;
    }

    if ((stamp - active_start_stamp_).seconds() < offboard_request_delay_s_) {
      return;
    }
    if (have_last_offboard_request_ &&
        (stamp - last_offboard_request_stamp_).seconds() < offboard_request_period_s_) {
      return;
    }

    if (!have_mode_before_offboard_ && !state_msg_.mode.empty() && state_msg_.mode != "OFFBOARD") {
      mode_before_offboard_ = state_msg_.mode;
      have_mode_before_offboard_ = true;
    }
    request_mode("OFFBOARD", "CH11 high");
    last_offboard_request_stamp_ = stamp;
    have_last_offboard_request_ = true;
  }

  void maybe_request_arm(const rclcpp::Time &stamp, bool active, bool state_ok)
  {
    if (!auto_arm_enable_ || !active || !state_ok || state_msg_.armed) {
      return;
    }
    if (require_offboard_for_auto_arm_ && !is_offboard()) {
      return;
    }

    if (!have_active_start_) {
      active_start_stamp_ = stamp;
      have_active_start_ = true;
    }

    if ((stamp - active_start_stamp_).seconds() < arm_request_delay_s_) {
      return;
    }
    if (have_last_arm_request_ &&
        (stamp - last_arm_request_stamp_).seconds() < arm_request_period_s_) {
      return;
    }

    request_arm(true, "test_att confirmed");
    last_arm_request_stamp_ = stamp;
    have_last_arm_request_ = true;
  }

  void maybe_restore_mode(const rclcpp::Time &stamp)
  {
    if (!restore_mode_on_inactive_ || !state_fresh(stamp) || !is_offboard()) {
      return;
    }
    const std::string restore_mode = have_mode_before_offboard_ && !mode_before_offboard_.empty() ?
      mode_before_offboard_ :
      std::string("MANUAL");
    request_mode(restore_mode, "CH11 low");
  }

  void maybe_disarm_on_inactive(const rclcpp::Time &stamp)
  {
    if (!disarm_on_inactive_ || !state_fresh(stamp) || !state_msg_.armed) {
      return;
    }
    request_arm(false, "test_att inactive");
  }

  bool test_att_switch_high() const
  {
    const double pwm = channel_pwm(rc_msg_, test_att_channel_, 1000.0);
    return switch_from_pwm(pwm) > active_threshold_;
  }

  double shaped_channel(int channel, bool reverse) const
  {
    const double raw = raw_axis_from_pwm(channel_pwm(rc_msg_, channel, 1500.0), reverse);
    return shape_axis(raw, stick_deadzone_, stick_expo_);
  }

  double target_thrust_from_rc() const
  {
    const double throttle = shaped_channel(3, throttle_reverse_);
    if (throttle >= 0.0) {
      return nominal_thrust_ + throttle * (thrust_max_ - nominal_thrust_);
    }
    return nominal_thrust_ + throttle * (nominal_thrust_ - thrust_min_);
  }

  bool thrust_ramp_allowed() const
  {
    if (require_armed_for_thrust_ramp_ && (!have_state_ || !state_msg_.armed)) {
      return false;
    }
    if (require_offboard_for_thrust_ramp_ && !is_offboard()) {
      return false;
    }
    return true;
  }

  void reset_automation_requests()
  {
    have_last_arm_request_ = false;
    have_last_offboard_request_ = false;
    have_last_setpoint_ = false;
  }

  void reset_thrust_ramp()
  {
    nominal_thrust_ = thrust_min_;
    current_thrust_ = thrust_min_;
  }

  Eigen::Vector3d actual_bodyrate_from_imu() const
  {
    return Eigen::Vector3d(
      imu_msg_.angular_velocity.x,
      imu_msg_.angular_velocity.y,
      imu_msg_.angular_velocity.z);
  }

  Eigen::Quaterniond imu_quaternion() const
  {
    return eigen_quaternion_from_msg(imu_msg_.orientation);
  }

  Eigen::Quaterniond odom_quaternion() const
  {
    return eigen_quaternion_from_msg(odom_msg_.pose.pose.orientation);
  }

  Eigen::Quaterniond actual_attitude_quaternion() const
  {
    if (setpoint_alignment_mode_ == "direct_odom") {
      return odom_quaternion();
    }
    return imu_quaternion();
  }

  Eigen::Quaterniond reference_seed_quaternion() const
  {
    if (setpoint_alignment_mode_ == "direct_odom") {
      return odom_quaternion();
    }
    return imu_quaternion();
  }

  Eigen::Quaterniond setpoint_quaternion_from_reference() const
  {
    if (setpoint_alignment_mode_ == "px4ctrl_align") {
      Eigen::Quaterniond q = imu_quaternion() * odom_quaternion().inverse() * reference_q_;
      if (q.norm() > 1e-6) {
        q.normalize();
        return q;
      }
      return reference_q_;
    }
    return reference_q_;
  }

  Eigen::Vector3d command_bodyrate_from_rc() const
  {
    const double roll_axis = shaped_channel(1, roll_reverse_);
    const double pitch_axis = shaped_channel(2, pitch_reverse_);
    const double yaw_axis = shaped_channel(4, yaw_reverse_);
    return Eigen::Vector3d(
      roll_axis * max_roll_rate_,
      pitch_axis * max_pitch_rate_,
      yaw_axis * max_yaw_rate_);
  }

  Eigen::Vector3d attitude_feedback_bodyrate(
    const Eigen::Quaterniond &actual_q,
    const Eigen::Quaterniond &target_q) const
  {
    Eigen::Quaterniond q_error = actual_q.inverse() * target_q;
    if (q_error.norm() > 1e-6) {
      q_error.normalize();
    } else {
      return Eigen::Vector3d::Zero();
    }

    const double sign = q_error.w() >= 0.0 ? 1.0 : -1.0;
    Eigen::Vector3d bodyrate(
      sign * 2.0 * attitude_kang_.x() * q_error.x(),
      sign * 2.0 * attitude_kang_.y() * q_error.y(),
      sign * 2.0 * attitude_kang_.z() * q_error.z());
    bodyrate.x() = clamp_symmetric(bodyrate.x(), max_bodyrate_cmd_.x());
    bodyrate.y() = clamp_symmetric(bodyrate.y(), max_bodyrate_cmd_.y());
    bodyrate.z() = clamp_symmetric(bodyrate.z(), max_bodyrate_cmd_.z());
    if (!bodyrate.allFinite()) {
      return Eigen::Vector3d::Zero();
    }
    return bodyrate;
  }

  void update_reference_rpy_from_quaternion()
  {
    reference_rpy_ = rpy_from_quaternion(reference_q_);
  }

  void initialize_reference_from_attitude(const Eigen::Quaterniond &actual_q)
  {
    reference_q_ = actual_q;
    if (reference_q_.norm() > 1e-6) {
      reference_q_.normalize();
    } else {
      reference_q_.setIdentity();
    }
    update_reference_rpy_from_quaternion();
    reference_bodyrate_.setZero();
    stick_bodyrate_.setZero();
    reference_initialized_ = true;
  }

  void update_reference(double dt, const Eigen::Quaterniond &actual_q)
  {
    if (!reference_initialized_) {
      initialize_reference_from_attitude(actual_q);
    }

    stick_bodyrate_ = command_bodyrate_from_rc();
    reference_q_ = integrate_bodyrate(reference_q_, stick_bodyrate_, dt);
    reference_rpy_ = rpy_from_quaternion(reference_q_);
    reference_rpy_.x() = clamp_symmetric(reference_rpy_.x(), max_roll_);
    reference_rpy_.y() = clamp_symmetric(reference_rpy_.y(), max_pitch_);
    reference_rpy_.z() = normalize_angle(reference_rpy_.z());
    reference_q_ = eigen_quaternion_from_rpy(reference_rpy_);
  }

  void stop_reference_motion()
  {
    stick_bodyrate_.setZero();
    reference_bodyrate_.setZero();
  }

  void update_setpoint_bodyrate(const Eigen::Quaterniond &actual_q)
  {
    if (setpoint_output_mode_ == "bodyrate") {
      reference_bodyrate_ = stick_bodyrate_;
      return;
    }
    if (setpoint_output_mode_ == "attitude_bodyrate") {
      reference_bodyrate_ = attitude_feedback_bodyrate(actual_q, reference_q_);
      return;
    }
    reference_bodyrate_.setZero();
  }

  void update_thrust(double dt, bool active)
  {
    if (active && thrust_ramp_allowed()) {
      nominal_thrust_ = slew_toward(
        nominal_thrust_,
        thrust_base_,
        thrust_ramp_per_s_ * std::max(0.0, dt));
    } else if (!active) {
      nominal_thrust_ = slew_toward(
        nominal_thrust_,
        thrust_min_,
        thrust_ramp_per_s_ * std::max(0.0, dt));
    }

    const double target = active ? target_thrust_from_rc() : thrust_min_;
    const double max_delta = thrust_slew_per_s_ * std::max(0.0, dt);
    current_thrust_ = slew_toward(current_thrust_, target, max_delta);
    current_thrust_ = std::clamp(current_thrust_, thrust_min_, thrust_max_);
  }

  void publish_setpoint(const rclcpp::Time &stamp)
  {
    mavros_msgs::msg::AttitudeTarget msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    if (setpoint_output_mode_ == "bodyrate" ||
        setpoint_output_mode_ == "attitude_bodyrate") {
      msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;
      msg.orientation = quaternion_msg_from_eigen(Eigen::Quaterniond::Identity());
      msg.body_rate.x = reference_bodyrate_.x();
      msg.body_rate.y = reference_bodyrate_.y();
      msg.body_rate.z = reference_bodyrate_.z();
      last_setpoint_q_ = actual_attitude_quaternion();
      last_setpoint_rpy_ = reference_rpy_;
      have_last_setpoint_ = true;
    } else {
      msg.type_mask =
        mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
        mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
        mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
      const Eigen::Quaterniond setpoint_q = setpoint_quaternion_from_reference();
      last_setpoint_q_ = setpoint_q;
      last_setpoint_rpy_ = rpy_from_quaternion(setpoint_q);
      have_last_setpoint_ = true;
      msg.orientation = quaternion_msg_from_eigen(setpoint_q);
      msg.body_rate.x = 0.0;
      msg.body_rate.y = 0.0;
      msg.body_rate.z = 0.0;
    }
    msg.thrust = static_cast<float>(current_thrust_);
    setpoint_pub_->publish(msg);
  }

  void publish_vectors(
    const rclcpp::Time &stamp,
    const Eigen::Quaterniond &actual_q,
    const Eigen::Vector3d &actual_bodyrate)
  {
    if (!reference_initialized_) {
      initialize_reference_from_attitude(actual_q);
    }

    const Eigen::Quaterniond setpoint_q = have_last_setpoint_ ?
      last_setpoint_q_ :
      setpoint_quaternion_from_reference();
    const Eigen::Vector3d setpoint_rpy = have_last_setpoint_ ?
      last_setpoint_rpy_ :
      rpy_from_quaternion(setpoint_q);
    const Eigen::Vector3d actual_rpy = rpy_from_quaternion(actual_q);
    Eigen::Vector3d rpy_error = setpoint_rpy - actual_rpy;
    rpy_error.x() = normalize_angle(rpy_error.x());
    rpy_error.y() = normalize_angle(rpy_error.y());
    rpy_error.z() = normalize_angle(rpy_error.z());
    const Eigen::Vector3d bodyrate_error = reference_bodyrate_ - actual_bodyrate;

    reference_rpy_pub_->publish(make_vector_msg(stamp, frame_id_, setpoint_rpy));
    desired_rpy_pub_->publish(make_vector_msg(stamp, frame_id_, reference_rpy_));
    actual_rpy_pub_->publish(make_vector_msg(stamp, frame_id_, actual_rpy));
    error_rpy_pub_->publish(make_vector_msg(stamp, frame_id_, rpy_error));
    reference_bodyrate_pub_->publish(make_vector_msg(stamp, "base_link", reference_bodyrate_));
    actual_bodyrate_pub_->publish(make_vector_msg(stamp, "base_link", actual_bodyrate));
    error_bodyrate_pub_->publish(make_vector_msg(stamp, "base_link", bodyrate_error));

    std_msgs::msg::Float64 thrust_msg;
    thrust_msg.data = current_thrust_;
    thrust_pub_->publish(thrust_msg);
  }

  void publish_status(
    const rclcpp::Time &stamp,
    bool active,
    bool rc_ok,
    bool imu_ok,
    bool odom_ok,
    bool state_ok) const
  {
    std_msgs::msg::String msg;
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(3);
    ss << "active=" << (active ? "true" : "false")
       << " ch" << test_att_channel_ << "_high=" << (test_att_switch_high() ? "true" : "false")
       << " enter_confirmed=" << (enter_confirmed_.load() ? "true" : "false")
       << " rc_ok=" << (rc_ok ? "true" : "false")
       << " imu_ok=" << (imu_ok ? "true" : "false")
       << " odom_ok=" << (odom_ok ? "true" : "false")
       << " state_ok=" << (state_ok ? "true" : "false")
       << " fcu_mode=" << (have_state_ ? state_msg_.mode : std::string("<none>"))
       << " armed=" << (have_state_ && state_msg_.armed ? "true" : "false")
       << " auto_arm=" << (auto_arm_enable_ ? "true" : "false")
       << " auto_offboard=" << (auto_offboard_enable_ ? "true" : "false")
       << " ch" << test_att_channel_ << "_pwm=" << channel_pwm(rc_msg_, test_att_channel_, 1000.0)
       << " rc_age_s=" << rc_age(stamp)
       << " imu_age_s=" << imu_age(stamp)
       << " odom_age_s=" << odom_age(stamp)
       << " state_age_s=" << state_age(stamp)
       << " setpoint_output=" << setpoint_output_mode_
       << " setpoint_alignment=" << setpoint_alignment_mode_
       << " KAng=(" << attitude_kang_.x() << "," << attitude_kang_.y() << ","
       << attitude_kang_.z() << ")"
       << " stick_bodyrate=(" << stick_bodyrate_.x() << "," << stick_bodyrate_.y()
       << "," << stick_bodyrate_.z() << ")"
       << " nominal_thrust=" << nominal_thrust_
       << " thrust=" << current_thrust_;
    msg.data = ss.str();
    status_pub_->publish(msg);
  }

  void on_timer()
  {
    const rclcpp::Time stamp = now();
    double dt = 1.0 / std::max(1.0, rate_hz_);
    if (have_last_loop_) {
      const double measured_dt = (stamp - last_loop_stamp_).seconds();
      if (measured_dt > 1e-4 && measured_dt < 0.2) {
        dt = measured_dt;
      }
    }
    have_last_loop_ = true;
    last_loop_stamp_ = stamp;

    const bool rc_ok = rc_fresh(stamp);
    const bool imu_ok = imu_fresh(stamp);
    const bool odom_ok = odom_fresh(stamp);
    const bool state_ok = state_fresh(stamp);
    const bool feedback_ok = imu_ok && (!odom_required_for_setpoint() || odom_ok);
    const bool ch11_high = test_att_switch_high();
    const bool enter_ok = !enter_confirm_enable_ || enter_confirmed_.load();
    const bool active = rc_ok && feedback_ok && ch11_high && enter_ok;

    if (!feedback_ok) {
      if (active_) {
        maybe_restore_mode(stamp);
        maybe_disarm_on_inactive(stamp);
      }
      active_ = false;
      have_active_start_ = false;
      update_thrust(dt, false);
      publish_status(stamp, false, rc_ok, imu_ok, odom_ok, state_ok);
      return;
    }

    const Eigen::Quaterniond actual_q = actual_attitude_quaternion();
    const Eigen::Vector3d actual_bodyrate = actual_bodyrate_from_imu();

    if (active && !active_) {
      initialize_reference_from_attitude(reference_seed_quaternion());
      active_start_stamp_ = stamp;
      have_active_start_ = true;
      reset_automation_requests();
      reset_thrust_ramp();
      if (!is_offboard() && state_ok && !state_msg_.mode.empty()) {
        mode_before_offboard_ = state_msg_.mode;
        have_mode_before_offboard_ = true;
      }
      RCLCPP_INFO(get_logger(), "CH%d high and Enter confirmed: entering test_att mode.", test_att_channel_);
    } else if (!active && active_) {
      stop_reference_motion();
      maybe_restore_mode(stamp);
      maybe_disarm_on_inactive(stamp);
      have_active_start_ = false;
      reset_automation_requests();
      if (enter_confirm_enable_) {
        enter_confirmed_.store(false);
      }
      RCLCPP_WARN(get_logger(), "Leaving test_att mode; setpoint publication stopped.");
    }

    if (active) {
      update_reference(dt, actual_q);
    } else {
      if (!reference_initialized_) {
        initialize_reference_from_attitude(actual_q);
      }
      stop_reference_motion();
    }
    update_setpoint_bodyrate(actual_q);

    update_thrust(dt, active);
    if (active) {
      publish_setpoint(stamp);
      maybe_request_offboard(stamp, active, state_ok);
      maybe_request_arm(stamp, active, state_ok);
    }
    publish_vectors(stamp, actual_q, actual_bodyrate);
    publish_status(stamp, active, rc_ok, imu_ok, odom_ok, state_ok);
    active_ = active;
  }

  std::string rc_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string state_topic_;
  std::string setpoint_topic_;
  std::string set_mode_service_;
  std::string arming_service_;
  std::string reference_rpy_topic_;
  std::string desired_rpy_topic_;
  std::string actual_rpy_topic_;
  std::string error_rpy_topic_;
  std::string reference_bodyrate_topic_;
  std::string actual_bodyrate_topic_;
  std::string error_bodyrate_topic_;
  std::string thrust_topic_;
  std::string status_topic_;
  std::string frame_id_;

  double rate_hz_{100.0};
  int test_att_channel_{11};
  double active_threshold_{0.75};
  double rc_timeout_s_{0.3};
  double imu_timeout_s_{0.3};
  double odom_timeout_s_{0.3};
  double state_timeout_s_{1.0};
  bool auto_arm_enable_{true};
  bool auto_offboard_enable_{true};
  bool require_offboard_for_auto_arm_{true};
  bool restore_mode_on_inactive_{true};
  bool disarm_on_inactive_{false};
  bool enter_confirm_enable_{true};
  double arm_request_delay_s_{0.2};
  double arm_request_period_s_{1.0};
  double offboard_request_delay_s_{0.5};
  double offboard_request_period_s_{1.0};
  double stick_deadzone_{0.08};
  double stick_expo_{1.7};
  bool roll_reverse_{false};
  bool pitch_reverse_{false};
  bool yaw_reverse_{true};
  bool throttle_reverse_{false};
  double max_roll_rate_{deg2rad(45.0)};
  double max_pitch_rate_{deg2rad(45.0)};
  double max_yaw_rate_{deg2rad(60.0)};
  double max_roll_{deg2rad(35.0)};
  double max_pitch_{deg2rad(35.0)};
  Eigen::Vector3d attitude_kang_{6.0, 6.0, 3.0};
  Eigen::Vector3d max_bodyrate_cmd_{deg2rad(143.0), deg2rad(143.0), deg2rad(86.0)};
  double thrust_base_{0.35};
  double thrust_min_{0.20};
  double thrust_max_{0.45};
  double thrust_ramp_per_s_{0.05};
  double thrust_slew_per_s_{0.20};
  bool require_armed_for_thrust_ramp_{true};
  bool require_offboard_for_thrust_ramp_{true};
  std::string setpoint_output_mode_{"attitude"};
  std::string setpoint_alignment_mode_{"direct_imu"};

  mavros_msgs::msg::RCIn rc_msg_;
  sensor_msgs::msg::Imu imu_msg_;
  nav_msgs::msg::Odometry odom_msg_;
  mavros_msgs::msg::State state_msg_;
  rclcpp::Time rc_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time imu_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_loop_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time active_start_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_arm_request_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_offboard_request_stamp_{0, 0, RCL_ROS_TIME};
  bool have_rc_{false};
  bool have_imu_{false};
  bool have_odom_{false};
  bool have_state_{false};
  bool have_last_loop_{false};
  bool have_active_start_{false};
  bool have_last_arm_request_{false};
  bool have_last_offboard_request_{false};
  bool have_mode_before_offboard_{false};
  bool active_{false};
  bool reference_initialized_{false};
  std::string mode_before_offboard_{"MANUAL"};

  Eigen::Quaterniond reference_q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d reference_rpy_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d stick_bodyrate_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d reference_bodyrate_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond last_setpoint_q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d last_setpoint_rpy_{Eigen::Vector3d::Zero()};
  double nominal_thrust_{0.20};
  double current_thrust_{0.20};
  std::atomic<bool> enter_confirmed_{false};
  bool have_last_setpoint_{false};

  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr reference_rpy_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr desired_rpy_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr actual_rpy_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr error_rpy_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr reference_bodyrate_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr actual_bodyrate_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr error_bodyrate_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestAttitudeModeNode>());
  rclcpp::shutdown();
  return 0;
}
