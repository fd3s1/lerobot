#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <sstream>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
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

Eigen::Vector3d rpy_from_quaternion(const geometry_msgs::msg::Quaternion &q_msg)
{
  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  if (q.norm() > 1e-6) {
    q.normalize();
  } else {
    q.setIdentity();
  }

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

geometry_msgs::msg::Quaternion quaternion_from_rpy(const Eigen::Vector3d &rpy)
{
  const Eigen::AngleAxisd roll_angle(rpy.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(rpy.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(rpy.z(), Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
  q.normalize();

  geometry_msgs::msg::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

Eigen::Vector3d euler_rates_to_body_rates(const Eigen::Vector3d &rpy, const Eigen::Vector3d &rpy_rate)
{
  const double roll = rpy.x();
  const double pitch = rpy.y();
  const double roll_dot = rpy_rate.x();
  const double pitch_dot = rpy_rate.y();
  const double yaw_dot = rpy_rate.z();

  const double p = roll_dot - yaw_dot * std::sin(pitch);
  const double q = pitch_dot * std::cos(roll) + yaw_dot * std::sin(roll) * std::cos(pitch);
  const double r = -pitch_dot * std::sin(roll) + yaw_dot * std::cos(roll) * std::cos(pitch);
  return Eigen::Vector3d(p, q, r);
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
    setpoint_topic_ =
      declare_parameter<std::string>("setpoint_topic", "/mavros/setpoint_raw/attitude");

    reference_rpy_topic_ =
      declare_parameter<std::string>("reference_rpy_topic", "/test_att/reference_rpy");
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

    stick_deadzone_ = declare_parameter<double>("stick_deadzone", 0.08);
    stick_expo_ = declare_parameter<double>("stick_expo", 1.7);
    roll_reverse_ = declare_parameter<bool>("roll_reverse", false);
    pitch_reverse_ = declare_parameter<bool>("pitch_reverse", true);
    yaw_reverse_ = declare_parameter<bool>("yaw_reverse", false);
    throttle_reverse_ = declare_parameter<bool>("throttle_reverse", true);

    max_roll_rate_ = deg2rad(declare_parameter<double>("max_roll_rate_dps", 45.0));
    max_pitch_rate_ = deg2rad(declare_parameter<double>("max_pitch_rate_dps", 45.0));
    max_yaw_rate_ = deg2rad(declare_parameter<double>("max_yaw_rate_dps", 60.0));
    max_roll_ = deg2rad(declare_parameter<double>("max_roll_deg", 35.0));
    max_pitch_ = deg2rad(declare_parameter<double>("max_pitch_deg", 35.0));

    thrust_base_ = declare_parameter<double>("thrust_base", 0.35);
    thrust_min_ = declare_parameter<double>("thrust_min", 0.20);
    thrust_max_ = declare_parameter<double>("thrust_max", 0.45);
    thrust_slew_per_s_ = declare_parameter<double>("thrust_slew_per_s", 0.20);
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

    setpoint_pub_ = create_publisher<mavros_msgs::msg::AttitudeTarget>(setpoint_topic_, 10);
    reference_rpy_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(reference_rpy_topic_, 10);
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

    current_thrust_ = thrust_base_;
    const double period_s = 1.0 / std::max(1.0, rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_s)),
      std::bind(&TestAttitudeModeNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "test_attitude_mode_node started. CH%d high enables setpoint. rc=%s imu=%s setpoint=%s",
      test_att_channel_,
      rc_topic_.c_str(),
      imu_topic_.c_str(),
      setpoint_topic_.c_str());
  }

private:
  void sanitize_parameters()
  {
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

  double rc_age(const rclcpp::Time &stamp) const
  {
    return have_rc_ ? (stamp - rc_stamp_).seconds() : -1.0;
  }

  double imu_age(const rclcpp::Time &stamp) const
  {
    return have_imu_ ? (stamp - imu_stamp_).seconds() : -1.0;
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
      return thrust_base_ + throttle * (thrust_max_ - thrust_base_);
    }
    return thrust_base_ + throttle * (thrust_base_ - thrust_min_);
  }

  Eigen::Vector3d actual_bodyrate_from_imu() const
  {
    return Eigen::Vector3d(
      imu_msg_.angular_velocity.x,
      imu_msg_.angular_velocity.y,
      imu_msg_.angular_velocity.z);
  }

  void initialize_reference_from_imu(const Eigen::Vector3d &actual_rpy)
  {
    reference_rpy_ = actual_rpy;
    reference_rate_rpy_.setZero();
    reference_bodyrate_.setZero();
    reference_initialized_ = true;
  }

  void update_reference(double dt, const Eigen::Vector3d &actual_rpy)
  {
    if (!reference_initialized_) {
      initialize_reference_from_imu(actual_rpy);
    }

    const double roll_axis = shaped_channel(1, roll_reverse_);
    const double pitch_axis = shaped_channel(2, pitch_reverse_);
    const double yaw_axis = shaped_channel(4, yaw_reverse_);

    reference_rate_rpy_.x() = roll_axis * max_roll_rate_;
    reference_rate_rpy_.y() = pitch_axis * max_pitch_rate_;
    reference_rate_rpy_.z() = yaw_axis * max_yaw_rate_;

    reference_rpy_ += reference_rate_rpy_ * dt;
    reference_rpy_.x() = clamp_symmetric(reference_rpy_.x(), max_roll_);
    reference_rpy_.y() = clamp_symmetric(reference_rpy_.y(), max_pitch_);
    reference_rpy_.z() = normalize_angle(reference_rpy_.z());

    reference_bodyrate_ = euler_rates_to_body_rates(reference_rpy_, reference_rate_rpy_);
  }

  void stop_reference_motion()
  {
    reference_rate_rpy_.setZero();
    reference_bodyrate_.setZero();
  }

  void update_thrust(double dt, bool active)
  {
    const double target = active ? target_thrust_from_rc() : thrust_base_;
    const double max_delta = thrust_slew_per_s_ * std::max(0.0, dt);
    current_thrust_ = slew_toward(current_thrust_, target, max_delta);
    current_thrust_ = std::clamp(current_thrust_, thrust_min_, thrust_max_);
  }

  void publish_setpoint(const rclcpp::Time &stamp)
  {
    mavros_msgs::msg::AttitudeTarget msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.type_mask =
      mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
      mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
      mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
    msg.orientation = quaternion_from_rpy(reference_rpy_);
    msg.body_rate.x = 0.0;
    msg.body_rate.y = 0.0;
    msg.body_rate.z = 0.0;
    msg.thrust = static_cast<float>(current_thrust_);
    setpoint_pub_->publish(msg);
  }

  void publish_vectors(
    const rclcpp::Time &stamp,
    const Eigen::Vector3d &actual_rpy,
    const Eigen::Vector3d &actual_bodyrate)
  {
    if (!reference_initialized_) {
      initialize_reference_from_imu(actual_rpy);
    }

    Eigen::Vector3d rpy_error = reference_rpy_ - actual_rpy;
    rpy_error.x() = normalize_angle(rpy_error.x());
    rpy_error.y() = normalize_angle(rpy_error.y());
    rpy_error.z() = normalize_angle(rpy_error.z());
    const Eigen::Vector3d bodyrate_error = reference_bodyrate_ - actual_bodyrate;

    reference_rpy_pub_->publish(make_vector_msg(stamp, frame_id_, reference_rpy_));
    actual_rpy_pub_->publish(make_vector_msg(stamp, frame_id_, actual_rpy));
    error_rpy_pub_->publish(make_vector_msg(stamp, frame_id_, rpy_error));
    reference_bodyrate_pub_->publish(make_vector_msg(stamp, "base_link", reference_bodyrate_));
    actual_bodyrate_pub_->publish(make_vector_msg(stamp, "base_link", actual_bodyrate));
    error_bodyrate_pub_->publish(make_vector_msg(stamp, "base_link", bodyrate_error));

    std_msgs::msg::Float64 thrust_msg;
    thrust_msg.data = current_thrust_;
    thrust_pub_->publish(thrust_msg);
  }

  void publish_status(const rclcpp::Time &stamp, bool active, bool rc_ok, bool imu_ok) const
  {
    std_msgs::msg::String msg;
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(3);
    ss << "active=" << (active ? "true" : "false")
       << " rc_ok=" << (rc_ok ? "true" : "false")
       << " imu_ok=" << (imu_ok ? "true" : "false")
       << " ch" << test_att_channel_ << "_pwm=" << channel_pwm(rc_msg_, test_att_channel_, 1000.0)
       << " rc_age_s=" << rc_age(stamp)
       << " imu_age_s=" << imu_age(stamp)
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
    const bool active = rc_ok && imu_ok && test_att_switch_high();

    if (!imu_ok) {
      active_ = false;
      update_thrust(dt, false);
      publish_status(stamp, false, rc_ok, false);
      return;
    }

    const Eigen::Vector3d actual_rpy = rpy_from_quaternion(imu_msg_.orientation);
    const Eigen::Vector3d actual_bodyrate = actual_bodyrate_from_imu();

    if (active && !active_) {
      initialize_reference_from_imu(actual_rpy);
      RCLCPP_INFO(get_logger(), "CH%d high: entering test_att mode.", test_att_channel_);
    } else if (!active && active_) {
      stop_reference_motion();
      RCLCPP_WARN(get_logger(), "Leaving test_att mode; setpoint publication stopped.");
    }

    if (active) {
      update_reference(dt, actual_rpy);
    } else {
      if (!reference_initialized_) {
        initialize_reference_from_imu(actual_rpy);
      }
      stop_reference_motion();
    }

    update_thrust(dt, active);
    if (active) {
      publish_setpoint(stamp);
    }
    publish_vectors(stamp, actual_rpy, actual_bodyrate);
    publish_status(stamp, active, rc_ok, imu_ok);
    active_ = active;
  }

  std::string rc_topic_;
  std::string imu_topic_;
  std::string setpoint_topic_;
  std::string reference_rpy_topic_;
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
  double stick_deadzone_{0.08};
  double stick_expo_{1.7};
  bool roll_reverse_{false};
  bool pitch_reverse_{true};
  bool yaw_reverse_{false};
  bool throttle_reverse_{true};
  double max_roll_rate_{deg2rad(45.0)};
  double max_pitch_rate_{deg2rad(45.0)};
  double max_yaw_rate_{deg2rad(60.0)};
  double max_roll_{deg2rad(35.0)};
  double max_pitch_{deg2rad(35.0)};
  double thrust_base_{0.35};
  double thrust_min_{0.20};
  double thrust_max_{0.45};
  double thrust_slew_per_s_{0.20};

  mavros_msgs::msg::RCIn rc_msg_;
  sensor_msgs::msg::Imu imu_msg_;
  rclcpp::Time rc_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time imu_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_loop_stamp_{0, 0, RCL_ROS_TIME};
  bool have_rc_{false};
  bool have_imu_{false};
  bool have_last_loop_{false};
  bool active_{false};
  bool reference_initialized_{false};

  Eigen::Vector3d reference_rpy_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d reference_rate_rpy_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d reference_bodyrate_{Eigen::Vector3d::Zero()};
  double current_thrust_{0.35};

  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr reference_rpy_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr actual_rpy_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr error_rpy_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr reference_bodyrate_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr actual_bodyrate_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr error_bodyrate_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestAttitudeModeNode>());
  rclcpp::shutdown();
  return 0;
}
