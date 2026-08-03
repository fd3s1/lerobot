#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <uav_utils/utils.h>

namespace {

using ControlPoints = std::array<Eigen::Vector3d, 6>;
using YawControls = std::array<double, 6>;

struct ReferenceState
{
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
  double yaw_accel{0.0};
};

struct GoalState
{
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

struct Trajectory
{
  ControlPoints p_ctrl{};
  YawControls yaw_ctrl{};
  rclcpp::Time start_time{0, 0, RCL_ROS_TIME};
  double duration_s{1.0};
  GoalState goal;
  uint32_t id{0};
  bool active{false};
};

bool finite_vector(const Eigen::Vector3d &value)
{
  return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

double bernstein5(int i, double u)
{
  const double v = 1.0 - u;
  switch (i) {
    case 0: return v * v * v * v * v;
    case 1: return 5.0 * u * v * v * v * v;
    case 2: return 10.0 * u * u * v * v * v;
    case 3: return 10.0 * u * u * u * v * v;
    case 4: return 5.0 * u * u * u * u * v;
    default: return u * u * u * u * u;
  }
}

double bernstein4(int i, double u)
{
  const double v = 1.0 - u;
  switch (i) {
    case 0: return v * v * v * v;
    case 1: return 4.0 * u * v * v * v;
    case 2: return 6.0 * u * u * v * v;
    case 3: return 4.0 * u * u * u * v;
    default: return u * u * u * u;
  }
}

double bernstein3(int i, double u)
{
  const double v = 1.0 - u;
  switch (i) {
    case 0: return v * v * v;
    case 1: return 3.0 * u * v * v;
    case 2: return 3.0 * u * u * v;
    default: return u * u * u;
  }
}

Eigen::Vector3d eval_position(const ControlPoints &ctrl, double u)
{
  Eigen::Vector3d out = Eigen::Vector3d::Zero();
  for (int i = 0; i < 6; ++i) {
    out += bernstein5(i, u) * ctrl[static_cast<std::size_t>(i)];
  }
  return out;
}

Eigen::Vector3d eval_velocity(const ControlPoints &ctrl, double u, double duration_s)
{
  Eigen::Vector3d out = Eigen::Vector3d::Zero();
  for (int i = 0; i < 5; ++i) {
    out += bernstein4(i, u) *
      (ctrl[static_cast<std::size_t>(i + 1)] - ctrl[static_cast<std::size_t>(i)]);
  }
  return 5.0 * out / duration_s;
}

Eigen::Vector3d eval_acceleration(const ControlPoints &ctrl, double u, double duration_s)
{
  Eigen::Vector3d out = Eigen::Vector3d::Zero();
  for (int i = 0; i < 4; ++i) {
    out += bernstein3(i, u) * (
      ctrl[static_cast<std::size_t>(i + 2)] -
      2.0 * ctrl[static_cast<std::size_t>(i + 1)] +
      ctrl[static_cast<std::size_t>(i)]);
  }
  return 20.0 * out / (duration_s * duration_s);
}

double eval_yaw(const YawControls &ctrl, double u)
{
  double out = 0.0;
  for (int i = 0; i < 6; ++i) {
    out += bernstein5(i, u) * ctrl[static_cast<std::size_t>(i)];
  }
  return uav_utils::normalize_angle(out);
}

double eval_yaw_rate(const YawControls &ctrl, double u, double duration_s)
{
  double out = 0.0;
  for (int i = 0; i < 5; ++i) {
    out += bernstein4(i, u) *
      (ctrl[static_cast<std::size_t>(i + 1)] - ctrl[static_cast<std::size_t>(i)]);
  }
  return 5.0 * out / duration_s;
}

double eval_yaw_accel(const YawControls &ctrl, double u, double duration_s)
{
  double out = 0.0;
  for (int i = 0; i < 4; ++i) {
    out += bernstein3(i, u) * (
      ctrl[static_cast<std::size_t>(i + 2)] -
      2.0 * ctrl[static_cast<std::size_t>(i + 1)] +
      ctrl[static_cast<std::size_t>(i)]);
  }
  return 20.0 * out / (duration_s * duration_s);
}

ControlPoints make_position_controls(
  const ReferenceState &start,
  const GoalState &goal,
  double duration_s)
{
  ControlPoints ctrl{};
  ctrl[0] = start.p;
  ctrl[1] = ctrl[0] + start.v * duration_s / 5.0;
  ctrl[2] = start.a * duration_s * duration_s / 20.0 + 2.0 * ctrl[1] - ctrl[0];
  ctrl[5] = goal.p;
  ctrl[4] = ctrl[5];
  ctrl[3] = ctrl[5];
  return ctrl;
}

YawControls make_yaw_controls(
  const ReferenceState &start,
  const GoalState &goal,
  double duration_s)
{
  const double unwrapped_goal_yaw =
    start.yaw + uav_utils::normalize_angle(goal.yaw - start.yaw);
  YawControls ctrl{};
  ctrl[0] = start.yaw;
  ctrl[1] = ctrl[0] + start.yaw_rate * duration_s / 5.0;
  ctrl[2] = start.yaw_accel * duration_s * duration_s / 20.0 + 2.0 * ctrl[1] - ctrl[0];
  ctrl[5] = unwrapped_goal_yaw;
  ctrl[4] = ctrl[5];
  ctrl[3] = ctrl[5];
  return ctrl;
}

double yaw_from_pose(const geometry_msgs::msg::PoseStamped &msg)
{
  Eigen::Quaterniond q(
    msg.pose.orientation.w,
    msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z);
  if (q.norm() <= 1e-6) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  q.normalize();
  return uav_utils::normalize_angle(uav_utils::get_yaw_from_quaternion(q));
}

}  // namespace

class TrajectoryPlannerNode final : public rclcpp::Node
{
public:
  TrajectoryPlannerNode()
  : Node("trajectory_planner")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/position_cmd_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/position_cmd_traj");
    state_topic_ = declare_parameter<std::string>("state_topic", "/px4ctrl/state");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    rate_hz_ = declare_parameter<double>("rate_hz", 50.0);
    state_gate_enable_ = declare_parameter<bool>("state_gate_enable", true);
    max_speed_xy_mps_ = declare_parameter<double>("max_speed_xy_mps", 0.30);
    max_accel_xy_mps2_ = declare_parameter<double>("max_accel_xy_mps2", 0.25);
    max_speed_z_mps_ = declare_parameter<double>("max_speed_z_mps", 0.15);
    max_accel_z_mps2_ = declare_parameter<double>("max_accel_z_mps2", 0.20);
    centering_max_speed_xy_mps_ = declare_parameter<double>("centering_max_speed_xy_mps", 0.10);
    centering_max_accel_xy_mps2_ = declare_parameter<double>("centering_max_accel_xy_mps2", 0.20);
    centering_goal_distance_m_ = declare_parameter<double>("centering_goal_distance_m", 0.20);
    centering_goal_z_window_m_ = declare_parameter<double>("centering_goal_z_window_m", 0.08);
    max_yaw_rate_radps_ = declare_parameter<double>("max_yaw_rate_radps", 0.35);
    max_yaw_accel_radps2_ = declare_parameter<double>("max_yaw_accel_radps2", 0.50);
    goal_replan_pos_threshold_m_ = declare_parameter<double>("goal_replan_pos_threshold_m", 0.005);
    goal_replan_yaw_threshold_rad_ = declare_parameter<double>("goal_replan_yaw_threshold_rad", 0.02);
    goal_stale_timeout_s_ = declare_parameter<double>("goal_stale_timeout_s", 0.5);
    min_duration_s_ = declare_parameter<double>("min_duration_s", 0.25);
    max_duration_s_ = declare_parameter<double>("max_duration_s", 12.0);
    duration_scale_ = declare_parameter<double>("duration_scale", 1.20);
    sample_check_count_ = declare_parameter<int>("sample_check_count", 80);
    zero_lateral_start_derivatives_for_vertical_goals_ =
      declare_parameter<bool>("zero_lateral_start_derivatives_for_vertical_goals", true);
    vertical_goal_xy_threshold_m_ = declare_parameter<double>("vertical_goal_xy_threshold_m", 0.03);
    vertical_goal_z_min_m_ = declare_parameter<double>("vertical_goal_z_min_m", 0.04);

    if (rate_hz_ <= 0.0) {
      rate_hz_ = 50.0;
    }
    if (sample_check_count_ < 8) {
      sample_check_count_ = 8;
    }

    pub_ = create_publisher<quadrotor_msgs::msg::PositionCommand>(output_topic_, 20);
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_,
      rclcpp::QoS(20),
      std::bind(&TrajectoryPlannerNode::goalCallback, this, std::placeholders::_1));
    state_sub_ = create_subscription<std_msgs::msg::String>(
      state_topic_,
      rclcpp::QoS(10).transient_local(),
      std::bind(&TrajectoryPlannerNode::stateCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_),
      std::bind(&TrajectoryPlannerNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "trajectory planner: %s -> %s rate=%.1fHz state_gate=%s state_topic=%s "
      "vxy=%.3f vz=%.3f axy=%.3f az=%.3f",
      input_topic_.c_str(),
      output_topic_.c_str(),
      rate_hz_,
      state_gate_enable_ ? "true" : "false",
      state_topic_.c_str(),
      max_speed_xy_mps_,
      max_speed_z_mps_,
      max_accel_xy_mps2_,
      max_accel_z_mps2_);
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    GoalState goal;
    goal.p = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    goal.yaw = yaw_from_pose(*msg);
    if (!finite_vector(goal.p) || !std::isfinite(goal.yaw)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "ignoring non-finite raw goal");
      return;
    }

    last_goal_time_ = now();

    if (!reference_initialized_) {
      reference_ = ReferenceState{};
      reference_.p = goal.p;
      reference_.yaw = goal.yaw;
      active_goal_ = goal;
      reference_initialized_ = true;
      trajectory_.active = false;
      if (shouldPublish(now())) {
        publishReference(reference_, trajectory_id_);
      }
      if (state_gate_enable_ && px4ctrl_state_ != "CMD_CTRL") {
        pending_goal_ = goal;
        pending_goal_available_ = true;
      }
      return;
    }

    if (state_gate_enable_ && px4ctrl_state_ != "CMD_CTRL") {
      pending_goal_ = goal;
      pending_goal_available_ = true;
      active_goal_ = goal;
      return;
    }

    const double goal_delta_pos = (goal.p - active_goal_.p).norm();
    const double goal_delta_yaw = std::abs(uav_utils::normalize_angle(goal.yaw - active_goal_.yaw));
    if (goal_delta_pos < goal_replan_pos_threshold_m_ &&
        goal_delta_yaw < goal_replan_yaw_threshold_rad_) {
      return;
    }

    ReferenceState start = reference_;
    if (trajectory_.active) {
      start = sampleTrajectory(now());
    }
    planTrajectory(start, goal);
  }

  void stateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string previous_state = px4ctrl_state_;
    px4ctrl_state_ = msg->data;
    last_state_time_ = now();
    if (px4ctrl_state_ == "AUTO_HOVER" && previous_state == "CMD_CTRL") {
      clearReference();
      RCLCPP_INFO(get_logger(), "AUTO_HOVER entered; clearing queued/active B-spline trajectory");
      return;
    }
    if (px4ctrl_state_ == "CMD_CTRL" && previous_state != "CMD_CTRL" && pending_goal_available_) {
      planTrajectory(reference_, pending_goal_);
      pending_goal_available_ = false;
      RCLCPP_INFO(get_logger(), "CMD_CTRL entered; planned queued raw goal as B-spline trajectory");
      return;
    }
    if (px4ctrl_state_ == "CMD_CTRL" && previous_state != "CMD_CTRL" && trajectory_.active) {
      trajectory_.start_time = last_state_time_;
      RCLCPP_INFO(get_logger(), "CMD_CTRL entered; starting queued B-spline trajectory now");
    }
  }

  void planTrajectory(const ReferenceState &start, const GoalState &goal)
  {
    ReferenceState planned_start = start;
    const Eigen::Vector3d delta = goal.p - start.p;
    const double xy_distance = std::hypot(delta.x(), delta.y());
    const double z_distance = std::abs(delta.z());
    const double yaw_delta = std::abs(uav_utils::normalize_angle(goal.yaw - start.yaw));
    const bool vertical_goal =
      zero_lateral_start_derivatives_for_vertical_goals_ &&
      xy_distance <= vertical_goal_xy_threshold_m_ &&
      z_distance >= vertical_goal_z_min_m_;
    if (vertical_goal) {
      planned_start.v.x() = 0.0;
      planned_start.v.y() = 0.0;
      planned_start.a.x() = 0.0;
      planned_start.a.y() = 0.0;
    }
    const bool use_centering_limits =
      xy_distance <= centering_goal_distance_m_ && z_distance <= centering_goal_z_window_m_;
    const double xy_speed_limit =
      use_centering_limits ? centering_max_speed_xy_mps_ : max_speed_xy_mps_;
    const double xy_accel_limit =
      use_centering_limits ? centering_max_accel_xy_mps2_ : max_accel_xy_mps2_;

    double duration_s = std::max({
      min_duration_s_,
      xy_distance / std::max(xy_speed_limit, 1e-3),
      z_distance / std::max(max_speed_z_mps_, 1e-3),
      yaw_delta / std::max(max_yaw_rate_radps_, 1e-3),
    });

    Trajectory candidate;
    candidate.goal = goal;
    candidate.id = ++trajectory_id_;
    for (int attempt = 0; attempt < 40; ++attempt) {
      candidate.duration_s = std::min(duration_s, max_duration_s_);
      candidate.p_ctrl = make_position_controls(planned_start, goal, candidate.duration_s);
      candidate.yaw_ctrl = make_yaw_controls(planned_start, goal, candidate.duration_s);
      if (trajectoryWithinLimits(candidate, xy_speed_limit, xy_accel_limit)) {
        break;
      }
      if (duration_s >= max_duration_s_) {
        break;
      }
      duration_s *= std::max(duration_scale_, 1.05);
    }

    candidate.start_time = now();
    candidate.active = true;
    trajectory_ = candidate;
    active_goal_ = goal;

    RCLCPP_INFO(
      get_logger(),
      "planned traj id=%u T=%.2fs goal=(%.3f, %.3f, %.3f yaw=%.3f) "
      "centering_limits=%s vertical_xy_reset=%s start_vxy=(%.3f, %.3f) start_axy=(%.3f, %.3f)",
      trajectory_.id,
      trajectory_.duration_s,
      goal.p.x(),
      goal.p.y(),
      goal.p.z(),
      goal.yaw,
      use_centering_limits ? "true" : "false",
      vertical_goal ? "true" : "false",
      start.v.x(),
      start.v.y(),
      start.a.x(),
      start.a.y());
  }

  bool trajectoryWithinLimits(
    const Trajectory &traj,
    double xy_speed_limit,
    double xy_accel_limit) const
  {
    for (int i = 0; i <= sample_check_count_; ++i) {
      const double u = static_cast<double>(i) / static_cast<double>(sample_check_count_);
      const Eigen::Vector3d v = eval_velocity(traj.p_ctrl, u, traj.duration_s);
      const Eigen::Vector3d a = eval_acceleration(traj.p_ctrl, u, traj.duration_s);
      const double yaw_rate = eval_yaw_rate(traj.yaw_ctrl, u, traj.duration_s);
      const double yaw_accel = eval_yaw_accel(traj.yaw_ctrl, u, traj.duration_s);
      if (std::hypot(v.x(), v.y()) > xy_speed_limit * 1.01) {
        return false;
      }
      if (std::abs(v.z()) > max_speed_z_mps_ * 1.01) {
        return false;
      }
      if (std::hypot(a.x(), a.y()) > xy_accel_limit * 1.01) {
        return false;
      }
      if (std::abs(a.z()) > max_accel_z_mps2_ * 1.01) {
        return false;
      }
      if (std::abs(yaw_rate) > max_yaw_rate_radps_ * 1.01) {
        return false;
      }
      if (std::abs(yaw_accel) > max_yaw_accel_radps2_ * 1.01) {
        return false;
      }
    }
    return true;
  }

  ReferenceState sampleTrajectory(const rclcpp::Time &time)
  {
    ReferenceState ref = reference_;
    if (!trajectory_.active) {
      return ref;
    }
    const double elapsed_s = (time - trajectory_.start_time).seconds();
    const double u = std::clamp(elapsed_s / std::max(trajectory_.duration_s, 1e-3), 0.0, 1.0);
    ref.p = eval_position(trajectory_.p_ctrl, u);
    ref.v = eval_velocity(trajectory_.p_ctrl, u, trajectory_.duration_s);
    ref.a = eval_acceleration(trajectory_.p_ctrl, u, trajectory_.duration_s);
    ref.yaw = eval_yaw(trajectory_.yaw_ctrl, u);
    ref.yaw_rate = eval_yaw_rate(trajectory_.yaw_ctrl, u, trajectory_.duration_s);
    ref.yaw_accel = eval_yaw_accel(trajectory_.yaw_ctrl, u, trajectory_.duration_s);
    if (u >= 1.0) {
      ref.p = trajectory_.goal.p;
      ref.v.setZero();
      ref.a.setZero();
      ref.yaw = trajectory_.goal.yaw;
      ref.yaw_rate = 0.0;
      ref.yaw_accel = 0.0;
      trajectory_.active = false;
    }
    reference_ = ref;
    return ref;
  }

  void timerCallback()
  {
    if (!reference_initialized_) {
      return;
    }

    const rclcpp::Time now_time = now();
    if (state_gate_enable_) {
      if (!stateFresh(now_time)) {
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "planner output stopped: px4ctrl state is stale");
        clearReference();
        return;
      }

      if (px4ctrl_state_ == "AUTO_HOVER") {
        if (rawGoalFresh(now_time)) {
          ReferenceState hold = reference_;
          hold.v.setZero();
          hold.a.setZero();
          hold.yaw_rate = 0.0;
          hold.yaw_accel = 0.0;
          publishReference(hold, trajectory_id_);
        }
        return;
      }

      if (px4ctrl_state_ != "CMD_CTRL") {
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "planner output stopped: state=%s",
          px4ctrl_state_.c_str());
        clearReference();
        return;
      }
    }

    const bool was_active = trajectory_.active;
    const ReferenceState ref = sampleTrajectory(now_time);
    if (!was_active && !trajectory_.active && goal_stale_timeout_s_ > 0.0 && !rawGoalFresh(now_time)) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "raw goal is stale after trajectory completion; stopping planner output");
      clearReference();
      return;
    }
    publishReference(ref, trajectory_.id);
  }

  bool rawGoalFresh(const rclcpp::Time &time) const
  {
    if (goal_stale_timeout_s_ <= 0.0) {
      return true;
    }
    if (last_goal_time_.nanoseconds() == 0) {
      return false;
    }
    return (time - last_goal_time_).seconds() <= goal_stale_timeout_s_;
  }

  bool stateFresh(const rclcpp::Time &time) const
  {
    if (last_state_time_.nanoseconds() == 0) {
      return false;
    }
    return (time - last_state_time_).seconds() <= 1.0;
  }

  bool shouldPublish(const rclcpp::Time &time) const
  {
    if (!state_gate_enable_) {
      return true;
    }
    if (!stateFresh(time)) {
      return false;
    }
    if (px4ctrl_state_ == "CMD_CTRL") {
      return true;
    }
    if (px4ctrl_state_ == "AUTO_HOVER") {
      return rawGoalFresh(time);
    }
    return false;
  }

  void clearReference()
  {
    reference_initialized_ = false;
    trajectory_.active = false;
    pending_goal_available_ = false;
  }

  void publishReference(const ReferenceState &ref, uint32_t trajectory_id)
  {
    quadrotor_msgs::msg::PositionCommand msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.position.x = ref.p.x();
    msg.position.y = ref.p.y();
    msg.position.z = ref.p.z();
    msg.velocity.x = ref.v.x();
    msg.velocity.y = ref.v.y();
    msg.velocity.z = ref.v.z();
    msg.acceleration.x = ref.a.x();
    msg.acceleration.y = ref.a.y();
    msg.acceleration.z = ref.a.z();
    msg.jerk.x = 0.0;
    msg.jerk.y = 0.0;
    msg.jerk.z = 0.0;
    msg.snap.x = 0.0;
    msg.snap.y = 0.0;
    msg.snap.z = 0.0;
    msg.yaw = ref.yaw;
    msg.yaw_dot = ref.yaw_rate;
    msg.yaw_ddot = 0.0;
    msg.trajectory_id = trajectory_id;
    msg.trajectory_flag = trajectory_.active ? 1U : 0U;
    pub_->publish(msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string state_topic_;
  std::string frame_id_;
  double rate_hz_{50.0};
  bool state_gate_enable_{true};
  double max_speed_xy_mps_{0.30};
  double max_accel_xy_mps2_{0.25};
  double max_speed_z_mps_{0.15};
  double max_accel_z_mps2_{0.20};
  double centering_max_speed_xy_mps_{0.10};
  double centering_max_accel_xy_mps2_{0.20};
  double centering_goal_distance_m_{0.20};
  double centering_goal_z_window_m_{0.08};
  double max_yaw_rate_radps_{0.35};
  double max_yaw_accel_radps2_{0.50};
  double goal_replan_pos_threshold_m_{0.005};
  double goal_replan_yaw_threshold_rad_{0.02};
  double goal_stale_timeout_s_{0.5};
  double min_duration_s_{0.25};
  double max_duration_s_{12.0};
  double duration_scale_{1.20};
  int sample_check_count_{80};
  bool zero_lateral_start_derivatives_for_vertical_goals_{true};
  double vertical_goal_xy_threshold_m_{0.03};
  double vertical_goal_z_min_m_{0.04};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  ReferenceState reference_;
  GoalState active_goal_;
  GoalState pending_goal_;
  Trajectory trajectory_;
  rclcpp::Time last_goal_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  std::string px4ctrl_state_;
  uint32_t trajectory_id_{0};
  bool reference_initialized_{false};
  bool pending_goal_available_{false};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
