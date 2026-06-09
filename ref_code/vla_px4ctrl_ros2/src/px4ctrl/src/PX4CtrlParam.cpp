#include "PX4CtrlParam.h"

#include <array>
#include <string>
#include <vector>

namespace {

std::vector<double> array_to_vector(const std::array<double, 3> &value)
{
  return {value[0], value[1], value[2]};
}

void declare_diag_parameter(
  rclcpp::Node &node,
  const std::string &name,
  std::array<double, 3> &value)
{
  const auto param_value =
    node.declare_parameter<std::vector<double>>(name, array_to_vector(value));
  if (param_value.size() != value.size()) {
    RCLCPP_ERROR(
      node.get_logger(),
      "Parameter %s must have exactly 3 elements; keeping defaults.",
      name.c_str());
    return;
  }
  for (std::size_t i = 0; i < value.size(); ++i) {
    value[i] = param_value[i];
  }
}

}  // namespace

void Parameter_t::config_from_ros_node(rclcpp::Node &node)
{
  ctrl_freq_max = node.declare_parameter<double>("ctrl_freq_max", ctrl_freq_max);
  max_manual_vel = node.declare_parameter<double>("max_manual_vel", max_manual_vel);
  frame_id = node.declare_parameter<std::string>("frame_id", frame_id);
  use_bodyrate_ctrl = node.declare_parameter<bool>("use_bodyrate_ctrl", use_bodyrate_ctrl);

  topics.rc = node.declare_parameter<std::string>("topics.rc", topics.rc);
  topics.odom = node.declare_parameter<std::string>("topics.odom", topics.odom);
  topics.imu = node.declare_parameter<std::string>("topics.imu", topics.imu);
  topics.cmd = node.declare_parameter<std::string>("topics.cmd", topics.cmd);
  topics.takeoff_land = node.declare_parameter<std::string>("topics.takeoff_land", topics.takeoff_land);
  topics.setpoint = node.declare_parameter<std::string>("topics.setpoint", topics.setpoint);
  topics.simulink_setpoint =
    node.declare_parameter<std::string>("topics.simulink_setpoint", topics.simulink_setpoint);
  topics.simulink_reference =
    node.declare_parameter<std::string>("topics.simulink_reference", topics.simulink_reference);
  topics.simulink_tracking_error =
    node.declare_parameter<std::string>(
      "topics.simulink_tracking_error", topics.simulink_tracking_error);
  topics.expert_pose = node.declare_parameter<std::string>("topics.expert_pose", topics.expert_pose);
  topics.gripper_command =
    node.declare_parameter<std::string>("topics.gripper_command", topics.gripper_command);
  topics.traj_start_trigger =
    node.declare_parameter<std::string>("topics.traj_start_trigger", topics.traj_start_trigger);
  topics.state = node.declare_parameter<std::string>("topics.state", topics.state);
  topics.extended_state =
    node.declare_parameter<std::string>("topics.extended_state", topics.extended_state);
  topics.battery = node.declare_parameter<std::string>("topics.battery", topics.battery);

  services.set_mode = node.declare_parameter<std::string>("services.set_mode", services.set_mode);
  services.arming = node.declare_parameter<std::string>("services.arming", services.arming);
  services.command = node.declare_parameter<std::string>("services.command", services.command);

  msg_timeout.odom = node.declare_parameter<double>("msg_timeout.odom", msg_timeout.odom);
  msg_timeout.imu = node.declare_parameter<double>("msg_timeout.imu", msg_timeout.imu);
  msg_timeout.rc = node.declare_parameter<double>("msg_timeout.rc", msg_timeout.rc);
  msg_timeout.cmd = node.declare_parameter<double>("msg_timeout.cmd", msg_timeout.cmd);
  msg_timeout.bat = node.declare_parameter<double>("msg_timeout.bat", msg_timeout.bat);

  rc_reverse.roll = node.declare_parameter<bool>("rc_reverse.roll", rc_reverse.roll);
  rc_reverse.pitch = node.declare_parameter<bool>("rc_reverse.pitch", rc_reverse.pitch);
  rc_reverse.yaw = node.declare_parameter<bool>("rc_reverse.yaw", rc_reverse.yaw);
  rc_reverse.throttle = node.declare_parameter<bool>("rc_reverse.throttle", rc_reverse.throttle);

  takeoff_land.enable =
    node.declare_parameter<bool>("auto_takeoff_land.enable", takeoff_land.enable);
  takeoff_land.enable_auto_arm =
    node.declare_parameter<bool>("auto_takeoff_land.enable_auto_arm", takeoff_land.enable_auto_arm);
  takeoff_land.no_RC =
    node.declare_parameter<bool>("auto_takeoff_land.no_RC", takeoff_land.no_RC);
  takeoff_land.height =
    node.declare_parameter<double>("auto_takeoff_land.takeoff_height", takeoff_land.height);
  takeoff_land.speed =
    node.declare_parameter<double>("auto_takeoff_land.takeoff_land_speed", takeoff_land.speed);

  limits.x_min = node.declare_parameter<double>("limits.x_min", limits.x_min);
  limits.x_max = node.declare_parameter<double>("limits.x_max", limits.x_max);
  limits.y_min = node.declare_parameter<double>("limits.y_min", limits.y_min);
  limits.y_max = node.declare_parameter<double>("limits.y_max", limits.y_max);
  limits.z_min = node.declare_parameter<double>("limits.z_min", limits.z_min);
  limits.z_max = node.declare_parameter<double>("limits.z_max", limits.z_max);

  gripper.rc_channel = node.declare_parameter<int>("gripper.rc_channel", gripper.rc_channel);
  gripper.pwm_open = node.declare_parameter<int>("gripper.pwm_open", gripper.pwm_open);
  gripper.pwm_close = node.declare_parameter<int>("gripper.pwm_close", gripper.pwm_close);
  gripper.open_position =
    node.declare_parameter<double>("gripper.open_position", gripper.open_position);
  gripper.closed_position =
    node.declare_parameter<double>("gripper.closed_position", gripper.closed_position);
  gripper.force_open_below_z =
    node.declare_parameter<double>("gripper.force_open_below_z", gripper.force_open_below_z);

  cmd_feedforward.enable =
    node.declare_parameter<bool>("cmd_feedforward.enable", cmd_feedforward.enable);
  cmd_feedforward.max_velocity =
    node.declare_parameter<double>("cmd_feedforward.max_velocity", cmd_feedforward.max_velocity);
  cmd_feedforward.max_acceleration =
    node.declare_parameter<double>("cmd_feedforward.max_acceleration", cmd_feedforward.max_acceleration);

  controller.gravity =
    node.declare_parameter<double>("controller.gravity", controller.gravity);
  controller.max_angle_deg =
    node.declare_parameter<double>("controller.max_angle_deg", controller.max_angle_deg);
  controller.max_bodyrate_x =
    node.declare_parameter<double>("controller.max_bodyrate_x", controller.max_bodyrate_x);
  controller.max_bodyrate_y =
    node.declare_parameter<double>("controller.max_bodyrate_y", controller.max_bodyrate_y);
  controller.max_bodyrate_z =
    node.declare_parameter<double>("controller.max_bodyrate_z", controller.max_bodyrate_z);
  controller.min_thrust =
    node.declare_parameter<double>("controller.min_thrust", controller.min_thrust);
  controller.max_thrust =
    node.declare_parameter<double>("controller.max_thrust", controller.max_thrust);

  declare_diag_parameter(node, "ude.Kp_diag", ude.Kp_diag);
  declare_diag_parameter(node, "ude.Kd_diag", ude.Kd_diag);
  declare_diag_parameter(node, "ude.T_diag", ude.T_diag);
  declare_diag_parameter(node, "ude.max_f_hat", ude.max_f_hat);
  declare_diag_parameter(node, "ude.max_u_acc", ude.max_u_acc);
  declare_diag_parameter(node, "attitude.KAng_diag", attitude.KAng_diag);

  thrust_model.hover_thrust =
    node.declare_parameter<double>("thrust_model.hover_thrust", thrust_model.hover_thrust);
  thrust_model.enable_estimation =
    node.declare_parameter<bool>("thrust_model.enable_estimation", thrust_model.enable_estimation);
  thrust_model.print_value =
    node.declare_parameter<bool>("thrust_model.print_value", thrust_model.print_value);
  thrust_model.rho2 =
    node.declare_parameter<double>("thrust_model.rho2", thrust_model.rho2);
  thrust_model.min_thr2acc =
    node.declare_parameter<double>("thrust_model.min_thr2acc", thrust_model.min_thr2acc);
  thrust_model.max_thr2acc =
    node.declare_parameter<double>("thrust_model.max_thr2acc", thrust_model.max_thr2acc);

  if (takeoff_land.enable_auto_arm && !takeoff_land.enable) {
    takeoff_land.enable_auto_arm = false;
    RCLCPP_ERROR(
      node.get_logger(),
      "\"enable_auto_arm\" is only allowed with \"auto_takeoff_land.enable\" enabled.");
  }
}
