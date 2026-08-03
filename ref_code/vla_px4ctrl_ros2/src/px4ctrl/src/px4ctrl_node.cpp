#include <chrono>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/tunnel.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/takeoff_land.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "PX4CtrlFSM.h"
#include "PX4CtrlParam.h"
#include "controller.h"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("px4ctrl");

  Parameter_t param;
  param.config_from_ros_node(*node);
  RCLCPP_INFO(
    node->get_logger(),
    "[PX4CTRL] FCU setpoint mode: %s",
    param.use_bodyrate_ctrl ? "BODYRATE" : "ATTITUDE");
  RCLCPP_INFO(
    node->get_logger(),
    "[PX4CTRL] physical TUNNEL: %s topic=%s payload_type=%d mass=%.3fkg "
    "body_rate_ff_scale=%.3f angular_accel_ff_scale=%.3f target=%d/%d",
    param.physical_control.enable ? "ENABLED" : "DISABLED",
    param.topics.physical_setpoint.c_str(),
    param.physical_control.payload_type,
    param.physical_control.mass_kg,
    param.physical_control.body_rate_feedforward_scale,
    param.physical_control.angular_acceleration_feedforward_scale,
    param.physical_control.target_system,
    param.physical_control.target_component);

  LinearControl controller(param);
  PX4CtrlFSM fsm(param, controller, node.get());
  const auto mavros_sensor_qos = rclcpp::SensorDataQoS();

  auto state_sub = node->create_subscription<mavros_msgs::msg::State>(
    param.topics.state,
    mavros_sensor_qos,
    [&fsm](const mavros_msgs::msg::State::SharedPtr msg) {
      fsm.state_data.feed(msg);
    });

  auto extended_state_sub = node->create_subscription<mavros_msgs::msg::ExtendedState>(
    param.topics.extended_state,
    mavros_sensor_qos,
    [&fsm](const mavros_msgs::msg::ExtendedState::SharedPtr msg) {
      fsm.extended_state_data.feed(msg);
    });

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    param.topics.odom,
    mavros_sensor_qos,
    [&fsm, &node](const nav_msgs::msg::Odometry::SharedPtr msg) {
      fsm.odom_data.feed(msg, node->now());
    });

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr mocap_twist_sub;
  if (param.mocap_state.enable) {
    mocap_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      param.topics.mocap_pose,
      mavros_sensor_qos,
      [&fsm, &node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        fsm.mocap_pose_data.feed(msg, node->now());
      });
    mocap_twist_sub = node->create_subscription<geometry_msgs::msg::TwistStamped>(
      param.topics.mocap_twist,
      mavros_sensor_qos,
      [&fsm, &node](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        fsm.mocap_twist_data.feed(msg, node->now());
      });
  }

  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
    param.topics.imu,
    mavros_sensor_qos,
    [&fsm, &node](const sensor_msgs::msg::Imu::SharedPtr msg) {
      fsm.imu_data.feed(msg, node->now());
    });

  auto cmd_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    param.topics.cmd,
    100,
    [&fsm, &node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      fsm.cmd_data.feed(msg, node->now());
    });

  auto cmd_traj_sub = node->create_subscription<quadrotor_msgs::msg::PositionCommand>(
    param.topics.cmd_traj,
    100,
    [&fsm, &node](const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
      fsm.cmd_data.feed(msg, node->now());
    });

  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub;
  if (!param.takeoff_land.no_RC) {
    rc_sub = node->create_subscription<mavros_msgs::msg::RCIn>(
      param.topics.rc,
      mavros_sensor_qos,
      [&fsm, &node](const mavros_msgs::msg::RCIn::SharedPtr msg) {
        fsm.rc_data.feed(msg, node->now());
      });
  }

  auto bat_sub = node->create_subscription<sensor_msgs::msg::BatteryState>(
    param.topics.battery,
    mavros_sensor_qos,
    [&fsm, &node](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
      fsm.bat_data.feed(msg, node->now());
    });

  auto takeoff_land_sub = node->create_subscription<quadrotor_msgs::msg::TakeoffLand>(
    param.topics.takeoff_land,
    100,
    [&fsm, &node](const quadrotor_msgs::msg::TakeoffLand::SharedPtr msg) {
      fsm.takeoff_land_data.feed(msg, node->now());
    });

  auto manual_flag_sub = node->create_subscription<std_msgs::msg::UInt8>(
    "/pub_trigger_flag",
    1,
    [&fsm](const std_msgs::msg::UInt8::SharedPtr msg) {
      fsm.manual_flag_cb(msg);
    });

  auto ude_tune_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    param.topics.ude_tune,
    10,
    [&fsm](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      fsm.ude_tune_cb(msg);
    });

  fsm.ctrl_FCU_pub =
    node->create_publisher<mavros_msgs::msg::AttitudeTarget>(param.topics.setpoint, 10);
  fsm.physical_setpoint_pub =
    node->create_publisher<mavros_msgs::msg::Tunnel>(param.topics.physical_setpoint, 10);
  fsm.simulink_setpoint_pub =
    node->create_publisher<nav_msgs::msg::Odometry>(param.topics.simulink_setpoint, 10);
  fsm.simulink_reference_pub =
    node->create_publisher<nav_msgs::msg::Odometry>(param.topics.simulink_reference, 10);
  fsm.simulink_actual_pub =
    node->create_publisher<nav_msgs::msg::Odometry>(param.topics.simulink_actual, 10);
  fsm.simulink_tracking_error_pub =
    node->create_publisher<nav_msgs::msg::Odometry>(param.topics.simulink_tracking_error, 10);
  fsm.simulink_ude_debug_pub =
    node->create_publisher<nav_msgs::msg::Odometry>(param.topics.simulink_ude_debug, 10);
  fsm.simulink_yaw_debug_pub =
    node->create_publisher<nav_msgs::msg::Odometry>(param.topics.simulink_yaw_debug, 10);
  fsm.ude_tune_status_pub =
    node->create_publisher<std_msgs::msg::Float64MultiArray>(param.topics.ude_tune_status, 10);
  fsm.ude_tune_status_text_pub =
    node->create_publisher<std_msgs::msg::String>(param.topics.ude_tune_status_text, 10);
  fsm.mocap_state_status_pub =
    node->create_publisher<std_msgs::msg::String>(param.topics.mocap_state_status, 10);
  fsm.expert_pose_pub =
    node->create_publisher<geometry_msgs::msg::PoseStamped>(param.topics.expert_pose, 10);
  fsm.traj_start_trigger_pub =
    node->create_publisher<geometry_msgs::msg::PoseStamped>(param.topics.traj_start_trigger, 10);
  fsm.gripper_cmd_pub =
    node->create_publisher<std_msgs::msg::Float64>(param.topics.gripper_command, 10);

  fsm.set_FCU_mode_srv =
    node->create_client<mavros_msgs::srv::SetMode>(param.services.set_mode);
  fsm.arming_client_srv =
    node->create_client<mavros_msgs::srv::CommandBool>(param.services.arming);
  fsm.reboot_FCU_srv =
    node->create_client<mavros_msgs::srv::CommandLong>(param.services.command);

  auto param_cb_handle = node->add_on_set_parameters_callback(
    [&fsm](const std::vector<rclcpp::Parameter> &params) {
      return fsm.runtime_param_cb(params);
    });
  (void)param_cb_handle;

  if (param.takeoff_land.no_RC) {
    RCLCPP_WARN(node->get_logger(), "[PX4CTRL] Remote controller disabled, be careful!");
  } else {
    RCLCPP_INFO(node->get_logger(), "[PX4CTRL] Waiting for RC");
    rclcpp::Rate wait_rate(10.0);
    while (rclcpp::ok() && !fsm.rc_is_received(node->now())) {
      rclcpp::spin_some(node);
      wait_rate.sleep();
    }
    if (rclcpp::ok()) {
      RCLCPP_INFO(node->get_logger(), "[PX4CTRL] RC received.");
    }
  }

  int trials = 0;
  rclcpp::Rate state_wait_rate(1.0);
  while (rclcpp::ok() && !fsm.state_data.current_state.connected) {
    rclcpp::spin_some(node);
    state_wait_rate.sleep();
    if (trials++ > 5) {
      RCLCPP_ERROR(node->get_logger(), "Unable to connect to PX4.");
      break;
    }
  }

  rclcpp::Rate rate(param.ctrl_freq_max);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    fsm.process();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
