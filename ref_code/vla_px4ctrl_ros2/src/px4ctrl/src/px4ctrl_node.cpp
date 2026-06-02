#include <chrono>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/takeoff_land.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
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

  LinearControl controller(param);
  PX4CtrlFSM fsm(param, controller, node.get());

  auto state_sub = node->create_subscription<mavros_msgs::msg::State>(
    param.topics.state,
    10,
    [&fsm](const mavros_msgs::msg::State::SharedPtr msg) {
      fsm.state_data.feed(msg);
    });

  auto extended_state_sub = node->create_subscription<mavros_msgs::msg::ExtendedState>(
    param.topics.extended_state,
    10,
    [&fsm](const mavros_msgs::msg::ExtendedState::SharedPtr msg) {
      fsm.extended_state_data.feed(msg);
    });

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    param.topics.odom,
    100,
    [&fsm, &node](const nav_msgs::msg::Odometry::SharedPtr msg) {
      fsm.odom_data.feed(msg, node->now());
    });

  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
    param.topics.imu,
    100,
    [&fsm, &node](const sensor_msgs::msg::Imu::SharedPtr msg) {
      fsm.imu_data.feed(msg, node->now());
    });

  auto cmd_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    param.topics.cmd,
    100,
    [&fsm, &node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      fsm.cmd_data.feed(msg, node->now());
    });

  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub;
  if (!param.takeoff_land.no_RC) {
    rc_sub = node->create_subscription<mavros_msgs::msg::RCIn>(
      param.topics.rc,
      10,
      [&fsm, &node](const mavros_msgs::msg::RCIn::SharedPtr msg) {
        fsm.rc_data.feed(msg, node->now());
      });
  }

  auto bat_sub = node->create_subscription<sensor_msgs::msg::BatteryState>(
    param.topics.battery,
    100,
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

  fsm.ctrl_FCU_pub =
    node->create_publisher<mavros_msgs::msg::AttitudeTarget>(param.topics.setpoint, 10);
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
