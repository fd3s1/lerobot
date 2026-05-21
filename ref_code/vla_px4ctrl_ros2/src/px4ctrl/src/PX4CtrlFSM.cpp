#include "PX4CtrlFSM.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <quadrotor_msgs/msg/takeoff_land.hpp>
#include <uav_utils/utils.h>

using mavros_msgs::msg::PositionTarget;

PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, LinearControl &controller_, rclcpp::Node *node)
  : param(param_), controller(controller_), node_(node)
{
  state = MANUAL_CTRL;
  hover_pose.setZero();
}

void PX4CtrlFSM::process()
{
  const rclcpp::Time now_time = node_->now();
  Desired_State_t des(odom_data);
  bool rotor_low_speed_during_land = false;

  publish_gripper_safety();

  switch (state) {
    case MANUAL_CTRL: {
      if (rc_data.enter_hover_mode) {
        if (!odom_is_received(now_time)) {
          RCLCPP_ERROR(node_->get_logger(), "[px4ctrl] Reject AUTO_HOVER. No odom!");
          break;
        }
        if (cmd_is_received(now_time)) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "[px4ctrl] Reject AUTO_HOVER. Stop sending commands before entering hover.");
          break;
        }

        state = AUTO_HOVER;
        set_hov_with_odom();
        toggle_offboard_mode(true);
        RCLCPP_INFO(node_->get_logger(), "[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)");
      } else if (
        param.takeoff_land.enable && takeoff_land_data.triggered &&
        takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::msg::TakeoffLand::TAKEOFF) {
        if (!odom_is_received(now_time)) {
          RCLCPP_ERROR(node_->get_logger(), "[px4ctrl] Reject AUTO_TAKEOFF. No odom!");
          break;
        }
        if (cmd_is_received(now_time)) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "[px4ctrl] Reject AUTO_TAKEOFF. Stop sending commands before takeoff.");
          break;
        }
        if (!get_landed()) {
          RCLCPP_ERROR(node_->get_logger(), "[px4ctrl] Reject AUTO_TAKEOFF. Drone is not landed.");
          break;
        }
        if (rc_is_received(now_time)) {
          if (!rc_data.is_hover_mode || !rc_data.is_command_mode || !rc_data.check_centered()) {
            RCLCPP_ERROR(
              node_->get_logger(),
              "[px4ctrl] Reject AUTO_TAKEOFF. Keep RC in hover+command and sticks centered.");
            break;
          }
        }

        state = AUTO_TAKEOFF;
        set_start_pose_for_takeoff_land(odom_data);
        toggle_offboard_mode(true);
        if (param.takeoff_land.enable_auto_arm) {
          toggle_arm_disarm(true);
        }
        takeoff_land.toggle_takeoff_land_time = now_time;
        RCLCPP_INFO(node_->get_logger(), "[px4ctrl] MANUAL_CTRL(L1) --> AUTO_TAKEOFF");
      }

      if (rc_data.toggle_reboot) {
        if (state_data.current_state.armed) {
          RCLCPP_ERROR(node_->get_logger(), "[px4ctrl] Reject reboot! Disarm first.");
          break;
        }
        reboot_FCU();
      }

      break;
    }

    case AUTO_HOVER: {
      if (!rc_data.is_hover_mode || !odom_is_received(now_time)) {
        state = MANUAL_CTRL;
        toggle_offboard_mode(false);
        RCLCPP_WARN(node_->get_logger(), "[px4ctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
      } else if (rc_data.is_command_mode && cmd_is_received(now_time)) {
        if (state_data.current_state.mode == "OFFBOARD") {
          state = CMD_CTRL;
          des = get_cmd_des();
          RCLCPP_INFO(node_->get_logger(), "[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)");
        }
      } else if (
        takeoff_land_data.triggered &&
        takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::msg::TakeoffLand::LAND) {
        state = AUTO_LAND;
        set_start_pose_for_takeoff_land(odom_data);
        publish_gripper_target(param.gripper.open_position, true);
        RCLCPP_INFO(node_->get_logger(), "[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND");
      } else {
        set_hov_with_rc();
        des = get_hover_des();
        if (
          rc_data.enter_command_mode ||
          (takeoff_land.delay_trigger &&
           (now_time - takeoff_land.delay_trigger_time).seconds() > 0.0)) {
          takeoff_land.delay_trigger = false;
          publish_trigger(odom_data.msg);
          RCLCPP_INFO(node_->get_logger(), "[px4ctrl] TRIGGER sent, allow user command.");
        }
      }
      break;
    }

    case CMD_CTRL: {
      if (!rc_data.is_hover_mode || !odom_is_received(now_time)) {
        state = MANUAL_CTRL;
        toggle_offboard_mode(false);
        RCLCPP_WARN(node_->get_logger(), "[px4ctrl] CMD_CTRL(L3) --> MANUAL_CTRL(L1)");
      } else if (!rc_data.is_command_mode || !cmd_is_received(now_time)) {
        state = AUTO_HOVER;
        set_hov_with_odom();
        des = get_hover_des();
        RCLCPP_INFO(node_->get_logger(), "[px4ctrl] CMD_CTRL(L3) --> AUTO_HOVER(L2)");
      } else {
        des = get_cmd_des();
      }

      if (
        takeoff_land_data.triggered &&
        takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::msg::TakeoffLand::LAND) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "[px4ctrl] Reject AUTO_LAND in CMD_CTRL. Stop commands first to return to AUTO_HOVER.");
      }
      break;
    }

    case AUTO_TAKEOFF: {
      if (!odom_is_received(now_time)) {
        state = MANUAL_CTRL;
        toggle_offboard_mode(false);
        RCLCPP_WARN(node_->get_logger(), "[px4ctrl] AUTO_TAKEOFF --> MANUAL_CTRL, odom timeout.");
      } else if (
        (now_time - takeoff_land.toggle_takeoff_land_time).seconds() <
        AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) {
        des = get_rotor_speed_up_des(now_time);
      } else if (odom_data.p(2) >= takeoff_land.start_pose(2) + param.takeoff_land.height) {
        state = AUTO_HOVER;
        set_hov_with_odom();
        takeoff_land.delay_trigger = true;
        takeoff_land.delay_trigger_time =
          now_time + rclcpp::Duration::from_seconds(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);
        RCLCPP_INFO(node_->get_logger(), "[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)");
      } else {
        des = get_takeoff_land_des(param.takeoff_land.speed);
      }
      break;
    }

    case AUTO_LAND: {
      if (!rc_data.is_hover_mode || !odom_is_received(now_time)) {
        state = MANUAL_CTRL;
        toggle_offboard_mode(false);
        RCLCPP_WARN(node_->get_logger(), "[px4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)");
      } else if (!rc_data.is_command_mode) {
        state = AUTO_HOVER;
        set_hov_with_odom();
        des = get_hover_des();
        RCLCPP_INFO(node_->get_logger(), "[px4ctrl] AUTO_LAND --> AUTO_HOVER(L2)");
      } else if (!get_landed()) {
        des = get_takeoff_land_des(-param.takeoff_land.speed);
      } else {
        rotor_low_speed_during_land = true;
        if (
          extended_state_data.current_extended_state.landed_state ==
          mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND) {
          if (toggle_arm_disarm(false)) {
            state = MANUAL_CTRL;
            toggle_offboard_mode(false);
            RCLCPP_INFO(node_->get_logger(), "[px4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)");
          }
        }
      }
      break;
    }
  }

  if (rotor_low_speed_during_land) {
    motors_idling(des);
  }

  if (odom_is_received(now_time)) {
    const Desired_State_t safe_des = clamp_desired(des);
    const Controller_Output_t u = controller.calculateControl(safe_des, odom_data);
    publish_position_ctrl(u, now_time);
    publish_expert_pose(safe_des, now_time);
  }

  land_detector(state, des, odom_data);

  rc_data.enter_hover_mode = false;
  rc_data.enter_command_mode = false;
  rc_data.toggle_reboot = false;
  takeoff_land_data.triggered = false;
}

Desired_State_t PX4CtrlFSM::get_hover_des()
{
  Desired_State_t des;
  des.p = hover_pose.head<3>();
  des.yaw = hover_pose(3);
  return des;
}

Desired_State_t PX4CtrlFSM::get_cmd_des()
{
  Desired_State_t des;
  des.p = cmd_data.p;
  des.yaw = cmd_data.yaw;
  return des;
}

Desired_State_t PX4CtrlFSM::get_rotor_speed_up_des(const rclcpp::Time & /*now*/)
{
  Desired_State_t des;
  des.p = takeoff_land.start_pose.head<3>() + Eigen::Vector3d(0, 0, 0.2);
  des.yaw = takeoff_land.start_pose(3);
  return des;
}

Desired_State_t PX4CtrlFSM::get_takeoff_land_des(double speed)
{
  const rclcpp::Time now = node_->now();
  const double delta_t =
    (now - takeoff_land.toggle_takeoff_land_time).seconds() -
    (speed > 0 ? AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME : 0.0);

  Desired_State_t des;
  des.p = takeoff_land.start_pose.head<3>() + Eigen::Vector3d(0, 0, speed * delta_t);
  des.yaw = takeoff_land.start_pose(3);
  return des;
}

void PX4CtrlFSM::motors_idling(Desired_State_t &des)
{
  des.p = takeoff_land.start_pose.head<3>() + Eigen::Vector3d(0, 0, 0.05);
  des.yaw = takeoff_land.start_pose(3);
}

void PX4CtrlFSM::land_detector(State_t current_state, const Desired_State_t &des, const Odom_Data_t &odom)
{
  static State_t last_state = State_t::MANUAL_CTRL;
  if (last_state == State_t::MANUAL_CTRL &&
      (current_state == State_t::AUTO_HOVER || current_state == State_t::AUTO_TAKEOFF)) {
    takeoff_land.landed = false;
  }
  last_state = current_state;

  if (current_state == State_t::MANUAL_CTRL && !state_data.current_state.armed) {
    takeoff_land.landed = true;
    return;
  }

  constexpr double POSITION_DEVIATION_C = -0.5;
  constexpr double TIME_KEEP_C = 3.0;

  static rclcpp::Time time_C12_reached(0, 0, RCL_ROS_TIME);
  static bool is_last_C12_satisfy = false;
  if (takeoff_land.landed) {
    time_C12_reached = node_->now();
    is_last_C12_satisfy = false;
    return;
  }

  const bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C;
  if (C12_satisfy && !is_last_C12_satisfy) {
    time_C12_reached = node_->now();
  } else if (C12_satisfy && is_last_C12_satisfy) {
    if ((node_->now() - time_C12_reached).seconds() > TIME_KEEP_C) {
      takeoff_land.landed = true;
    }
  }
  is_last_C12_satisfy = C12_satisfy;
}

void PX4CtrlFSM::manual_flag_cb(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data == 0 || !traj_start_trigger_pub) {
    return;
  }

  geometry_msgs::msg::PoseStamped now_pose;
  now_pose.header.stamp = node_->now();
  now_pose.header.frame_id = param.frame_id;
  now_pose.pose.position.x = odom_data.p.x();
  now_pose.pose.position.y = odom_data.p.y();
  now_pose.pose.position.z = odom_data.p.z();
  now_pose.pose.orientation = odom_data.msg.pose.orientation;
  traj_start_trigger_pub->publish(now_pose);
}

void PX4CtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom)
{
  takeoff_land.start_pose.head<3>() = odom.p;
  takeoff_land.start_pose(3) =
    uav_utils::normalize_angle(uav_utils::get_yaw_from_quaternion(odom.q));
  takeoff_land.toggle_takeoff_land_time = node_->now();
}

void PX4CtrlFSM::set_hov_with_odom()
{
  hover_pose.head<3>() = odom_data.p;
  hover_pose(3) =
    uav_utils::normalize_angle(uav_utils::get_yaw_from_quaternion(odom_data.q));
  last_set_hover_pose_time = node_->now();
}

void PX4CtrlFSM::set_hov_with_rc()
{
  const rclcpp::Time now = node_->now();
  const double delta_t = std::max(0.0, (now - last_set_hover_pose_time).seconds());
  last_set_hover_pose_time = now;

  hover_pose(0) +=
    rc_data.ch[1] * param.max_manual_vel * delta_t * (param.rc_reverse.pitch ? 1.0 : -1.0);
  hover_pose(1) +=
    rc_data.ch[0] * param.max_manual_vel * delta_t * (param.rc_reverse.roll ? 1.0 : -1.0);
  hover_pose(2) +=
    rc_data.ch[2] * param.max_manual_vel * delta_t * (param.rc_reverse.throttle ? 1.0 : -1.0);
  hover_pose(3) = uav_utils::normalize_angle(
    hover_pose(3) +
    rc_data.ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw ? 1.0 : -1.0));

  hover_pose(0) = clamp(hover_pose(0), param.limits.x_min, param.limits.x_max);
  hover_pose(1) = clamp(hover_pose(1), param.limits.y_min, param.limits.y_max);
  hover_pose(2) = clamp(hover_pose(2), param.limits.z_min, param.limits.z_max);
}

bool PX4CtrlFSM::rc_is_received(const rclcpp::Time &now_time) const
{
  return rc_data.is_received(now_time, param.msg_timeout.rc);
}

bool PX4CtrlFSM::cmd_is_received(const rclcpp::Time &now_time) const
{
  return cmd_data.is_received(now_time, param.msg_timeout.cmd);
}

bool PX4CtrlFSM::odom_is_received(const rclcpp::Time &now_time) const
{
  return odom_data.is_received(now_time, param.msg_timeout.odom);
}

bool PX4CtrlFSM::bat_is_received(const rclcpp::Time &now_time) const
{
  return bat_data.is_received(now_time, param.msg_timeout.bat);
}

bool PX4CtrlFSM::recv_new_odom()
{
  if (odom_data.recv_new_msg) {
    odom_data.recv_new_msg = false;
    return true;
  }
  return false;
}

void PX4CtrlFSM::publish_position_ctrl(const Controller_Output_t &u, const rclcpp::Time &stamp)
{
  if (!ctrl_FCU_pub) {
    return;
  }

  PositionTarget msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = param.frame_id;
  msg.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
  msg.type_mask =
    PositionTarget::IGNORE_VX |
    PositionTarget::IGNORE_VY |
    PositionTarget::IGNORE_VZ |
    PositionTarget::IGNORE_AFX |
    PositionTarget::IGNORE_AFY |
    PositionTarget::IGNORE_AFZ |
    PositionTarget::IGNORE_YAW_RATE;

  msg.position.x = u.position.x();
  msg.position.y = u.position.y();
  msg.position.z = u.position.z();
  msg.yaw = static_cast<float>(uav_utils::normalize_angle(u.yaw));
  ctrl_FCU_pub->publish(msg);
}

void PX4CtrlFSM::publish_expert_pose(const Desired_State_t &des, const rclcpp::Time &stamp)
{
  if (!expert_pose_pub) {
    return;
  }

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = param.frame_id;
  msg.pose.position.x = des.p.x();
  msg.pose.position.y = des.p.y();
  msg.pose.position.z = des.p.z();

  const Eigen::Quaterniond q = uav_utils::yaw_to_quaternion(uav_utils::normalize_angle(des.yaw));
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();

  expert_pose_pub->publish(msg);
}

void PX4CtrlFSM::publish_trigger(const geometry_msgs::msg::PoseStamped &odom_msg)
{
  if (!traj_start_trigger_pub) {
    return;
  }
  traj_start_trigger_pub->publish(odom_msg);
}

void PX4CtrlFSM::publish_gripper_safety()
{
  if (should_force_gripper_open()) {
    publish_gripper_target(param.gripper.open_position);
    return;
  }

  publish_gripper_from_rc();
}

void PX4CtrlFSM::publish_gripper_from_rc()
{
  if (!gripper_cmd_pub || !rc_data.received) {
    return;
  }

  const double pwm = rc_data.channel_pwm(static_cast<std::size_t>(param.gripper.rc_channel));
  double target = last_gripper_target;
  bool has_new_target = false;

  if (pwm >= param.gripper.pwm_close) {
    target = param.gripper.closed_position;
    has_new_target = true;
  } else if (pwm <= param.gripper.pwm_open) {
    target = param.gripper.open_position;
    has_new_target = true;
  }

  if (!has_new_target) {
    return;
  }

  publish_gripper_target(target);
}

void PX4CtrlFSM::publish_gripper_target(double target, bool force)
{
  if (!gripper_cmd_pub) {
    return;
  }

  target = clamp(target, 0.0, 100.0);
  if (!force && have_gripper_target && std::abs(target - last_gripper_target) < 1e-6) {
    return;
  }

  std_msgs::msg::Float64 msg;
  msg.data = target;
  gripper_cmd_pub->publish(msg);
  have_gripper_target = true;
  last_gripper_target = target;
}

bool PX4CtrlFSM::px4_mode_allows_gripper_rc() const
{
  const std::string &mode = state_data.current_state.mode;
  return mode == "POSCTL" || mode == "OFFBOARD";
}

bool PX4CtrlFSM::should_force_gripper_open() const
{
  const bool land_requested =
    takeoff_land_data.triggered &&
    takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::msg::TakeoffLand::LAND;
  const bool below_safe_height =
    odom_data.received && odom_data.p.z() <= param.gripper.force_open_below_z;

  return land_requested || state == AUTO_TAKEOFF || state == AUTO_LAND ||
         below_safe_height || !state_data.current_state.armed || !px4_mode_allows_gripper_rc();
}

bool PX4CtrlFSM::toggle_offboard_mode(bool on_off)
{
  if (!set_FCU_mode_srv || !set_FCU_mode_srv->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "[px4ctrl] set_mode service is not ready.");
    return false;
  }

  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  if (on_off) {
    state_data.state_before_offboard = state_data.current_state;
    if (state_data.state_before_offboard.mode == "OFFBOARD" ||
        state_data.state_before_offboard.mode.empty()) {
      state_data.state_before_offboard.mode = "MANUAL";
    }
    request->custom_mode = "OFFBOARD";
  } else {
    request->custom_mode = state_data.state_before_offboard.mode.empty()
      ? std::string("MANUAL")
      : state_data.state_before_offboard.mode;
  }

  set_FCU_mode_srv->async_send_request(request);
  return true;
}

bool PX4CtrlFSM::toggle_arm_disarm(bool arm)
{
  if (!arming_client_srv || !arming_client_srv->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "[px4ctrl] arming service is not ready.");
    return false;
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = arm;
  arming_client_srv->async_send_request(request);
  return true;
}

void PX4CtrlFSM::reboot_FCU()
{
  if (!reboot_FCU_srv || !reboot_FCU_srv->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "[px4ctrl] command service is not ready.");
    return;
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  request->broadcast = false;
  request->command = 246;
  request->param1 = 1.0;
  reboot_FCU_srv->async_send_request(request);
}

double PX4CtrlFSM::clamp(double value, double low, double high) const
{
  return std::min(high, std::max(low, value));
}

Desired_State_t PX4CtrlFSM::clamp_desired(const Desired_State_t &des) const
{
  Desired_State_t safe = des;
  safe.p.x() = clamp(safe.p.x(), param.limits.x_min, param.limits.x_max);
  safe.p.y() = clamp(safe.p.y(), param.limits.y_min, param.limits.y_max);
  safe.p.z() = clamp(safe.p.z(), param.limits.z_min, param.limits.z_max);
  safe.yaw = uav_utils::normalize_angle(safe.yaw);
  return safe;
}
