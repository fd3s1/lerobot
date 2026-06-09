#include "PX4CtrlFSM.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <quadrotor_msgs/msg/takeoff_land.hpp>
#include <uav_utils/utils.h>

using mavros_msgs::msg::AttitudeTarget;

namespace {

Eigen::Vector3d limit_norm(const Eigen::Vector3d &value, double max_norm)
{
  if (max_norm <= 0.0) {
    return Eigen::Vector3d::Zero();
  }
  const double norm = value.norm();
  if (norm <= max_norm || norm < 1e-6) {
    return value;
  }
  return value * (max_norm / norm);
}

void set_vector3(geometry_msgs::msg::Vector3 &msg, const Eigen::Vector3d &value)
{
  msg.x = value.x();
  msg.y = value.y();
  msg.z = value.z();
}

void set_point(geometry_msgs::msg::Point &msg, const Eigen::Vector3d &value)
{
  msg.x = value.x();
  msg.y = value.y();
  msg.z = value.z();
}

void set_quaternion(geometry_msgs::msg::Quaternion &msg, const Eigen::Quaterniond &value)
{
  msg.x = value.x();
  msg.y = value.y();
  msg.z = value.z();
  msg.w = value.w();
}

Eigen::Vector3d quaternion_to_rpy(const Eigen::Quaterniond &q)
{
  const double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
  const double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  const double pitch = std::abs(sinp) >= 1.0 ?
    std::copysign(1.5707963267948966, sinp) :
    std::asin(sinp);

  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  const double yaw = std::atan2(siny_cosp, cosy_cosp);
  return Eigen::Vector3d(roll, pitch, yaw);
}

void append_vector(std::vector<double> &data, const Eigen::Vector3d &value)
{
  data.push_back(value.x());
  data.push_back(value.y());
  data.push_back(value.z());
}

void append_array(std::vector<double> &data, const std::array<double, 3> &value)
{
  data.push_back(value[0]);
  data.push_back(value[1]);
  data.push_back(value[2]);
}

bool is_nan(double value)
{
  return std::isnan(value);
}

void add_array_param_from_tune(
  std::vector<rclcpp::Parameter> &updates,
  const std::vector<double> &data,
  std::size_t offset,
  const char *name)
{
  if (data.size() <= offset + 2) {
    return;
  }
  if (is_nan(data[offset]) || is_nan(data[offset + 1]) || is_nan(data[offset + 2])) {
    return;
  }
  updates.emplace_back(
    name,
    std::vector<double>{data[offset], data[offset + 1], data[offset + 2]});
}

}  // namespace

PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, LinearControl &controller_, rclcpp::Node *node)
  : param(param_), controller(controller_), node_(node)
{
  state = MANUAL_CTRL;
  hover_pose.setZero();
  fsm_state_pub = node_->create_publisher<std_msgs::msg::String>("/px4ctrl/state", 1);
}

void PX4CtrlFSM::process()
{
  const rclcpp::Time now_time = node_->now();
  Desired_State_t des(odom_data);
  bool rotor_low_speed_during_land = false;
  bool rotor_speedup_during_takeoff = false;

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

        change_state(AUTO_HOVER);
        set_hov_with_odom();
        toggle_offboard_mode(true);
        RCLCPP_INFO(node_->get_logger(), "\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[0m");
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

        change_state(AUTO_TAKEOFF);
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
        change_state(MANUAL_CTRL);
        toggle_offboard_mode(false);
        publish_gripper_force_open(now_time);
        RCLCPP_WARN(node_->get_logger(), "\033[31m[px4ctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)\033[0m");
      } else if (rc_data.is_command_mode && cmd_is_received(now_time)) {
        if (state_data.current_state.mode == "OFFBOARD") {
          change_state(CMD_CTRL);
          des = get_cmd_des();
          RCLCPP_INFO(node_->get_logger(), "\033[31m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[0m");
        }
      } else if (
        takeoff_land_data.triggered &&
        takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::msg::TakeoffLand::LAND) {
        change_state(AUTO_LAND);
        set_start_pose_for_takeoff_land(odom_data);
        publish_gripper_target(param.gripper.open_position, true);
        RCLCPP_INFO(node_->get_logger(), "\033[31m[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND\033[0m");
      } else {
        set_hov_with_rc();
        des = get_hover_des();
        if (
          rc_data.enter_command_mode ||
          (takeoff_land.delay_trigger &&
           (now_time - takeoff_land.delay_trigger_time).seconds() > 0.0)) {
          takeoff_land.delay_trigger = false;
          publish_trigger(odom_data, now_time);
          RCLCPP_INFO(node_->get_logger(), "[px4ctrl] TRIGGER sent, allow user command.");
        }
      }
      break;
    }

    case CMD_CTRL: {
      if (!rc_data.is_hover_mode || !odom_is_received(now_time)) {
        change_state(MANUAL_CTRL);
        toggle_offboard_mode(false);
        RCLCPP_WARN(node_->get_logger(), "[px4ctrl] CMD_CTRL(L3) --> MANUAL_CTRL(L1)");
      } else if (!rc_data.is_command_mode || !cmd_is_received(now_time)) {
        change_state(AUTO_HOVER);
        set_hov_with_odom();
        des = get_hover_des();
        RCLCPP_INFO(node_->get_logger(), "\033[32m[px4ctrl] CMD_CTRL(L3) --> AUTO_HOVER(L2)\033[0m");
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
        change_state(MANUAL_CTRL);
        toggle_offboard_mode(false);
        RCLCPP_WARN(node_->get_logger(), "[px4ctrl] AUTO_TAKEOFF --> MANUAL_CTRL, odom timeout.");
      } else if (
        (now_time - takeoff_land.toggle_takeoff_land_time).seconds() <
        AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) {
        rotor_speedup_during_takeoff = true;
        des = get_rotor_speed_up_des(now_time);
      } else if (odom_data.p(2) >= takeoff_land.start_pose(2) + param.takeoff_land.height) {
        change_state(AUTO_HOVER);
        set_hov_with_odom();
        takeoff_land.delay_trigger = true;
        takeoff_land.delay_trigger_time =
          now_time + rclcpp::Duration::from_seconds(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);
        RCLCPP_INFO(node_->get_logger(), "\033[32m[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[0m");
      } else {
        des = get_takeoff_land_des(param.takeoff_land.speed);
      }
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        500,
        "[px4ctrl] AUTO_TAKEOFF phase=%s odom=(%.3f,%.3f,%.3f) start=(%.3f,%.3f,%.3f) "
        "des=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f,%.3f)",
        rotor_speedup_during_takeoff ? "speedup" : "climb",
        odom_data.p.x(),
        odom_data.p.y(),
        odom_data.p.z(),
        takeoff_land.start_pose.x(),
        takeoff_land.start_pose.y(),
        takeoff_land.start_pose.z(),
        des.p.x(),
        des.p.y(),
        des.p.z(),
        odom_data.v.x(),
        odom_data.v.y(),
        odom_data.v.z());
      break;
    }

    case AUTO_LAND: {
      if (!rc_data.is_hover_mode || !odom_is_received(now_time)) {
        change_state(MANUAL_CTRL);
        toggle_offboard_mode(false);
        RCLCPP_WARN(node_->get_logger(), "[px4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)");
      } else if (!rc_data.is_command_mode) {
        change_state(AUTO_HOVER);
        set_hov_with_odom();
        des = get_hover_des();
        RCLCPP_INFO(node_->get_logger(), "\033[32m[px4ctrl] AUTO_LAND --> AUTO_HOVER(L2)\033[0m");
      } else if (!get_landed()) {
        des = get_takeoff_land_des(-param.takeoff_land.speed);
      } else {
        rotor_low_speed_during_land = true;
        if (
          extended_state_data.current_extended_state.landed_state ==
          mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND) {
          if (toggle_arm_disarm(false)) {
            change_state(MANUAL_CTRL);
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

  publish_gripper_safety(now_time);

  const bool control_feedback_valid = odom_is_received(now_time) && imu_is_received(now_time);
  if (!control_feedback_valid) {
    if (had_valid_control_feedback) {
      controller.resetControlState();
    }
    had_valid_control_feedback = false;
  } else {
    if (!had_valid_control_feedback) {
      controller.resetControlState();
    }
    had_valid_control_feedback = true;

    if (
      param.thrust_model.enable_estimation &&
      (state == AUTO_HOVER || state == CMD_CTRL)) {
      controller.estimateThrustModel(imu_data.a, now_time);
    }

    const Desired_State_t safe_des = clamp_desired(des);
    Controller_Output_t u;
    Controller_Debug_t debug;
    bool have_debug = false;
    if (state == MANUAL_CTRL || rotor_low_speed_during_land) {
      u.q = odom_data.q;
      u.bodyrates.setZero();
      u.thrust = clamp(
        param.thrust_model.hover_thrust,
        param.controller.min_thrust,
        param.controller.max_thrust);
      controller.resetControlState();
    } else if (rotor_speedup_during_takeoff) {
      const double elapsed = (now_time - takeoff_land.toggle_takeoff_land_time).seconds();
      const double ratio = clamp(
        elapsed / std::max(AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME, 1e-3),
        0.0,
        1.0);
      const double ramp_thrust = clamp(
        param.controller.min_thrust +
        ratio * (param.thrust_model.hover_thrust - param.controller.min_thrust),
        param.controller.min_thrust,
        param.controller.max_thrust);

      controller.resetControlState();
      u = controller.calculateControl(safe_des, odom_data, imu_data, now_time, &debug);
      have_debug = true;
      u.thrust = clamp(
        std::min(u.thrust, ramp_thrust),
        param.controller.min_thrust,
        param.controller.max_thrust);
      debug.thrust = u.thrust;
      controller.resetControlState();
    } else {
      u = controller.calculateControl(safe_des, odom_data, imu_data, now_time, &debug);
      have_debug = true;
    }
    if (state == AUTO_TAKEOFF) {
      const Eigen::Vector3d cmd_rpy = quaternion_to_rpy(u.q);
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        500,
        "[px4ctrl] AUTO_TAKEOFF output err=(%.3f,%.3f,%.3f) cmd_rpy=(%.3f,%.3f,%.3f) "
        "thrust=%.3f bodyrate=(%.3f,%.3f,%.3f)",
        safe_des.p.x() - odom_data.p.x(),
        safe_des.p.y() - odom_data.p.y(),
        safe_des.p.z() - odom_data.p.z(),
        cmd_rpy.x(),
        cmd_rpy.y(),
        cmd_rpy.z(),
        u.thrust,
        u.bodyrates.x(),
        u.bodyrates.y(),
        u.bodyrates.z());
    }
    publish_ctrl(u, now_time);
    publish_expert_pose(safe_des, now_time);
    publish_simulink_reference(safe_des, now_time);
    publish_simulink_actual(odom_data, now_time);
    publish_simulink_tracking_error(safe_des, odom_data, now_time);
    if (have_debug) {
      publish_simulink_ude_debug(debug, now_time);
    }
  }

  land_detector(state, des, odom_data);
  publish_fsm_state();

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
  if (param.cmd_feedforward.enable) {
    des.v = limit_norm(cmd_data.v, param.cmd_feedforward.max_velocity);
    des.a = limit_norm(cmd_data.a, param.cmd_feedforward.max_acceleration);
  } else {
    des.v.setZero();
    des.a.setZero();
  }
  des.yaw = cmd_data.yaw;
  return des;
}

Desired_State_t PX4CtrlFSM::get_rotor_speed_up_des(const rclcpp::Time & /*now*/)
{
  Desired_State_t des;
  des.p = takeoff_land.start_pose.head<3>();
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
  des.v = Eigen::Vector3d(0.0, 0.0, speed);
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
  now_pose.pose.orientation = odom_data.msg.pose.pose.orientation;
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

bool PX4CtrlFSM::imu_is_received(const rclcpp::Time &now_time) const
{
  return imu_data.is_received(now_time, param.msg_timeout.imu);
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

void PX4CtrlFSM::publish_ctrl(const Controller_Output_t &u, const rclcpp::Time &stamp)
{
  if (!ctrl_FCU_pub) {
    return;
  }

  AttitudeTarget msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = param.frame_id;
  if (param.use_bodyrate_ctrl) {
    msg.type_mask = AttitudeTarget::IGNORE_ATTITUDE;
    msg.body_rate.x = u.bodyrates.x();
    msg.body_rate.y = u.bodyrates.y();
    msg.body_rate.z = u.bodyrates.z();
  } else {
    msg.type_mask =
      AttitudeTarget::IGNORE_ROLL_RATE |
      AttitudeTarget::IGNORE_PITCH_RATE |
      AttitudeTarget::IGNORE_YAW_RATE;
    msg.orientation.x = u.q.x();
    msg.orientation.y = u.q.y();
    msg.orientation.z = u.q.z();
    msg.orientation.w = u.q.w();
    msg.body_rate.x = 0.0;
    msg.body_rate.y = 0.0;
    msg.body_rate.z = 0.0;
  }
  msg.thrust = static_cast<float>(std::clamp(u.thrust, 0.0, 1.0));
  ctrl_FCU_pub->publish(msg);

  if (simulink_setpoint_pub) {
    nav_msgs::msg::Odometry out;
    out.header = msg.header;
    out.child_frame_id = param.use_bodyrate_ctrl ? "bodyrate_setpoint" : "attitude_setpoint";
    out.pose.pose.orientation = msg.orientation;
    out.twist.twist.angular = msg.body_rate;
    out.twist.twist.linear.x = static_cast<double>(msg.thrust);
    out.twist.twist.linear.y = static_cast<double>(msg.type_mask);
    out.twist.twist.linear.z = param.use_bodyrate_ctrl ? 1.0 : 0.0;
    simulink_setpoint_pub->publish(out);
  }
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

void PX4CtrlFSM::publish_simulink_reference(
  const Desired_State_t &des,
  const rclcpp::Time &stamp)
{
  if (!simulink_reference_pub) {
    return;
  }

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = param.frame_id;
  msg.child_frame_id = "reference_state";
  set_point(msg.pose.pose.position, des.p);
  set_vector3(msg.twist.twist.linear, des.v);
  msg.twist.twist.angular.z = des.yaw_rate;

  const Eigen::Quaterniond q = uav_utils::yaw_to_quaternion(uav_utils::normalize_angle(des.yaw));
  set_quaternion(msg.pose.pose.orientation, q);

  simulink_reference_pub->publish(msg);
}

void PX4CtrlFSM::publish_simulink_actual(
  const Odom_Data_t &odom,
  const rclcpp::Time &stamp)
{
  if (!simulink_actual_pub) {
    return;
  }

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = param.frame_id;
  msg.child_frame_id = "actual_state_world_velocity";
  set_point(msg.pose.pose.position, odom.p);
  set_quaternion(msg.pose.pose.orientation, odom.q);
  set_vector3(msg.twist.twist.linear, odom.v);
  set_vector3(msg.twist.twist.angular, odom.w);
  simulink_actual_pub->publish(msg);
}

void PX4CtrlFSM::publish_simulink_tracking_error(
  const Desired_State_t &des,
  const Odom_Data_t &odom,
  const rclcpp::Time &stamp)
{
  if (!simulink_tracking_error_pub) {
    return;
  }

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = param.frame_id;
  msg.child_frame_id = "tracking_error_des_minus_odom";
  set_point(msg.pose.pose.position, des.p - odom.p);
  set_vector3(msg.twist.twist.linear, des.v - odom.v);

  const double odom_yaw = uav_utils::normalize_angle(uav_utils::get_yaw_from_quaternion(odom.q));
  const double yaw_error = uav_utils::normalize_angle(des.yaw - odom_yaw);
  const Eigen::Quaterniond yaw_error_q = uav_utils::yaw_to_quaternion(yaw_error);
  set_quaternion(msg.pose.pose.orientation, yaw_error_q);
  msg.twist.twist.angular.z = des.yaw_rate - odom.w.z();

  simulink_tracking_error_pub->publish(msg);
}

void PX4CtrlFSM::publish_simulink_ude_debug(
  const Controller_Debug_t &debug,
  const rclcpp::Time &stamp)
{
  if (!simulink_ude_debug_pub) {
    return;
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.layout.dim.resize(1);
  msg.layout.dim[0].label = "ude_debug_v1";
  msg.layout.dim[0].size = 49;
  msg.layout.dim[0].stride = 49;
  msg.layout.data_offset = 0;
  msg.data.reserve(49);

  msg.data.push_back(stamp.seconds());
  msg.data.push_back(static_cast<double>(state));
  append_vector(msg.data, debug.des_p);
  append_vector(msg.data, debug.odom_p);
  append_vector(msg.data, debug.e);
  append_vector(msg.data, debug.des_v);
  append_vector(msg.data, debug.odom_v);
  append_vector(msg.data, debug.e_dot);
  append_vector(msg.data, debug.u0);
  append_vector(msg.data, debug.integral_u0);
  append_vector(msg.data, debug.f_hat);
  append_vector(msg.data, debug.u_acc);
  append_vector(msg.data, debug.thrust_acc_limited);
  append_vector(msg.data, debug.bodyrates_ff);
  append_vector(msg.data, debug.bodyrates_fb);
  append_vector(msg.data, debug.bodyrates_cmd);
  msg.data.push_back(debug.thrust);
  msg.data.push_back(debug.yaw_des);
  msg.data.push_back(debug.yaw_odom);
  msg.data.push_back(debug.yaw_error);
  msg.data.push_back(debug.dt);

  simulink_ude_debug_pub->publish(msg);
}

void PX4CtrlFSM::publish_ude_tune_status(
  double seq,
  bool accepted,
  double code,
  const std::string &text)
{
  if (ude_tune_status_pub) {
    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "ude_tune_status_v1";
    msg.layout.dim[0].size = 12;
    msg.layout.dim[0].stride = 12;
    msg.layout.data_offset = 0;
    msg.data.reserve(12);
    msg.data.push_back(seq);
    msg.data.push_back(accepted ? 1.0 : 0.0);
    msg.data.push_back(code);
    append_array(msg.data, param.ude.Kp_diag);
    append_array(msg.data, param.ude.Kd_diag);
    append_array(msg.data, param.ude.T_diag);
    ude_tune_status_pub->publish(msg);
  }

  if (ude_tune_status_text_pub) {
    std_msgs::msg::String msg;
    std::ostringstream oss;
    oss << "seq=" << seq << " accepted=" << (accepted ? "true" : "false")
        << " code=" << code << " " << text;
    msg.data = oss.str();
    ude_tune_status_text_pub->publish(msg);
  }
}

void PX4CtrlFSM::publish_trigger(const Odom_Data_t &odom, const rclcpp::Time &stamp)
{
  if (!traj_start_trigger_pub) {
    return;
  }
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = param.frame_id;
  msg.pose.position.x = odom.p.x();
  msg.pose.position.y = odom.p.y();
  msg.pose.position.z = odom.p.z();
  msg.pose.orientation = odom.msg.pose.pose.orientation;
  traj_start_trigger_pub->publish(msg);
}

void PX4CtrlFSM::publish_fsm_state()
{
  if (!fsm_state_pub) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = state_to_string(state);
  fsm_state_pub->publish(msg);
}

void PX4CtrlFSM::publish_gripper_safety(const rclcpp::Time &now_time)
{
  if (should_force_gripper_open(now_time)) {
    publish_gripper_force_open(now_time);
    return;
  }

  publish_gripper_from_rc();
}

void PX4CtrlFSM::publish_gripper_from_rc()
{
  if (!gripper_cmd_pub || !rc_data.received || param.gripper.rc_channel <= 0) {
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

void PX4CtrlFSM::publish_gripper_force_open(const rclcpp::Time &now_time)
{
  if (!gripper_cmd_pub) {
    return;
  }

  const bool first_force_open = last_gripper_force_open_time.nanoseconds() == 0;
  const bool retry_due =
    !first_force_open && (now_time - last_gripper_force_open_time).seconds() >= 0.2;
  const bool target_not_open =
    !have_gripper_target || std::abs(last_gripper_target - param.gripper.open_position) > 1e-6;

  if (first_force_open || retry_due || target_not_open) {
    publish_gripper_target(param.gripper.open_position, true);
    last_gripper_force_open_time = now_time;
  }
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

bool PX4CtrlFSM::should_force_gripper_open(const rclcpp::Time &now_time) const
{
  const bool land_requested =
    takeoff_land_data.triggered &&
    takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::msg::TakeoffLand::LAND;
  const bool odom_timeout = !odom_is_received(now_time);
  const bool rc_timeout = !rc_is_received(now_time);
  const bool below_safe_height =
    odom_data.received && odom_data.p.z() <= param.gripper.force_open_below_z;

  return land_requested || state == AUTO_TAKEOFF || state == AUTO_LAND ||
         odom_timeout || rc_timeout || below_safe_height ||
         !state_data.current_state.armed || !px4_mode_allows_gripper_rc();
}

void PX4CtrlFSM::ude_tune_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  constexpr double kMalformedCode = -1.0;
  constexpr double kRejectedCode = 0.0;
  constexpr double kAcceptedCode = 1.0;

  const std::vector<double> &data = msg->data;
  const double seq = data.empty() ? -1.0 : data[0];
  if (data.size() < 2) {
    publish_ude_tune_status(
      seq,
      false,
      kMalformedCode,
      "ude_tune expects at least seq and reset_control.");
    return;
  }

  std::vector<rclcpp::Parameter> updates;
  add_array_param_from_tune(updates, data, 2, "ude.Kp_diag");
  add_array_param_from_tune(updates, data, 5, "ude.Kd_diag");
  add_array_param_from_tune(updates, data, 8, "ude.T_diag");

  bool thrust_mapping_changed = false;
  const auto result = param.apply_runtime_parameters(updates, &thrust_mapping_changed);
  if (!result.successful) {
    publish_ude_tune_status(seq, false, kRejectedCode, result.reason);
    return;
  }

  const bool reset_control = data[1] >= 0.5;
  if (reset_control) {
    controller.resetControlState();
  }
  if (thrust_mapping_changed) {
    controller.resetThrustMapping();
  }

  std::ostringstream oss;
  oss << result.reason << "; updates=" << updates.size()
      << " reset_control=" << (reset_control ? "true" : "false");
  publish_ude_tune_status(seq, true, kAcceptedCode, oss.str());
}

rcl_interfaces::msg::SetParametersResult PX4CtrlFSM::runtime_param_cb(
  const std::vector<rclcpp::Parameter> &params)
{
  bool thrust_mapping_changed = false;
  const auto result = param.apply_runtime_parameters(params, &thrust_mapping_changed);
  if (result.successful) {
    controller.resetControlState();
    if (thrust_mapping_changed) {
      controller.resetThrustMapping();
    }
  }
  publish_ude_tune_status(
    -1.0,
    result.successful,
    result.successful ? 1.0 : 0.0,
    result.reason);
  return result;
}

void PX4CtrlFSM::change_state(State_t new_state)
{
  if (state != new_state) {
    controller.resetControlState();
    had_valid_control_feedback = false;
  }
  state = new_state;
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

const char *PX4CtrlFSM::state_to_string(State_t state) const
{
  switch (state) {
    case MANUAL_CTRL:
      return "MANUAL_CTRL";
    case AUTO_HOVER:
      return "AUTO_HOVER";
    case CMD_CTRL:
      return "CMD_CTRL";
    case AUTO_TAKEOFF:
      return "AUTO_TAKEOFF";
    case AUTO_LAND:
      return "AUTO_LAND";
  }
  return "UNKNOWN";
}
