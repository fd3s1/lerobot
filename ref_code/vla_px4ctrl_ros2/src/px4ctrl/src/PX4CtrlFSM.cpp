#include "PX4CtrlFSM.h"

#include <array>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <mavros/frame_tf.hpp>
#include <quadrotor_msgs/msg/takeoff_land.hpp>
#include <uav_utils/utils.h>

#include "physical_setpoint_protocol.h"

using mavros_msgs::msg::AttitudeTarget;

namespace {

Eigen::Vector3d limit_norm(const Eigen::Vector3d &value, double max_norm)
{
  if (!value.allFinite() || !std::isfinite(max_norm) || max_norm <= 0.0) {
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

bool vector_finite(const Eigen::Vector3d &value)
{
  return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

bool quaternion_finite(const Eigen::Quaterniond &value)
{
  return value.coeffs().allFinite() && std::isfinite(value.norm()) && value.norm() > 1e-6;
}

double abs_time_diff_s(const rclcpp::Time &lhs, const rclcpp::Time &rhs)
{
  return std::abs((lhs - rhs).seconds());
}

std::string lower_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
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

double yaw_from_geometry_quaternion(const geometry_msgs::msg::Quaternion &msg)
{
  Eigen::Quaterniond q(msg.w, msg.x, msg.y, msg.z);
  if (q.norm() <= 1e-6) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  q.normalize();
  return uav_utils::normalize_angle(uav_utils::get_yaw_from_quaternion(q));
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
  const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
  fsm_state_pub = node_->create_publisher<std_msgs::msg::String>("/px4ctrl/state", state_qos);
  if (param.ctrl_freq_max <= 0.0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[px4ctrl] TD disabled because ctrl_freq_max=%.6f is invalid.",
      param.ctrl_freq_max);
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "[px4ctrl] TD h=%.6fs from ctrl_freq_max=%.3fHz enable=%s r=(%.3f,%.3f,%.3f)",
      td_h(),
      param.ctrl_freq_max,
      param.td.enable ? "true" : "false",
      param.td.r_diag[0],
      param.td.r_diag[1],
      param.td.r_diag[2]);
  }
}

void PX4CtrlFSM::process()
{
  const rclcpp::Time now_time = node_->now();
  if (!odom_is_received(now_time)) {
    reset_td_tracker("odom timeout");
  }

  control_odom_data = build_control_odom(now_time);
  publish_mocap_state_status(now_time);

  Desired_State_t des(control_odom_data);
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
        set_hov_with_odom(control_odom_data);
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
          if (
            !rc_data.is_hover_mode || !rc_data.is_command_mode ||
            !rc_data.check_takeoff_sticks(param.takeoff_land.auto_arm_throttle_max_pwm)) {
            RCLCPP_ERROR(
              node_->get_logger(),
              "[px4ctrl] Reject AUTO_TAKEOFF. Keep RC in hover+command, roll/pitch/yaw "
              "centered, and throttle low. ch=(%.3f,%.3f,%.3f,%.3f) ch3_pwm=%.0f "
              "throttle_max_pwm=%.0f mode=%.3f gear=%.3f",
              rc_data.ch[0],
              rc_data.ch[1],
              rc_data.ch[2],
              rc_data.ch[3],
              rc_data.channel_pwm(3),
              param.takeoff_land.auto_arm_throttle_max_pwm,
              rc_data.mode,
              rc_data.gear);
            break;
          }
        }

        change_state(AUTO_TAKEOFF);
        set_start_pose_for_takeoff_land(control_odom_data);
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
          const double rc_age_s = rc_data.received ?
            (now_time - rc_data.rcv_stamp).seconds() :
            std::numeric_limits<double>::infinity();
          const double cmd_age_s = cmd_data.received ?
            (now_time - cmd_data.rcv_stamp).seconds() :
            std::numeric_limits<double>::infinity();
          change_state(CMD_CTRL);
          des = get_cmd_des();
          RCLCPP_INFO(
            node_->get_logger(),
            "\033[31m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3) "
            "rc_age=%.3fs cmd_age=%.3fs mode=%.3f gear=%.3f\033[0m",
            rc_age_s,
            cmd_age_s,
            rc_data.mode,
            rc_data.gear);
        }
      } else if (
        takeoff_land_data.triggered &&
        takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::msg::TakeoffLand::LAND) {
        change_state(AUTO_LAND);
        set_start_pose_for_takeoff_land(control_odom_data);
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
          publish_trigger(control_odom_data, now_time);
          RCLCPP_INFO(node_->get_logger(), "[px4ctrl] TRIGGER sent, allow user command.");
        }
      }
      break;
    }

    case CMD_CTRL: {
      const bool rc_fresh = rc_is_received(now_time);
      const bool odom_fresh = odom_is_received(now_time);
      const bool cmd_fresh = cmd_is_received(now_time);
      const double rc_age_s = rc_data.received ?
        (now_time - rc_data.rcv_stamp).seconds() :
        std::numeric_limits<double>::infinity();
      const double cmd_age_s = cmd_data.received ?
        (now_time - cmd_data.rcv_stamp).seconds() :
        std::numeric_limits<double>::infinity();
      const double odom_age_s = odom_data.received ?
        (now_time - odom_data.rcv_stamp).seconds() :
        std::numeric_limits<double>::infinity();

      if (!rc_data.is_hover_mode || !odom_fresh) {
        change_state(MANUAL_CTRL);
        toggle_offboard_mode(false);
        RCLCPP_WARN(
          node_->get_logger(),
          "[px4ctrl] CMD_CTRL(L3) --> MANUAL_CTRL(L1): hover_mode=%d odom_fresh=%d "
          "odom_age=%.3fs rc_fresh=%d rc_age=%.3fs mode=%.3f gear=%.3f",
          rc_data.is_hover_mode ? 1 : 0,
          odom_fresh ? 1 : 0,
          odom_age_s,
          rc_fresh ? 1 : 0,
          rc_age_s,
          rc_data.mode,
          rc_data.gear);
      } else if (!rc_data.is_command_mode || !cmd_fresh) {
        change_state(AUTO_HOVER);
        set_hov_with_odom(control_odom_data);
        des = get_hover_des();
        RCLCPP_INFO(
          node_->get_logger(),
          "\033[32m[px4ctrl] CMD_CTRL(L3) --> AUTO_HOVER(L2): command_mode=%d "
          "cmd_fresh=%d cmd_age=%.3fs rc_fresh=%d rc_age=%.3fs mode=%.3f gear=%.3f\033[0m",
          rc_data.is_command_mode ? 1 : 0,
          cmd_fresh ? 1 : 0,
          cmd_age_s,
          rc_fresh ? 1 : 0,
          rc_age_s,
          rc_data.mode,
          rc_data.gear);
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
      const double takeoff_elapsed_s =
        (now_time - takeoff_land.toggle_takeoff_land_time).seconds();
      if (!odom_is_received(now_time)) {
        change_state(MANUAL_CTRL);
        toggle_offboard_mode(false);
        RCLCPP_WARN(node_->get_logger(), "[px4ctrl] AUTO_TAKEOFF --> MANUAL_CTRL, odom timeout.");
      } else if (
        param.takeoff_land.enable_auto_arm && !state_data.current_state.armed &&
        takeoff_elapsed_s > param.takeoff_land.auto_arm_timeout_s) {
        change_state(MANUAL_CTRL);
        toggle_offboard_mode(false);
        RCLCPP_ERROR(
          node_->get_logger(),
          "[px4ctrl] AUTO_TAKEOFF aborted: FCU is not armed %.2fs after auto-arm request. "
          "Check PX4 arming denial, RC throttle low, and safety switch state.",
          takeoff_elapsed_s);
      } else if (
        takeoff_elapsed_s < AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) {
        rotor_speedup_during_takeoff = true;
        des = get_rotor_speed_up_des(now_time);
      } else if (control_odom_data.p(2) >= takeoff_land.start_pose(2) + param.takeoff_land.height) {
        change_state(AUTO_HOVER);
        set_hov_with_odom(control_odom_data);
        hover_pose(2) = clamp(
          takeoff_land.start_pose(2) + param.takeoff_land.height,
          param.limits.z_min,
          param.limits.z_max);
        des = get_hover_des();
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
        "[px4ctrl] AUTO_TAKEOFF phase=%s control_odom=(%.3f,%.3f,%.3f) start=(%.3f,%.3f,%.3f) "
        "des=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f,%.3f)",
        rotor_speedup_during_takeoff ? "speedup" : "climb",
        control_odom_data.p.x(),
        control_odom_data.p.y(),
        control_odom_data.p.z(),
        takeoff_land.start_pose.x(),
        takeoff_land.start_pose.y(),
        takeoff_land.start_pose.z(),
        des.p.x(),
        des.p.y(),
        des.p.z(),
        control_odom_data.v.x(),
        control_odom_data.v.y(),
        control_odom_data.v.z());
      break;
    }

    case AUTO_LAND: {
      if (!rc_data.is_hover_mode || !odom_is_received(now_time)) {
        change_state(MANUAL_CTRL);
        toggle_offboard_mode(false);
        RCLCPP_WARN(node_->get_logger(), "[px4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)");
      } else if (!rc_data.is_command_mode) {
        change_state(AUTO_HOVER);
        set_hov_with_odom(control_odom_data);
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
      !param.physical_control.enable &&
      (state == AUTO_HOVER || state == CMD_CTRL)) {
      controller.estimateThrustModel(imu_data.a, now_time);
    }

    const Desired_State_t safe_des = clamp_desired(des);
    Controller_Output_t u;
    Controller_Debug_t debug;
    bool have_debug = false;
    if (state == MANUAL_CTRL || rotor_low_speed_during_land) {
      u.q = control_odom_data.q;
      u.bodyrates.setZero();
      u.thrust = clamp(
        param.thrust_model.hover_thrust,
        param.controller.min_thrust,
        param.controller.max_thrust);
      u.total_thrust_n = 0.0;
      u.physical_setpoint_valid = false;
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
      u = controller.calculateControl(safe_des, control_odom_data, imu_data, now_time, &debug);
      have_debug = true;
      u.thrust = clamp(
        std::min(u.thrust, ramp_thrust),
        param.controller.min_thrust,
        param.controller.max_thrust);
      if (std::isfinite(u.total_thrust_n) && param.thrust_model.hover_thrust > 1e-6) {
        const double physical_ramp_limit_n =
          param.physical_control.mass_kg * param.controller.gravity *
          ramp_thrust / param.thrust_model.hover_thrust;
        u.total_thrust_n = std::min(u.total_thrust_n, physical_ramp_limit_n);
      }
      debug.thrust = u.thrust;
      controller.resetControlState();
    } else {
      u = controller.calculateControl(safe_des, control_odom_data, imu_data, now_time, &debug);
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
        safe_des.p.x() - control_odom_data.p.x(),
        safe_des.p.y() - control_odom_data.p.y(),
        safe_des.p.z() - control_odom_data.p.z(),
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
    publish_simulink_actual(control_odom_data, now_time);
    publish_simulink_tracking_error(safe_des, control_odom_data, now_time);
    publish_simulink_yaw_debug(
      safe_des,
      control_odom_data,
      imu_data,
      u,
      have_debug ? &debug : nullptr,
      now_time);
    if (have_debug) {
      publish_simulink_ude_debug(debug, now_time);
    }
  }

  land_detector(state, des, control_odom_data);
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
  return apply_td_reference(des, AUTO_HOVER, node_->now());
}

Desired_State_t PX4CtrlFSM::get_cmd_des()
{
  Desired_State_t des;
  des.p = cmd_data.p;
  if (param.cmd_feedforward.enable) {
    des.v = limit_norm(cmd_data.v, param.cmd_feedforward.max_velocity);
    des.a = limit_norm(cmd_data.a, param.cmd_feedforward.max_acceleration);
    des.j = limit_norm(cmd_data.j, param.cmd_feedforward.max_jerk);
    des.snap = limit_norm(cmd_data.snap, param.cmd_feedforward.max_snap);
  } else {
    des.v.setZero();
    des.a.setZero();
    des.j.setZero();
    des.snap.setZero();
  }
  des.yaw = cmd_data.yaw;
  des.yaw_rate = param.cmd_feedforward.enable ? cmd_data.yaw_rate : 0.0;
  des.yaw_acceleration = param.cmd_feedforward.enable ?
    std::clamp(
      cmd_data.yaw_acceleration,
      -std::abs(param.cmd_feedforward.max_yaw_acceleration),
      std::abs(param.cmd_feedforward.max_yaw_acceleration)) : 0.0;
  return apply_td_reference(des, CMD_CTRL, node_->now());
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
  if (speed > 0.0) {
    const double target_z = takeoff_land.start_pose(2) + param.takeoff_land.height;
    if (des.p.z() >= target_z) {
      des.p.z() = target_z;
      des.v.z() = 0.0;
    }
  }
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
  const Odom_Data_t &odom = control_odom_data.received ? control_odom_data : odom_data;
  now_pose.header.stamp = node_->now();
  now_pose.header.frame_id = param.frame_id;
  now_pose.pose.position.x = odom.p.x();
  now_pose.pose.position.y = odom.p.y();
  now_pose.pose.position.z = odom.p.z();
  now_pose.pose.orientation = odom.msg.pose.pose.orientation;
  traj_start_trigger_pub->publish(now_pose);
}

void PX4CtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom)
{
  takeoff_land.start_pose.head<3>() = odom.p;
  takeoff_land.start_pose(3) =
    uav_utils::normalize_angle(uav_utils::get_yaw_from_quaternion(odom.q));
  takeoff_land.toggle_takeoff_land_time = node_->now();
}

void PX4CtrlFSM::set_hov_with_odom(const Odom_Data_t &odom)
{
  hover_pose.head<3>() = odom.p;
  hover_pose(3) =
    uav_utils::normalize_angle(uav_utils::get_yaw_from_quaternion(odom.q));
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

Odom_Data_t PX4CtrlFSM::build_control_odom(const rclcpp::Time &now_time)
{
  Odom_Data_t result = odom_data;
  MocapControlStatus status;
  status.odom_stamp_from_receive_time = odom_data.stamp_from_receive_time;

  auto set_result_message = [&result]() {
    result.msg.pose.pose.position.x = result.p.x();
    result.msg.pose.pose.position.y = result.p.y();
    result.msg.pose.pose.position.z = result.p.z();
    result.msg.pose.pose.orientation.x = result.q.x();
    result.msg.pose.pose.orientation.y = result.q.y();
    result.msg.pose.pose.orientation.z = result.q.z();
    result.msg.pose.pose.orientation.w = result.q.w();
    result.msg.twist.twist.linear.x = result.v.x();
    result.msg.twist.twist.linear.y = result.v.y();
    result.msg.twist.twist.linear.z = result.v.z();
  };

  if (!param.mocap_state.enable) {
    status.reason = "mocap_state disabled";
    mocap_control_status = status;
    set_result_message();
    return result;
  }

  const std::string twist_frame = lower_copy(param.mocap_state.twist_frame);
  const bool twist_frame_is_map =
    twist_frame == "map" || twist_frame == "enu" || twist_frame == "world";
  const bool pose_fresh =
    mocap_pose_data.is_received(now_time, param.msg_timeout.mocap_pose);
  const bool twist_fresh =
    mocap_twist_data.is_received(now_time, param.msg_timeout.mocap_twist);
  const bool pose_finite = vector_finite(mocap_pose_data.p);
  const bool twist_finite = vector_finite(mocap_twist_data.v);

  status.pose_stamp_from_receive_time = mocap_pose_data.stamp_from_receive_time;
  status.twist_stamp_from_receive_time = mocap_twist_data.stamp_from_receive_time;

  bool use_mocap = false;
  if (!pose_fresh) {
    status.reason = "mocap pose stale/missing";
  } else if (!twist_fresh) {
    status.reason = "mocap twist stale/missing";
  } else if (!pose_finite) {
    status.reason = "mocap pose non-finite";
  } else if (!twist_finite) {
    status.reason = "mocap twist non-finite";
  } else if (!twist_frame_is_map) {
    status.reason = "mocap twist_frame is not map/enu/world";
  } else {
    status.pose_twist_dt_s =
      abs_time_diff_s(mocap_pose_data.msg_stamp, mocap_twist_data.msg_stamp);
    status.mocap_odom_dt_s = std::max(
      abs_time_diff_s(mocap_pose_data.msg_stamp, odom_data.msg_stamp),
      abs_time_diff_s(mocap_twist_data.msg_stamp, odom_data.msg_stamp));

    if (status.pose_twist_dt_s > param.mocap_state.max_pair_dt_s) {
      status.reason = "mocap pose/twist stamp mismatch";
    } else if (status.mocap_odom_dt_s > param.mocap_state.max_odom_attitude_dt_s) {
      status.reason = "mocap and odom attitude stamp mismatch";
    } else {
      use_mocap = true;
    }
  }

  if (use_mocap) {
    result.p = mocap_pose_data.p;
    result.v = mocap_twist_data.v;
    status.p_source = "mocap";
    status.v_source = "mocap";
    status.reason = "mocap_synced";
  } else {
    status.p_source = "odom";
    status.v_source = "odom";
    if (status.reason.empty()) {
      status.reason = "odom fallback";
    }
    if (!param.mocap_state.fallback_to_odom) {
      status.reason += "; fallback_to_odom=false, holding odom for safety";
    }
  }

  const Eigen::Vector3d base_position = result.p;
  const Eigen::Vector3d base_velocity = result.v;
  const std::string source_key = status.p_source + "/" + status.v_source;
  status.pose_to_odom_dt_s =
    (odom_data.msg_stamp - mocap_pose_data.msg_stamp).seconds();
  status.twist_to_odom_dt_s =
    (odom_data.msg_stamp - mocap_twist_data.msg_stamp).seconds();

  const double max_prediction_dt_s =
    std::max(0.0, param.mocap_state.max_prediction_dt_s);
  const double max_future_dt_s =
    std::max(0.0, param.mocap_state.max_future_dt_s);
  status.prediction_valid =
    use_mocap &&
    std::isfinite(status.pose_to_odom_dt_s) &&
    std::isfinite(status.twist_to_odom_dt_s) &&
    status.pose_to_odom_dt_s >= -max_future_dt_s &&
    status.twist_to_odom_dt_s >= -max_future_dt_s &&
    status.pose_to_odom_dt_s <= max_prediction_dt_s &&
    status.twist_to_odom_dt_s <= max_prediction_dt_s;
  status.prediction_dt_s = status.prediction_valid ?
    std::clamp(status.pose_to_odom_dt_s, 0.0, max_prediction_dt_s) : 0.0;
  Eigen::Vector3d prediction_target = Eigen::Vector3d::Zero();
  if (status.prediction_valid) {
    prediction_target = mocap_twist_data.v * status.prediction_dt_s;
  }

  double alignment_dt_s = 0.0;
  if (state_alignment.initialized) {
    const double raw_dt_s = (now_time - state_alignment.last_update_time).seconds();
    if (std::isfinite(raw_dt_s) && raw_dt_s > 0.0) {
      alignment_dt_s = std::min(raw_dt_s, 0.05);
    }
  }
  state_alignment.last_update_time = now_time;

  const double blend_tau_s =
    std::max(0.01, param.mocap_state.prediction_blend_tau_s);
  const double blend_alpha = alignment_dt_s > 0.0 ?
    std::clamp(1.0 - std::exp(-alignment_dt_s / blend_tau_s), 0.0, 1.0) : 0.0;
  const double decay = 1.0 - blend_alpha;
  const bool source_changed =
    state_alignment.initialized && source_key != state_alignment.source_key;

  if (!state_alignment.initialized) {
    state_alignment.prediction_correction = prediction_target;
    state_alignment.prediction_weight = status.prediction_valid ? 1.0 : 0.0;
    state_alignment.source_position_offset.setZero();
    state_alignment.source_velocity_offset.setZero();
    state_alignment.initialized = true;
  } else {
    state_alignment.prediction_correction +=
      blend_alpha * (prediction_target - state_alignment.prediction_correction);
    state_alignment.prediction_weight +=
      blend_alpha *
      ((status.prediction_valid ? 1.0 : 0.0) - state_alignment.prediction_weight);

    if (source_changed) {
      state_alignment.source_position_offset =
        state_alignment.last_position -
        (base_position + state_alignment.prediction_correction);
      state_alignment.source_velocity_offset =
        state_alignment.last_velocity - base_velocity;
    } else {
      state_alignment.source_position_offset *= decay;
      state_alignment.source_velocity_offset *= decay;
    }
  }

  state_alignment.source_key = source_key;
  result.p =
    base_position +
    state_alignment.prediction_correction +
    state_alignment.source_position_offset;
  result.v = base_velocity + state_alignment.source_velocity_offset;
  state_alignment.last_position = result.p;
  state_alignment.last_velocity = result.v;

  status.prediction_weight = state_alignment.prediction_weight;
  status.position_correction =
    state_alignment.prediction_correction +
    state_alignment.source_position_offset;
  status.velocity_correction = state_alignment.source_velocity_offset;
  status.source_transition_active =
    source_changed ||
    state_alignment.source_position_offset.norm() > 1e-4 ||
    state_alignment.source_velocity_offset.norm() > 1e-4;

  if (
    status.reason != "mocap_synced" &&
    status.reason != "mocap_state disabled") {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[px4ctrl] mocap p/v fallback to odom: reason='%s' pose_twist_dt=%.4f "
      "mocap_odom_dt=%.4f pose_fresh=%d twist_fresh=%d",
      status.reason.c_str(),
      status.pose_twist_dt_s,
      status.mocap_odom_dt_s,
      pose_fresh ? 1 : 0,
      twist_fresh ? 1 : 0);
  }

  if (
    status.pose_stamp_from_receive_time ||
    status.twist_stamp_from_receive_time ||
    status.odom_stamp_from_receive_time) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[px4ctrl] state sync used receive time because header stamp was zero: "
      "pose=%d twist=%d odom=%d",
      status.pose_stamp_from_receive_time ? 1 : 0,
      status.twist_stamp_from_receive_time ? 1 : 0,
      status.odom_stamp_from_receive_time ? 1 : 0);
  }

  mocap_control_status = status;
  set_result_message();
  return result;
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
  publish_physical_setpoint(u, stamp);

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

void PX4CtrlFSM::publish_physical_setpoint(
  const Controller_Output_t &u,
  const rclcpp::Time &stamp)
{
  if (!param.physical_control.enable || !physical_setpoint_pub) {
    return;
  }

  physical_setpoint_protocol::Sample sample;
  const bool angular_acceleration_enabled =
    !param.use_bodyrate_ctrl &&
    param.physical_control.angular_acceleration_feedforward_scale > 1e-6;
  // Keep the validated v1 sample when angular-acceleration feedforward is off.
  // Use v2 only when the analytic higher-order feedforward is explicitly enabled.
  sample.version = angular_acceleration_enabled ?
    physical_setpoint_protocol::kVersionV2 :
    physical_setpoint_protocol::kVersionV1;
  sample.mode = param.use_bodyrate_ctrl ?
    physical_setpoint_protocol::kModeBodyrate :
    physical_setpoint_protocol::kModeAttitude;
  sample.sequence = physical_setpoint_sequence++;
  sample.source_time_us = stamp.nanoseconds() > 0 ?
    static_cast<std::uint64_t>(stamp.nanoseconds() / 1000) : 0U;

  Eigen::Quaterniond q = u.q;
  if (quaternion_finite(q)) {
    q.normalize();
    Eigen::Quaterniond q_ned_frd = mavros::ftf::transform_orientation_enu_ned(
      mavros::ftf::transform_orientation_baselink_aircraft(q));
    if (quaternion_finite(q_ned_frd)) {
      q_ned_frd.normalize();
      sample.q_d_wxyz = {
        static_cast<float>(q_ned_frd.w()),
        static_cast<float>(q_ned_frd.x()),
        static_cast<float>(q_ned_frd.y()),
        static_cast<float>(q_ned_frd.z())};
      sample.valid_flags |= physical_setpoint_protocol::kQuaternionValid;
    }
  }

  // Attitude mode carries desired-frame feedforward rates. Bodyrate mode carries
  // the complete current-body rate command used by SET_ATTITUDE_TARGET.
  const Eigen::Vector3d physical_body_rate =
    param.use_bodyrate_ctrl ?
    u.bodyrates :
    param.physical_control.body_rate_feedforward_scale * u.bodyrates_ff;
  if (vector_finite(physical_body_rate)) {
    const Eigen::Vector3d body_rate_frd =
      mavros::ftf::transform_frame_baselink_aircraft(physical_body_rate);
    if (vector_finite(body_rate_frd)) {
      sample.body_rate_d = {
        static_cast<float>(body_rate_frd.x()),
        static_cast<float>(body_rate_frd.y()),
        static_cast<float>(body_rate_frd.z())};
      sample.valid_flags |= physical_setpoint_protocol::kBodyRateValid;
    }
  }

  Eigen::Vector3d physical_body_rate_dot = Eigen::Vector3d::Zero();
  if (angular_acceleration_enabled) {
    physical_body_rate_dot =
      param.physical_control.angular_acceleration_feedforward_scale * u.bodyrates_dot_ff;
  }
  if (angular_acceleration_enabled && vector_finite(physical_body_rate_dot)) {
    const Eigen::Vector3d body_rate_dot_frd =
      mavros::ftf::transform_frame_baselink_aircraft(physical_body_rate_dot);
    if (vector_finite(body_rate_dot_frd)) {
      sample.body_rate_dot_d = {
        static_cast<float>(body_rate_dot_frd.x()),
        static_cast<float>(body_rate_dot_frd.y()),
        static_cast<float>(body_rate_dot_frd.z())};
      sample.valid_flags |= physical_setpoint_protocol::kAngularAccelerationValid;
    }
  }

  if (u.physical_setpoint_valid && std::isfinite(u.total_thrust_n) &&
      u.total_thrust_n >= 0.0 &&
      u.total_thrust_n <= param.physical_control.max_total_thrust_n) {
    sample.total_thrust_n = static_cast<float>(u.total_thrust_n);
    sample.valid_flags |= physical_setpoint_protocol::kPhysicalThrustValid;
  }

  mavros_msgs::msg::Tunnel msg;
  msg.target_system = static_cast<std::uint8_t>(param.physical_control.target_system);
  msg.target_component = static_cast<std::uint8_t>(param.physical_control.target_component);
  msg.payload_type = static_cast<std::uint16_t>(param.physical_control.payload_type);
  msg.payload_length = static_cast<std::uint8_t>(
    physical_setpoint_protocol::payload_size(sample.version));
  if (!physical_setpoint_protocol::encode(sample, msg.payload)) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[px4ctrl] Failed to encode physical TUNNEL setpoint.");
    return;
  }
  physical_setpoint_pub->publish(msg);
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
  if (
    mocap_control_status.p_source == "mocap" &&
    mocap_control_status.v_source == "mocap") {
    msg.child_frame_id = "actual_state_mocap_synced_pv_odom_attitude";
  } else if (!param.mocap_state.enable) {
    msg.child_frame_id = "actual_state_odom_mocap_disabled";
  } else {
    msg.child_frame_id = "actual_state_odom_fallback";
  }
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
  msg.child_frame_id = "tracking_error_des_minus_actual";
  set_point(msg.pose.pose.position, des.p - odom.p);
  set_vector3(msg.twist.twist.linear, des.v - odom.v);

  const double odom_yaw = uav_utils::normalize_angle(uav_utils::get_yaw_from_quaternion(odom.q));
  const double yaw_error = uav_utils::normalize_angle(des.yaw - odom_yaw);
  const Eigen::Quaterniond yaw_error_q = uav_utils::yaw_to_quaternion(yaw_error);
  set_quaternion(msg.pose.pose.orientation, yaw_error_q);
  msg.twist.twist.angular.z = des.yaw_rate - odom.w.z();

  simulink_tracking_error_pub->publish(msg);
}

void PX4CtrlFSM::publish_simulink_yaw_debug(
  const Desired_State_t &des,
  const Odom_Data_t &odom,
  const Imu_Data_t &imu,
  const Controller_Output_t &u,
  const Controller_Debug_t *debug,
  const rclcpp::Time &stamp)
{
  if (!simulink_yaw_debug_pub) {
    return;
  }

  const double yaw_des = uav_utils::normalize_angle(des.yaw);
  const double yaw_odom = uav_utils::normalize_angle(uav_utils::get_yaw_from_quaternion(odom.q));
  const double yaw_imu = uav_utils::normalize_angle(uav_utils::get_yaw_from_quaternion(imu.q));
  double yaw_mocap = std::numeric_limits<double>::quiet_NaN();
  if (mocap_pose_data.is_received(stamp, param.msg_timeout.mocap_pose)) {
    yaw_mocap = yaw_from_geometry_quaternion(mocap_pose_data.msg.pose.orientation);
  }

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = param.frame_id;
  msg.child_frame_id =
    "yaw_debug: pos.x=des pos.y=odom pos.z=imu lin.x=mocap "
    "lin.y=err_des_odom lin.z=err_des_imu ang.x=odom_rate ang.y=imu_rate ang.z=cmd_rate "
    "ori.x=ff_z ori.y=fb_z ori.z=raw_yaw_cmd ori.w=deadbanded_yaw_error";

  msg.pose.pose.position.x = yaw_des;
  msg.pose.pose.position.y = yaw_odom;
  msg.pose.pose.position.z = yaw_imu;
  if (debug) {
    msg.pose.pose.orientation.x = debug->bodyrates_ff.z();
    msg.pose.pose.orientation.y = debug->bodyrates_fb.z();
    msg.pose.pose.orientation.z = debug->yaw_rate_cmd_raw;
    msg.pose.pose.orientation.w = debug->yaw_error_deadbanded;
  } else {
    msg.pose.pose.orientation.w = 1.0;
  }
  msg.twist.twist.linear.x = yaw_mocap;
  msg.twist.twist.linear.y = uav_utils::normalize_angle(yaw_des - yaw_odom);
  msg.twist.twist.linear.z = uav_utils::normalize_angle(yaw_des - yaw_imu);
  msg.twist.twist.angular.x = odom.w.z();
  msg.twist.twist.angular.y = imu.w.z();
  msg.twist.twist.angular.z = u.bodyrates.z();
  simulink_yaw_debug_pub->publish(msg);
}

void PX4CtrlFSM::publish_simulink_ude_debug(
  const Controller_Debug_t &debug,
  const rclcpp::Time &stamp)
{
  if (!simulink_ude_debug_pub) {
    return;
  }

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = param.frame_id;
  msg.child_frame_id =
    "ude_debug: position=e orientation.xyz=u0 orientation.w=thrust "
    "linear=f_hat angular=u_acc pose_cov[0:3]=integral_u0 "
    "pose_cov[3:6]=thrust_acc twist_cov[0:3]=bodyrate_ff "
    "twist_cov[3:6]=bodyrate_dot_ff twist_cov[6:9]=bodyrate_fb "
    "twist_cov[9:12]=bodyrate_cmd";
  set_point(msg.pose.pose.position, debug.e);
  msg.pose.pose.orientation.x = debug.u0.x();
  msg.pose.pose.orientation.y = debug.u0.y();
  msg.pose.pose.orientation.z = debug.u0.z();
  msg.pose.pose.orientation.w = debug.thrust;
  set_vector3(msg.twist.twist.linear, debug.f_hat);
  set_vector3(msg.twist.twist.angular, debug.u_acc);
  for (int axis = 0; axis < 3; ++axis) {
    msg.pose.covariance[axis] = debug.integral_u0(axis);
    msg.pose.covariance[3 + axis] = debug.thrust_acc_limited(axis);
    msg.twist.covariance[axis] = debug.bodyrates_ff(axis);
    msg.twist.covariance[3 + axis] = debug.bodyrates_dot_ff(axis);
    msg.twist.covariance[6 + axis] = debug.bodyrates_fb(axis);
    msg.twist.covariance[9 + axis] = debug.bodyrates_cmd(axis);
  }
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

void PX4CtrlFSM::publish_mocap_state_status(const rclcpp::Time &stamp, bool force)
{
  if (!mocap_state_status_pub) {
    return;
  }

  if (!force && (stamp - last_mocap_status_publish_time).seconds() < 0.1) {
    return;
  }
  last_mocap_status_publish_time = stamp;

  std_msgs::msg::String msg;
  std::ostringstream oss;
  oss << "p_source=" << mocap_control_status.p_source
      << " v_source=" << mocap_control_status.v_source
      << " pose_twist_dt=" << mocap_control_status.pose_twist_dt_s
      << " mocap_odom_dt=" << mocap_control_status.mocap_odom_dt_s
      << " pose_to_odom_dt=" << mocap_control_status.pose_to_odom_dt_s
      << " twist_to_odom_dt=" << mocap_control_status.twist_to_odom_dt_s
      << " prediction_dt=" << mocap_control_status.prediction_dt_s
      << " prediction_weight=" << mocap_control_status.prediction_weight
      << " prediction_valid=" << (mocap_control_status.prediction_valid ? "true" : "false")
      << " source_transition="
      << (mocap_control_status.source_transition_active ? "true" : "false")
      << " position_correction=["
      << mocap_control_status.position_correction.x() << ","
      << mocap_control_status.position_correction.y() << ","
      << mocap_control_status.position_correction.z() << "]"
      << " velocity_correction=["
      << mocap_control_status.velocity_correction.x() << ","
      << mocap_control_status.velocity_correction.y() << ","
      << mocap_control_status.velocity_correction.z() << "]"
      << " pose_stamp_receive=" << (mocap_control_status.pose_stamp_from_receive_time ? "true" : "false")
      << " twist_stamp_receive=" << (mocap_control_status.twist_stamp_from_receive_time ? "true" : "false")
      << " odom_stamp_receive=" << (mocap_control_status.odom_stamp_from_receive_time ? "true" : "false")
      << " reason='" << mocap_control_status.reason << "'";
  msg.data = oss.str();
  mocap_state_status_pub->publish(msg);
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
  const Odom_Data_t &height_odom = control_odom_data.received ? control_odom_data : odom_data;
  const bool below_safe_height =
    height_odom.received && height_odom.p.z() <= param.gripper.force_open_below_z;

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
  const bool td_was_enabled = param.td.enable;
  bool thrust_mapping_changed = false;
  const auto result = param.apply_runtime_parameters(params, &thrust_mapping_changed);
  if (result.successful) {
    controller.resetControlState();
    if (thrust_mapping_changed) {
      controller.resetThrustMapping();
    }
    if (!td_was_enabled && param.td.enable) {
      reset_td_tracker("td.enable enabled at runtime");
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
    const State_t old_state = state;
    const auto is_airborne_control_state = [](State_t value) {
        return value == AUTO_TAKEOFF || value == AUTO_HOVER || value == CMD_CTRL ||
               value == AUTO_LAND;
      };
    const bool preserve_control_state =
      is_airborne_control_state(old_state) && is_airborne_control_state(new_state);
    if (!preserve_control_state) {
      controller.resetControlState();
      had_valid_control_feedback = false;
    }
    if (td_applicable_state(old_state) || td_applicable_state(new_state)) {
      std::ostringstream oss;
      oss << "state " << state_to_string(old_state) << " -> " << state_to_string(new_state);
      reset_td_tracker(oss.str());
    }
  }
  state = new_state;
}

Desired_State_t PX4CtrlFSM::apply_td_reference(
  const Desired_State_t &raw_des,
  State_t source_state,
  const rclcpp::Time &now_time)
{
  if (!param.td.enable) {
    reset_td_tracker("td disabled");
    return raw_des;
  }
  if (!td_applicable_state(source_state)) {
    reset_td_tracker("state is not TD applicable");
    return raw_des;
  }
  if (!td_parameters_valid()) {
    reset_td_tracker("invalid TD parameters");
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[px4ctrl] TD fallback to raw reference because h/r is invalid.");
    return raw_des;
  }

  const auto vector_finite = [](const Eigen::Vector3d &value) {
    return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
  };
  if (!vector_finite(raw_des.p)) {
    reset_td_tracker("raw reference is not finite");
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[px4ctrl] TD fallback to raw reference because raw_des.p is not finite.");
    return raw_des;
  }
  if (!vector_finite(td_tracker.v1) || !vector_finite(td_tracker.v2)) {
    reset_td_tracker("TD state is not finite");
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[px4ctrl] TD state was not finite; resetting and using raw reference this cycle.");
    return raw_des;
  }
  if (td_tracker.initialized && td_tracker.last_state != source_state) {
    std::ostringstream oss;
    oss << "source state " << state_to_string(td_tracker.last_state)
        << " -> " << state_to_string(source_state);
    reset_td_tracker(oss.str());
  }

  const double h = td_h();
  Desired_State_t des = raw_des;

  if (!td_tracker.initialized) {
    td_tracker.v1 = control_odom_data.p;
    td_tracker.v2.setZero();
    td_tracker.last_raw_ref = raw_des.p;
    td_tracker.last_process_time = now_time;
    td_tracker.last_state = source_state;
    td_tracker.initialized = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "[px4ctrl] TD init reason='%s' state=%s h=%.6f r=(%.3f,%.3f,%.3f) "
      "odom_init=(%.3f,%.3f,%.3f) raw_ref=(%.3f,%.3f,%.3f)",
      td_tracker.reset_reason.c_str(),
      state_to_string(source_state),
      h,
      param.td.r_diag[0],
      param.td.r_diag[1],
      param.td.r_diag[2],
      td_tracker.v1.x(),
      td_tracker.v1.y(),
      td_tracker.v1.z(),
      raw_des.p.x(),
      raw_des.p.y(),
      raw_des.p.z());
    td_tracker.reset_reason = "continuous";
    des.p = td_tracker.v1;
    des.v = td_tracker.v2;
    des.a.setZero();
    des.j.setZero();
    return des;
  }

  const double elapsed = (now_time - td_tracker.last_process_time).seconds();
  if (!std::isfinite(elapsed) || elapsed < 0.0) {
    reset_td_tracker("TD time jump");
    RCLCPP_WARN(
      node_->get_logger(),
      "[px4ctrl] TD saw invalid elapsed=%.6f; resetting and using raw reference this cycle.",
      elapsed);
    return raw_des;
  }
  if (elapsed > 2.5 * h) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[px4ctrl] TD process interval %.6fs exceeded 2.5*h=%.6fs; holding TD output this cycle.",
      elapsed,
      2.5 * h);
    td_tracker.last_raw_ref = raw_des.p;
    td_tracker.last_process_time = now_time;
    des.p = td_tracker.v1;
    des.v = td_tracker.v2;
    des.a.setZero();
    des.j.setZero();
    return des;
  }

  Eigen::Vector3d next_v1 = td_tracker.v1;
  Eigen::Vector3d next_v2 = td_tracker.v2;
  for (int i = 0; i < 3; ++i) {
    const double fn =
      td_fst(td_tracker.v1(i) - raw_des.p(i), td_tracker.v2(i), param.td.r_diag[i], h);
    next_v1(i) = td_tracker.v1(i) + h * td_tracker.v2(i);
    next_v2(i) = td_tracker.v2(i) + h * fn;
  }

  if (!vector_finite(next_v1) || !vector_finite(next_v2)) {
    reset_td_tracker("TD update produced non-finite state");
    RCLCPP_ERROR(
      node_->get_logger(),
      "[px4ctrl] TD update produced non-finite state; resetting and using raw reference this cycle.");
    return raw_des;
  }

  td_tracker.v1 = next_v1;
  td_tracker.v2 = next_v2;
  td_tracker.last_raw_ref = raw_des.p;
  td_tracker.last_process_time = now_time;
  td_tracker.last_state = source_state;

  des.p = td_tracker.v1;
  des.v = td_tracker.v2;
  des.a.setZero();
  des.j.setZero();
  return des;
}

void PX4CtrlFSM::reset_td_tracker(const std::string &reason)
{
  td_tracker.initialized = false;
  td_tracker.v1.setZero();
  td_tracker.v2.setZero();
  td_tracker.last_raw_ref.setZero();
  td_tracker.last_process_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  td_tracker.reset_reason = reason;
}

bool PX4CtrlFSM::td_applicable_state(State_t check_state) const
{
  return check_state == AUTO_HOVER || check_state == CMD_CTRL;
}

bool PX4CtrlFSM::td_parameters_valid() const
{
  if (td_h() <= 0.0 || !std::isfinite(td_h())) {
    return false;
  }
  for (double r : param.td.r_diag) {
    if (!std::isfinite(r) || r <= 0.0) {
      return false;
    }
  }
  return true;
}

double PX4CtrlFSM::td_h() const
{
  if (param.ctrl_freq_max <= 0.0 || !std::isfinite(param.ctrl_freq_max)) {
    return 0.0;
  }
  return 1.0 / param.ctrl_freq_max;
}

double PX4CtrlFSM::td_fst(double x1, double x2, double r, double h) const
{
  const auto sgn = [](double x) {
    if (x > 0.0) {
      return 1.0;
    }
    if (x < 0.0) {
      return -1.0;
    }
    return 0.0;
  };

  const double d = h * r;
  if (d <= 0.0 || !std::isfinite(d)) {
    return 0.0;
  }
  const double d0 = h * d;
  const double y = x1 + h * x2;
  const double a0 = std::sqrt(d * d + 8.0 * r * std::abs(y));
  const double a = std::abs(y) <= d0 ?
    x2 + y / h :
    x2 + 0.5 * (a0 - d) * sgn(y);
  const double sat = std::abs(a) <= d ? a / d : sgn(a);
  return -r * sat;
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
