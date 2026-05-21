#ifndef PX4CTRL_FSM_H
#define PX4CTRL_FSM_H

#include <utility>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "controller.h"
#include "input.h"

struct AutoTakeoffLand_t
{
  bool landed{true};
  rclcpp::Time toggle_takeoff_land_time{0, 0, RCL_ROS_TIME};
  bool delay_trigger{false};
  rclcpp::Time delay_trigger_time{0, 0, RCL_ROS_TIME};
  Eigen::Vector4d start_pose{Eigen::Vector4d::Zero()};

  static constexpr double MOTORS_SPEEDUP_TIME = 3.0;
  static constexpr double DELAY_TRIGGER_TIME = 2.0;
};

class PX4CtrlFSM
{
public:
  enum State_t
  {
    MANUAL_CTRL = 1,
    AUTO_HOVER,
    CMD_CTRL,
    AUTO_TAKEOFF,
    AUTO_LAND
  };

  Parameter_t &param;

  RC_Data_t rc_data;
  State_Data_t state_data;
  ExtendedState_Data_t extended_state_data;
  Odom_Data_t odom_data;
  Command_Data_t cmd_data;
  Battery_Data_t bat_data;
  Takeoff_Land_Data_t takeoff_land_data;

  LinearControl &controller;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr traj_start_trigger_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr expert_pose_pub;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr ctrl_FCU_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_cmd_pub;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_FCU_mode_srv;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_srv;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr reboot_FCU_srv;

  Eigen::Vector4d hover_pose{Eigen::Vector4d::Zero()};
  rclcpp::Time last_set_hover_pose_time{0, 0, RCL_ROS_TIME};

  PX4CtrlFSM(Parameter_t &param, LinearControl &controller, rclcpp::Node *node);

  void process();
  bool rc_is_received(const rclcpp::Time &now_time) const;
  bool cmd_is_received(const rclcpp::Time &now_time) const;
  bool odom_is_received(const rclcpp::Time &now_time) const;
  bool bat_is_received(const rclcpp::Time &now_time) const;
  bool recv_new_odom();
  State_t get_state() const { return state; }
  bool get_landed() const { return takeoff_land.landed; }
  void manual_flag_cb(const std_msgs::msg::UInt8::SharedPtr msg);

private:
  rclcpp::Node *node_;
  State_t state{MANUAL_CTRL};
  AutoTakeoffLand_t takeoff_land;
  bool have_gripper_target{false};
  double last_gripper_target{0.0};

  Desired_State_t get_hover_des();
  Desired_State_t get_cmd_des();
  Desired_State_t get_rotor_speed_up_des(const rclcpp::Time &now);
  Desired_State_t get_takeoff_land_des(double speed);

  void motors_idling(Desired_State_t &des);
  void land_detector(State_t state, const Desired_State_t &des, const Odom_Data_t &odom);
  void set_start_pose_for_takeoff_land(const Odom_Data_t &odom);
  void set_hov_with_odom();
  void set_hov_with_rc();
  void publish_position_ctrl(const Controller_Output_t &u, const rclcpp::Time &stamp);
  void publish_expert_pose(const Desired_State_t &des, const rclcpp::Time &stamp);
  void publish_trigger(const geometry_msgs::msg::PoseStamped &odom_msg);
  void publish_gripper_safety();
  void publish_gripper_from_rc();
  void publish_gripper_target(double target, bool force = false);
  bool px4_mode_allows_gripper_rc() const;
  bool should_force_gripper_open() const;

  bool toggle_offboard_mode(bool on_off);
  bool toggle_arm_disarm(bool arm);
  void reboot_FCU();
  double clamp(double value, double low, double high) const;
  Desired_State_t clamp_desired(const Desired_State_t &des) const;
};

#endif
