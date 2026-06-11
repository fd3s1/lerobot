#ifndef PX4CTRL_PARAM_H
#define PX4CTRL_PARAM_H

#include <array>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

class Parameter_t
{
public:
  struct MsgTimeout
  {
    double odom{0.5};
    double mocap_pose{0.3};
    double mocap_twist{0.3};
    double imu{0.5};
    double rc{0.5};
    double cmd{0.5};
    double bat{0.5};
  };

  struct RCReverse
  {
    bool roll{false};
    bool pitch{true};
    bool yaw{false};
    bool throttle{true};
  };

  struct AutoTakeoffLand
  {
    bool enable{true};
    bool enable_auto_arm{true};
    bool no_RC{false};
    double height{1.0};
    double speed{0.2};
  };

  struct Topics
  {
    std::string rc{"/mavros/rc/in"};
    std::string odom{"/mavros/local_position/odom"};
    std::string mocap_pose{"/mavros/vision_pose/pose"};
    std::string mocap_twist{"/vla_drone1/twist"};
    std::string imu{"/mavros/imu/data"};
    std::string cmd{"/position_cmd"};
    std::string takeoff_land{"/px4ctrl/takeoff_land"};
    std::string setpoint{"/mavros/setpoint_raw/attitude"};
    std::string simulink_setpoint{"/px4ctrl/simulink/attitude_target"};
    std::string simulink_reference{"/px4ctrl/simulink/reference_state"};
    std::string simulink_actual{"/px4ctrl/simulink/actual_state"};
    std::string simulink_tracking_error{"/px4ctrl/simulink/tracking_error"};
    std::string simulink_ude_debug{"/px4ctrl/simulink/ude_debug"};
    std::string ude_tune{"/px4ctrl/ude_tune"};
    std::string ude_tune_status{"/px4ctrl/ude_tune_status"};
    std::string ude_tune_status_text{"/px4ctrl/ude_tune_status_text"};
    std::string mocap_state_status{"/px4ctrl/mocap_state_status"};
    std::string expert_pose{"/px4ctrl/expert_pose"};
    std::string gripper_command{"/gripper/command"};
    std::string traj_start_trigger{"/traj_start_trigger"};
    std::string state{"/mavros/state"};
    std::string extended_state{"/mavros/extended_state"};
    std::string battery{"/mavros/battery"};
  };

  struct Services
  {
    std::string set_mode{"/mavros/set_mode"};
    std::string arming{"/mavros/cmd/arming"};
    std::string command{"/mavros/cmd/command"};
  };

  struct Limits
  {
    double x_min{-7.0};
    double x_max{14.0};
    double y_min{-2.5};
    double y_max{2.5};
    double z_min{-0.3};
    double z_max{2.5};
  };

  struct Gripper
  {
    int rc_channel{10};
    int pwm_open{1300};
    int pwm_close{1700};
    double open_position{100.0};
    double closed_position{0.0};
    double force_open_below_z{0.15};
  };

  struct CmdFeedforward
  {
    bool enable{false};
    double max_velocity{1.0};
    double max_acceleration{2.0};
  };

  struct MocapState
  {
    bool enable{true};
    std::string twist_frame{"map"};
    double max_pair_dt_s{0.02};
    double max_odom_attitude_dt_s{0.03};
    bool fallback_to_odom{true};
  };

  struct TrackingDifferentiator
  {
    bool enable{true};
    std::array<double, 3> r_diag{2.0, 2.0, 2.0};
  };

  struct Controller
  {
    double gravity{9.81};
    double max_angle_deg{25.0};
    double max_bodyrate_x{2.0};
    double max_bodyrate_y{2.0};
    double max_bodyrate_z{1.5};
    double min_thrust{0.05};
    double max_thrust{0.90};
  };

  struct Ude
  {
    bool enable{true};
    std::array<double, 3> Kp_diag{1.0, 1.0, 1.0};
    std::array<double, 3> Kd_diag{2.0, 2.0, 2.0};
    std::array<double, 3> T_diag{1.0, 1.0, 1.0};
    std::array<double, 3> max_f_hat{3.0, 3.0, 3.0};
    std::array<double, 3> max_u_acc{4.0, 4.0, 4.0};
  };

  struct Attitude
  {
    std::array<double, 3> KAng_diag{8.0, 8.0, 4.0};
  };

  struct ThrustModel
  {
    double hover_thrust{0.5};
    bool enable_estimation{false};
    bool print_value{false};
    double rho2{0.998};
    double min_thr2acc{5.0};
    double max_thr2acc{40.0};
  };

  MsgTimeout msg_timeout;
  RCReverse rc_reverse;
  AutoTakeoffLand takeoff_land;
  Topics topics;
  Services services;
  Limits limits;
  Gripper gripper;
  CmdFeedforward cmd_feedforward;
  MocapState mocap_state;
  TrackingDifferentiator td;
  Controller controller;
  Ude ude;
  Attitude attitude;
  ThrustModel thrust_model;

  double ctrl_freq_max{100.0};
  double max_manual_vel{1.0};
  bool use_bodyrate_ctrl{true};
  std::string frame_id{"map"};

  Parameter_t() = default;
  void config_from_ros_node(rclcpp::Node &node);
  rcl_interfaces::msg::SetParametersResult apply_runtime_parameters(
    const std::vector<rclcpp::Parameter> &params,
    bool *thrust_mapping_changed = nullptr);
};

#endif
