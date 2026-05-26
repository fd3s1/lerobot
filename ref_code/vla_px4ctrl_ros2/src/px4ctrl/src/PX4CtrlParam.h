#ifndef PX4CTRL_PARAM_H
#define PX4CTRL_PARAM_H

#include <string>

#include <rclcpp/rclcpp.hpp>

class Parameter_t
{
public:
  struct MsgTimeout
  {
    double odom{0.5};
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
    std::string odom{"/mavros/vision_pose/pose"};
    std::string cmd{"/position_cmd"};
    std::string takeoff_land{"/px4ctrl/takeoff_land"};
    std::string setpoint{"/mavros/setpoint_raw/local"};
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
    double x_min{-6.5};
    double x_max{6.5};
    double y_min{-3.5};
    double y_max{3.5};
    double z_min{-0.3};
    double z_max{3.0};
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

  MsgTimeout msg_timeout;
  RCReverse rc_reverse;
  AutoTakeoffLand takeoff_land;
  Topics topics;
  Services services;
  Limits limits;
  Gripper gripper;
  CmdFeedforward cmd_feedforward;

  double ctrl_freq_max{100.0};
  double max_manual_vel{1.0};
  bool use_bodyrate_ctrl{false};
  std::string frame_id{"map"};

  Parameter_t() = default;
  void config_from_ros_node(rclcpp::Node &node);
};

#endif
