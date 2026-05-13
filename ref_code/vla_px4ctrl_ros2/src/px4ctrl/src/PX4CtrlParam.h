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
    std::string rc{"/drone6/mavros/rc/in"};
    std::string odom{"/drone6/mavros/vision_pose/pose"};
    std::string cmd{"/drone6/position_cmd"};
    std::string takeoff_land{"/drone6/px4ctrl/takeoff_land"};
    std::string setpoint{"/drone6/mavros/setpoint_raw/local"};
    std::string gripper_command{"/drone6/gripper/command"};
    std::string traj_start_trigger{"/drone6/traj_start_trigger"};
    std::string state{"/drone6/mavros/state"};
    std::string extended_state{"/drone6/mavros/extended_state"};
    std::string battery{"/drone6/mavros/battery"};
  };

  struct Services
  {
    std::string set_mode{"/drone6/mavros/set_mode"};
    std::string arming{"/drone6/mavros/cmd/arming"};
    std::string command{"/drone6/mavros/cmd/command"};
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
  };

  MsgTimeout msg_timeout;
  RCReverse rc_reverse;
  AutoTakeoffLand takeoff_land;
  Topics topics;
  Services services;
  Limits limits;
  Gripper gripper;

  double ctrl_freq_max{100.0};
  double max_manual_vel{1.0};
  bool use_bodyrate_ctrl{false};
  std::string frame_id{"map"};

  Parameter_t() = default;
  void config_from_ros_node(rclcpp::Node &node);
};

#endif
