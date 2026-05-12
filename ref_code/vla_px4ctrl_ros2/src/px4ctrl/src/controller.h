#ifndef PX4CTRL_CONTROLLER_H
#define PX4CTRL_CONTROLLER_H

#include <Eigen/Dense>

#include "PX4CtrlParam.h"
#include "input.h"

struct Desired_State_t
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  double yaw{0.0};

  Desired_State_t() = default;
  explicit Desired_State_t(const Odom_Data_t &odom)
  {
    p = odom.p;
    yaw = uav_utils::normalize_angle(uav_utils::get_yaw_from_quaternion(odom.q));
  }
};

struct Controller_Output_t
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

class LinearControl
{
public:
  explicit LinearControl(Parameter_t &param);
  Controller_Output_t calculateControl(const Desired_State_t &des, const Odom_Data_t &odom);

private:
  Parameter_t &param_;
};

#endif
