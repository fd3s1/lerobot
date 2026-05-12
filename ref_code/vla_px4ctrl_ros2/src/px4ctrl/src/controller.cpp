#include "controller.h"

LinearControl::LinearControl(Parameter_t &param) : param_(param) {}

Controller_Output_t LinearControl::calculateControl(
  const Desired_State_t &des,
  const Odom_Data_t & /*odom*/)
{
  Controller_Output_t u;
  u.position = des.p;
  u.yaw = uav_utils::normalize_angle(des.yaw);
  return u;
}
