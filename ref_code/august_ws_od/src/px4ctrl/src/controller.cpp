#include "controller.h"

using namespace std;



LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
  //resetThrustMapping();
}

/* 
  compute u.thrust and u.q, controller gains and other parameters are in param_ 
*/
geometry_msgs::PoseStamped
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    Controller_Output_t &u)
{
  u.position=des.p;
}







