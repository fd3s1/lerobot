// #include "controller.h"

// using namespace std;



// LinearControl::LinearControl(Parameter_t &param) : param_(param)
// {
//   //resetThrustMapping();
// }

// /* 
//   compute u.thrust and u.q, controller gains and other parameters are in param_ 
// */
// geometry_msgs::PoseStamped
// LinearControl::calculateControl(const Desired_State_t &des,
//     const Odom_Data_t &odom,
//     Controller_Output_t &u)
// {
//   u.position=des.p;
//   //geometry_msgs::PoseStamped debug_msg;
//   debug_msg_.header.stamp=ros::Time::now();
//   debug_msg_.header.frame_id="world";
//   debug_msg_.pose.position.x=des.p(0);
//   debug_msg_.pose.position.y=des.p(1);
//   debug_msg_.pose.position.z=des.p(2);
//   return debug_msg_;
// }







