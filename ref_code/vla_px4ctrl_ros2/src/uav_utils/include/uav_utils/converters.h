#ifndef UAV_UTILS_CONVERTERS_H
#define UAV_UTILS_CONVERTERS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace uav_utils {

inline void extract_odometry(
  const nav_msgs::msg::Odometry &msg,
  Eigen::Vector3d &p,
  Eigen::Vector3d &v,
  Eigen::Quaterniond &q)
{
  p(0) = msg.pose.pose.position.x;
  p(1) = msg.pose.pose.position.y;
  p(2) = msg.pose.pose.position.z;

  v(0) = msg.twist.twist.linear.x;
  v(1) = msg.twist.twist.linear.y;
  v(2) = msg.twist.twist.linear.z;

  q.w() = msg.pose.pose.orientation.w;
  q.x() = msg.pose.pose.orientation.x;
  q.y() = msg.pose.pose.orientation.y;
  q.z() = msg.pose.pose.orientation.z;
}

inline void extract_odometry(
  const geometry_msgs::msg::PoseStamped &msg,
  Eigen::Vector3d &p,
  Eigen::Quaterniond &q)
{
  p(0) = msg.pose.position.x;
  p(1) = msg.pose.position.y;
  p(2) = msg.pose.position.z;

  q.w() = msg.pose.orientation.w;
  q.x() = msg.pose.orientation.x;
  q.y() = msg.pose.orientation.y;
  q.z() = msg.pose.orientation.z;
}

inline Eigen::Quaterniond from_quaternion_msg(const geometry_msgs::msg::Quaternion &msg)
{
  return Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
}

inline geometry_msgs::msg::Quaternion to_quaternion_msg(const Eigen::Quaterniond &q)
{
  geometry_msgs::msg::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

inline Eigen::Vector3d from_vector3_msg(const geometry_msgs::msg::Vector3 &msg)
{
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline geometry_msgs::msg::Vector3 to_vector3_msg(const Eigen::Vector3d &v)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
  return msg;
}

inline geometry_msgs::msg::Point to_point_msg(const Eigen::Vector3d &v)
{
  geometry_msgs::msg::Point msg;
  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
  return msg;
}

}  // namespace uav_utils

#endif
