#ifndef UAV_UTILS_GEOMETRY_UTILS_H
#define UAV_UTILS_GEOMETRY_UTILS_H

#include <cassert>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace uav_utils {

template <typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 3> rotx(Scalar_t t)
{
  const Scalar_t ct = std::cos(t);
  const Scalar_t st = std::sin(t);
  Eigen::Matrix<Scalar_t, 3, 3> R;
  R << 1, 0, 0,
       0, ct, -st,
       0, st, ct;
  return R;
}

template <typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 3> roty(Scalar_t t)
{
  const Scalar_t ct = std::cos(t);
  const Scalar_t st = std::sin(t);
  Eigen::Matrix<Scalar_t, 3, 3> R;
  R << ct, 0, st,
       0, 1, 0,
       -st, 0, ct;
  return R;
}

template <typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 3> rotz(Scalar_t t)
{
  const Scalar_t ct = std::cos(t);
  const Scalar_t st = std::sin(t);
  Eigen::Matrix<Scalar_t, 3, 3> R;
  R << ct, -st, 0,
       st, ct, 0,
       0, 0, 1;
  return R;
}

template <typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_ypr(const Eigen::Quaternion<Scalar_t> &q_)
{
  Eigen::Quaternion<Scalar_t> q = q_.normalized();

  Eigen::Matrix<Scalar_t, 3, 1> ypr;
  ypr(2) = std::atan2(
    2 * (q.w() * q.x() + q.y() * q.z()),
    1 - 2 * (q.x() * q.x() + q.y() * q.y()));
  ypr(1) = std::asin(2 * (q.w() * q.y() - q.z() * q.x()));
  ypr(0) = std::atan2(
    2 * (q.w() * q.z() + q.x() * q.y()),
    1 - 2 * (q.y() * q.y() + q.z() * q.z()));

  return ypr;
}

template <typename Scalar_t>
Scalar_t get_yaw_from_quaternion(const Eigen::Quaternion<Scalar_t> &q)
{
  return quaternion_to_ypr(q)(0);
}

template <typename Scalar_t>
Eigen::Quaternion<Scalar_t> yaw_to_quaternion(Scalar_t yaw)
{
  return Eigen::Quaternion<Scalar_t>(rotz(yaw));
}

template <typename Scalar_t>
Scalar_t normalize_angle(Scalar_t a)
{
  int cnt = 0;
  while (true) {
    cnt++;

    if (a < -M_PI) {
      a += M_PI * 2.0;
    } else if (a > M_PI) {
      a -= M_PI * 2.0;
    }

    if (-M_PI <= a && a <= M_PI) {
      break;
    }

    assert(cnt < 10 && "[uav_utils] invalid input angle");
  }

  return a;
}

template <typename Scalar_t>
Scalar_t angle_add(Scalar_t a, Scalar_t b)
{
  return normalize_angle(a + b);
}

template <typename Scalar_t>
Scalar_t yaw_add(Scalar_t a, Scalar_t b)
{
  return angle_add(a, b);
}

template <typename Scalar_t>
Scalar_t toRad(Scalar_t deg)
{
  return deg / 180.0 * M_PI;
}

template <typename Scalar_t>
Scalar_t toDeg(Scalar_t rad)
{
  return rad / M_PI * 180.0;
}

}  // namespace uav_utils

#endif
