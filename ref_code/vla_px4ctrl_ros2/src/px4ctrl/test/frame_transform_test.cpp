#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <mavros/frame_tf.hpp>

TEST(PhysicalSetpointFrameTransform, BodyVectorFluToFrd)
{
  const Eigen::Vector3d flu(1.0, 2.0, 3.0);
  const Eigen::Vector3d frd =
    mavros::ftf::transform_frame_baselink_aircraft(flu);
  EXPECT_TRUE(frd.isApprox(Eigen::Vector3d(1.0, -2.0, -3.0), 1e-12));
}

TEST(PhysicalSetpointFrameTransform, AttitudeRoundTrip)
{
  Eigen::Quaterniond enu_flu =
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX());
  enu_flu.normalize();

  const Eigen::Quaterniond ned_frd =
    mavros::ftf::transform_orientation_enu_ned(
      mavros::ftf::transform_orientation_baselink_aircraft(enu_flu));
  Eigen::Quaterniond recovered =
    mavros::ftf::transform_orientation_aircraft_baselink(
      mavros::ftf::transform_orientation_ned_enu(ned_frd));
  recovered.normalize();

  EXPECT_NEAR(std::abs(enu_flu.dot(recovered)), 1.0, 1e-12);
}
