#include <gtest/gtest.h>

#include <limits>

#include "controller.h"

namespace
{

Parameter_t makeTestParameters()
{
  Parameter_t param;
  param.ude.Kp_diag = {0.0, 0.0, 0.0};
  param.ude.Kd_diag = {0.0, 0.0, 0.0};
  param.ude.T_diag = {1.0, 1.0, 1.0};
  param.ude.velocity_lpf_tau_s = {0.0, 0.0, 0.0};
  param.ude.max_f_hat = {100.0, 100.0, 100.0};
  param.ude.max_u_acc = {100.0, 100.0, 100.0};
  param.controller.max_angle_deg = 89.0;
  return param;
}

TEST(ControllerUde, IntegratesFeedforwardAsPartOfNominalInput)
{
  Parameter_t param = makeTestParameters();
  param.ude.enable = true;
  LinearControl controller(param);

  Desired_State_t desired;
  desired.a = Eigen::Vector3d(1.0, 2.0, 0.5);
  Odom_Data_t odom;
  Imu_Data_t imu;
  Controller_Debug_t debug;

  controller.calculateControl(
    desired, odom, imu, rclcpp::Time(1, 0, RCL_ROS_TIME), &debug);
  EXPECT_TRUE(debug.u0.isApprox(desired.a, 1e-12));
  EXPECT_TRUE(debug.integral_u0.isZero(1e-12));
  EXPECT_TRUE(debug.f_hat.isZero(1e-12));

  controller.calculateControl(
    desired, odom, imu, rclcpp::Time(1, 10000000, RCL_ROS_TIME), &debug);
  const Eigen::Vector3d expected_integral = 0.01 * desired.a;
  EXPECT_TRUE(debug.u0.isApprox(desired.a, 1e-12));
  EXPECT_TRUE(debug.integral_u0.isApprox(expected_integral, 1e-12));
  EXPECT_TRUE(debug.f_hat.isApprox(-expected_integral, 1e-12));
  EXPECT_TRUE(debug.u_acc.isApprox(desired.a + expected_integral, 1e-12));
}

TEST(ControllerUde, AppliesFeedforwardOnlyOnceWhenUdeIsDisabled)
{
  Parameter_t param = makeTestParameters();
  param.ude.enable = false;
  LinearControl controller(param);

  Desired_State_t desired;
  desired.a = Eigen::Vector3d(0.0, 0.0, 1.0);
  Odom_Data_t odom;
  Imu_Data_t imu;
  Controller_Debug_t debug;

  const Controller_Output_t output = controller.calculateControl(
    desired, odom, imu, rclcpp::Time(1, 0, RCL_ROS_TIME), &debug);
  EXPECT_TRUE(debug.u0.isApprox(desired.a, 1e-12));
  EXPECT_TRUE(debug.u_acc.isApprox(desired.a, 1e-12));
  EXPECT_NEAR(output.thrust_acc.z(), param.controller.gravity + desired.a.z(), 1e-12);
}

TEST(ControllerFlatnessFeedforward, UsesJerkAndLimitsBodyRateByAxis)
{
  Parameter_t param = makeTestParameters();
  param.ude.enable = false;
  param.attitude.feedback_mode = "full_quaternion";
  param.controller.max_bodyrate_x = 5.0;
  param.controller.max_bodyrate_y = 3.0;
  param.controller.max_bodyrate_z = 5.0;
  LinearControl controller(param);

  Desired_State_t desired;
  desired.j = Eigen::Vector3d(10.0 * param.controller.gravity, 0.0, 0.0);
  Odom_Data_t odom;
  Imu_Data_t imu;

  const Controller_Output_t output = controller.calculateControl(
    desired, odom, imu, rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_NEAR(output.bodyrates_ff.x(), 0.0, 1e-12);
  EXPECT_NEAR(output.bodyrates_ff.y(), param.controller.max_bodyrate_y, 1e-12);
  EXPECT_NEAR(output.bodyrates_ff.z(), 0.0, 1e-12);
}

Controller_Output_t evaluatePolynomialFlatnessReference(
  const Parameter_t &parameters,
  double time_s)
{
  Parameter_t param = parameters;
  LinearControl controller(param);
  Desired_State_t desired;
  const Eigen::Vector3d acceleration_initial(0.25, -0.15, 0.10);
  const Eigen::Vector3d jerk_initial(0.35, 0.20, -0.10);
  const Eigen::Vector3d snap(0.18, -0.12, 0.08);
  desired.a =
    acceleration_initial +
    jerk_initial * time_s +
    0.5 * snap * time_s * time_s;
  desired.j = jerk_initial + snap * time_s;
  desired.snap = snap;
  desired.yaw = 0.2 + 0.4 * time_s + 0.5 * 0.3 * time_s * time_s;
  desired.yaw_rate = 0.4 + 0.3 * time_s;
  desired.yaw_acceleration = 0.3;

  Odom_Data_t odom;
  Imu_Data_t imu;
  return controller.calculateControl(
    desired, odom, imu, rclcpp::Time(1, 0, RCL_ROS_TIME));
}

TEST(ControllerFlatnessFeedforward, AnalyticBodyRateDotMatchesNumericalDerivative)
{
  Parameter_t param = makeTestParameters();
  param.ude.enable = false;
  param.physical_control.angular_acceleration_feedforward_scale = 1.0;
  param.attitude.feedback_mode = "full_quaternion";
  param.controller.max_bodyrate_x = 50.0;
  param.controller.max_bodyrate_y = 50.0;
  param.controller.max_bodyrate_z = 50.0;

  constexpr double time_s = 0.6;
  constexpr double step_s = 1e-4;
  const Controller_Output_t before =
    evaluatePolynomialFlatnessReference(param, time_s - step_s);
  const Controller_Output_t center =
    evaluatePolynomialFlatnessReference(param, time_s);
  const Controller_Output_t after =
    evaluatePolynomialFlatnessReference(param, time_s + step_s);
  const Eigen::Vector3d numerical_rate_dot =
    (after.bodyrates_ff - before.bodyrates_ff) / (2.0 * step_s);

  EXPECT_TRUE(center.bodyrates_dot_ff.allFinite());
  EXPECT_TRUE(center.bodyrates_dot_ff.isApprox(numerical_rate_dot, 2e-5));
}

TEST(ControllerFlatnessFeedforward, UsesFeedbackCorrectedThrustAcceleration)
{
  Parameter_t param = makeTestParameters();
  param.ude.enable = true;
  param.physical_control.angular_acceleration_feedforward_scale = 1.0;
  param.ude.Kp_diag = {1.2, 0.8, 0.5};
  param.ude.Kd_diag = {0.9, 0.7, 0.4};
  param.attitude.feedback_mode = "full_quaternion";
  param.controller.max_bodyrate_x = 50.0;
  param.controller.max_bodyrate_y = 50.0;
  param.controller.max_bodyrate_z = 50.0;

  Desired_State_t desired;
  desired.a = Eigen::Vector3d(0.3, -0.2, 0.1);
  desired.j = Eigen::Vector3d(0.4, 0.25, -0.15);
  desired.snap = Eigen::Vector3d(-0.2, 0.1, 0.08);
  desired.yaw = 0.35;
  desired.yaw_rate = 0.25;
  desired.yaw_acceleration = -0.12;

  Odom_Data_t nominal_odom;
  Odom_Data_t corrected_odom;
  corrected_odom.p = Eigen::Vector3d(-1.0, 0.6, -0.3);
  corrected_odom.v = Eigen::Vector3d(-0.4, 0.2, -0.1);
  Imu_Data_t imu;

  LinearControl nominal_controller(param);
  LinearControl corrected_controller(param);
  const Controller_Output_t nominal = nominal_controller.calculateControl(
    desired, nominal_odom, imu, rclcpp::Time(1, 0, RCL_ROS_TIME));
  const Controller_Output_t corrected = corrected_controller.calculateControl(
    desired, corrected_odom, imu, rclcpp::Time(1, 0, RCL_ROS_TIME));

  EXPECT_FALSE(nominal.q.isApprox(corrected.q, 1e-5));

  EXPECT_FALSE(nominal.bodyrates_ff.isApprox(corrected.bodyrates_ff, 1e-5));
  EXPECT_FALSE(
    nominal.bodyrates_dot_ff.isApprox(corrected.bodyrates_dot_ff, 1e-5));
}

TEST(ControllerFlatnessFeedforward, DisabledAngularAccelerationDoesNotReadSnap)
{
  Parameter_t param = makeTestParameters();
  param.ude.enable = false;
  param.attitude.feedback_mode = "full_quaternion";

  Desired_State_t valid_desired;
  valid_desired.a = Eigen::Vector3d(1.0, -0.5, 0.2);
  valid_desired.yaw = 0.4;
  Desired_State_t invalid_desired = valid_desired;
  invalid_desired.snap.y() = std::numeric_limits<double>::infinity();

  Odom_Data_t odom;
  Imu_Data_t imu;
  LinearControl valid_controller(param);
  LinearControl invalid_controller(param);
  const Controller_Output_t valid = valid_controller.calculateControl(
    valid_desired, odom, imu, rclcpp::Time(1, 0, RCL_ROS_TIME));
  const Controller_Output_t invalid = invalid_controller.calculateControl(
    invalid_desired, odom, imu, rclcpp::Time(1, 0, RCL_ROS_TIME));

  EXPECT_TRUE(valid.q.isApprox(invalid.q, 1e-12));
  EXPECT_TRUE(valid.bodyrates_ff.isApprox(invalid.bodyrates_ff, 1e-12));
  EXPECT_TRUE(invalid.bodyrates_dot_ff.isZero(1e-12));
}

}  // namespace
