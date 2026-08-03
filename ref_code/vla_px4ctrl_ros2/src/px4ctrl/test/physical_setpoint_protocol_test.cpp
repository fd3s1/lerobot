#include "physical_setpoint_protocol.h"

#include <array>
#include <cstdint>

#include <gtest/gtest.h>

TEST(PhysicalSetpointProtocol, GoldenVector)
{
  physical_setpoint_protocol::Sample sample;
  sample.version = physical_setpoint_protocol::kVersionV1;
  sample.mode = physical_setpoint_protocol::kModeAttitude;
  sample.valid_flags = physical_setpoint_protocol::kRequiredValidFlagsV1;
  sample.sequence = 0x01020304U;
  sample.source_time_us = 0x0102030405060708ULL;
  sample.q_d_wxyz = {1.0F, 0.0F, 0.0F, 0.0F};
  sample.body_rate_d = {2.0F, -2.0F, 4.0F};
  sample.total_thrust_n = 16.0F;

  std::array<std::uint8_t, physical_setpoint_protocol::kPayloadCapacity> payload{};
  ASSERT_TRUE(physical_setpoint_protocol::encode(sample, payload));

  const std::array<std::uint8_t, physical_setpoint_protocol::kPayloadSizeV1> expected{
    0x50, 0x43, 0x54, 0x4c, 0x01, 0x00, 0x07, 0x00,
    0x04, 0x03, 0x02, 0x01, 0x08, 0x07, 0x06, 0x05,
    0x04, 0x03, 0x02, 0x01, 0x00, 0x00, 0x80, 0x3f,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
    0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x80, 0x40,
    0x00, 0x00, 0x80, 0x41};

  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(payload[i], expected[i]) << "byte offset " << i;
  }
  for (std::size_t i = expected.size(); i < payload.size(); ++i) {
    EXPECT_EQ(payload[i], 0U) << "padding byte offset " << i;
  }
}

TEST(PhysicalSetpointProtocol, Version2GoldenVector)
{
  physical_setpoint_protocol::Sample sample;
  sample.version = physical_setpoint_protocol::kVersionV2;
  sample.mode = physical_setpoint_protocol::kModeAttitude;
  sample.valid_flags =
    physical_setpoint_protocol::kRequiredValidFlagsV2 |
    physical_setpoint_protocol::kAngularAccelerationValid;
  sample.sequence = 0x01020304U;
  sample.source_time_us = 0x0102030405060708ULL;
  sample.q_d_wxyz = {1.0F, 0.0F, 0.0F, 0.0F};
  sample.body_rate_d = {2.0F, -2.0F, 4.0F};
  sample.total_thrust_n = 16.0F;
  sample.body_rate_dot_d = {120.0F, -60.0F, 0.5F};

  std::array<std::uint8_t, physical_setpoint_protocol::kPayloadCapacity> payload{};
  ASSERT_TRUE(physical_setpoint_protocol::encode(sample, payload));

  const std::array<std::uint8_t, physical_setpoint_protocol::kPayloadSizeV2> expected{
    0x50, 0x43, 0x54, 0x4c, 0x02, 0x00, 0x0f, 0x00,
    0x04, 0x03, 0x02, 0x01, 0x08, 0x07, 0x06, 0x05,
    0x04, 0x03, 0x02, 0x01, 0x00, 0x00, 0x80, 0x3f,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
    0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x80, 0x40,
    0x00, 0x00, 0x80, 0x41, 0x00, 0x00, 0xf0, 0x42,
    0x00, 0x00, 0x70, 0xc2, 0x00, 0x00, 0x00, 0x3f};

  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(payload[i], expected[i]) << "byte offset " << i;
  }
}

TEST(PhysicalSetpointProtocol, Version2AllowsAngularAccelerationToBeDisabled)
{
  physical_setpoint_protocol::Sample sample;
  sample.version = physical_setpoint_protocol::kVersionV2;
  sample.mode = physical_setpoint_protocol::kModeAttitude;
  sample.valid_flags = physical_setpoint_protocol::kRequiredValidFlagsV2;

  std::array<std::uint8_t, physical_setpoint_protocol::kPayloadCapacity> payload{};
  ASSERT_TRUE(physical_setpoint_protocol::encode(sample, payload));
  EXPECT_EQ(payload[6], 0x07U);
  EXPECT_EQ(payload[7], 0x00U);
}

TEST(PhysicalSetpointProtocol, RejectsUnknownMode)
{
  physical_setpoint_protocol::Sample sample;
  sample.mode = 42;
  std::array<std::uint8_t, physical_setpoint_protocol::kPayloadCapacity> payload{};
  EXPECT_FALSE(physical_setpoint_protocol::encode(sample, payload));
}

TEST(PhysicalSetpointProtocol, RejectsUnknownVersion)
{
  physical_setpoint_protocol::Sample sample;
  sample.version = 3;
  std::array<std::uint8_t, physical_setpoint_protocol::kPayloadCapacity> payload{};
  EXPECT_FALSE(physical_setpoint_protocol::encode(sample, payload));
}
