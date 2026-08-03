#ifndef PX4CTRL_PHYSICAL_SETPOINT_PROTOCOL_H
#define PX4CTRL_PHYSICAL_SETPOINT_PROTOCOL_H

#include <array>
#include <cstddef>
#include <cstdint>

namespace physical_setpoint_protocol
{

constexpr std::size_t kPayloadCapacity = 128;
constexpr std::size_t kPayloadSizeV1 = 52;
constexpr std::size_t kPayloadSizeV2 = 64;
constexpr std::uint8_t kVersionV1 = 1;
constexpr std::uint8_t kVersionV2 = 2;
constexpr std::uint16_t kPayloadType = 42001;

constexpr std::uint8_t kModeAttitude = 0;
constexpr std::uint8_t kModeBodyrate = 1;

constexpr std::uint16_t kQuaternionValid = 1u << 0;
constexpr std::uint16_t kBodyRateValid = 1u << 1;
constexpr std::uint16_t kPhysicalThrustValid = 1u << 2;
constexpr std::uint16_t kAngularAccelerationValid = 1u << 3;
constexpr std::uint16_t kRequiredValidFlagsV1 =
  kQuaternionValid | kBodyRateValid | kPhysicalThrustValid;
// Version 2 carries the angular-acceleration field, but its valid flag is
// optional so scale=0 is a true feedforward-off command.
constexpr std::uint16_t kRequiredValidFlagsV2 = kRequiredValidFlagsV1;

struct Sample
{
  std::uint8_t version{kVersionV2};
  std::uint8_t mode{kModeAttitude};
  std::uint16_t valid_flags{0};
  std::uint32_t sequence{0};
  std::uint64_t source_time_us{0};
  std::array<float, 4> q_d_wxyz{1.0F, 0.0F, 0.0F, 0.0F};
  std::array<float, 3> body_rate_d{0.0F, 0.0F, 0.0F};
  float total_thrust_n{0.0F};
  std::array<float, 3> body_rate_dot_d{0.0F, 0.0F, 0.0F};
};

bool encode(const Sample &sample, std::array<std::uint8_t, kPayloadCapacity> &payload);
std::size_t payload_size(std::uint8_t version);

}  // namespace physical_setpoint_protocol

#endif
