#include "physical_setpoint_protocol.h"

#include <cstring>
#include <limits>

namespace physical_setpoint_protocol
{
namespace
{

void put_u16(std::array<std::uint8_t, kPayloadCapacity> &payload, std::size_t &offset, std::uint16_t value)
{
  for (std::size_t i = 0; i < sizeof(value); ++i) {
    payload[offset++] = static_cast<std::uint8_t>((value >> (8U * i)) & 0xffU);
  }
}

void put_u32(std::array<std::uint8_t, kPayloadCapacity> &payload, std::size_t &offset, std::uint32_t value)
{
  for (std::size_t i = 0; i < sizeof(value); ++i) {
    payload[offset++] = static_cast<std::uint8_t>((value >> (8U * i)) & 0xffU);
  }
}

void put_u64(std::array<std::uint8_t, kPayloadCapacity> &payload, std::size_t &offset, std::uint64_t value)
{
  for (std::size_t i = 0; i < sizeof(value); ++i) {
    payload[offset++] = static_cast<std::uint8_t>((value >> (8U * i)) & 0xffU);
  }
}

void put_float(std::array<std::uint8_t, kPayloadCapacity> &payload, std::size_t &offset, float value)
{
  static_assert(sizeof(float) == sizeof(std::uint32_t), "32-bit IEEE-754 float required");
  static_assert(std::numeric_limits<float>::is_iec559, "IEEE-754 float required");
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  put_u32(payload, offset, bits);
}

}  // namespace

bool encode(const Sample &sample, std::array<std::uint8_t, kPayloadCapacity> &payload)
{
  const std::size_t expected_size = payload_size(sample.version);
  if (expected_size == 0 ||
      (sample.mode != kModeAttitude && sample.mode != kModeBodyrate)) {
    return false;
  }

  payload.fill(0);
  std::size_t offset = 0;
  payload[offset++] = static_cast<std::uint8_t>('P');
  payload[offset++] = static_cast<std::uint8_t>('C');
  payload[offset++] = static_cast<std::uint8_t>('T');
  payload[offset++] = static_cast<std::uint8_t>('L');
  payload[offset++] = sample.version;
  payload[offset++] = sample.mode;
  put_u16(payload, offset, sample.valid_flags);
  put_u32(payload, offset, sample.sequence);
  put_u64(payload, offset, sample.source_time_us);

  for (const float value : sample.q_d_wxyz) {
    put_float(payload, offset, value);
  }
  for (const float value : sample.body_rate_d) {
    put_float(payload, offset, value);
  }
  put_float(payload, offset, sample.total_thrust_n);
  if (sample.version == kVersionV2) {
    for (const float value : sample.body_rate_dot_d) {
      put_float(payload, offset, value);
    }
  }
  return offset == expected_size;
}

std::size_t payload_size(std::uint8_t version)
{
  if (version == kVersionV1) {
    return kPayloadSizeV1;
  }
  if (version == kVersionV2) {
    return kPayloadSizeV2;
  }
  return 0;
}

}  // namespace physical_setpoint_protocol
