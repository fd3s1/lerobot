#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

RC_TOPIC="${RC_TOPIC:-/mavros/rc/in}"
IMU_TOPIC="${IMU_TOPIC:-/mavros/imu/data}"
ODOM_TOPIC="${ODOM_TOPIC:-/mavros/local_position/odom}"
SETPOINT_TOPIC="${SETPOINT_TOPIC:-/mavros/setpoint_raw/attitude}"
RATE_HZ="${RATE_HZ:-100.0}"

TEST_ATT_CHANNEL="${TEST_ATT_CHANNEL:-11}"
ACTIVE_THRESHOLD="${ACTIVE_THRESHOLD:-0.75}"
RC_TIMEOUT_S="${RC_TIMEOUT_S:-0.3}"
IMU_TIMEOUT_S="${IMU_TIMEOUT_S:-0.3}"
ODOM_TIMEOUT_S="${ODOM_TIMEOUT_S:-0.3}"

STICK_DEADZONE="${STICK_DEADZONE:-0.08}"
STICK_EXPO="${STICK_EXPO:-1.7}"
ROLL_REVERSE="${ROLL_REVERSE:-false}"
PITCH_REVERSE="${PITCH_REVERSE:-true}"
YAW_REVERSE="${YAW_REVERSE:-false}"
THROTTLE_REVERSE="${THROTTLE_REVERSE:-true}"

MAX_ROLL_RATE_DPS="${MAX_ROLL_RATE_DPS:-45.0}"
MAX_PITCH_RATE_DPS="${MAX_PITCH_RATE_DPS:-45.0}"
MAX_YAW_RATE_DPS="${MAX_YAW_RATE_DPS:-60.0}"
MAX_ROLL_DEG="${MAX_ROLL_DEG:-35.0}"
MAX_PITCH_DEG="${MAX_PITCH_DEG:-35.0}"

THRUST_BASE="${THRUST_BASE:-0.35}"
THRUST_MIN="${THRUST_MIN:-0.20}"
THRUST_MAX="${THRUST_MAX:-0.45}"
THRUST_SLEW_PER_S="${THRUST_SLEW_PER_S:-0.20}"
FRAME_ID="${FRAME_ID:-map}"

as_float() {
  printf "%.6f" "$1"
}

RATE_HZ_P="$(as_float "${RATE_HZ}")"
ACTIVE_THRESHOLD_P="$(as_float "${ACTIVE_THRESHOLD}")"
RC_TIMEOUT_S_P="$(as_float "${RC_TIMEOUT_S}")"
IMU_TIMEOUT_S_P="$(as_float "${IMU_TIMEOUT_S}")"
ODOM_TIMEOUT_S_P="$(as_float "${ODOM_TIMEOUT_S}")"
STICK_DEADZONE_P="$(as_float "${STICK_DEADZONE}")"
STICK_EXPO_P="$(as_float "${STICK_EXPO}")"
MAX_ROLL_RATE_DPS_P="$(as_float "${MAX_ROLL_RATE_DPS}")"
MAX_PITCH_RATE_DPS_P="$(as_float "${MAX_PITCH_RATE_DPS}")"
MAX_YAW_RATE_DPS_P="$(as_float "${MAX_YAW_RATE_DPS}")"
MAX_ROLL_DEG_P="$(as_float "${MAX_ROLL_DEG}")"
MAX_PITCH_DEG_P="$(as_float "${MAX_PITCH_DEG}")"
THRUST_BASE_P="$(as_float "${THRUST_BASE}")"
THRUST_MIN_P="$(as_float "${THRUST_MIN}")"
THRUST_MAX_P="$(as_float "${THRUST_MAX}")"
THRUST_SLEW_PER_S_P="$(as_float "${THRUST_SLEW_PER_S}")"

set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/opt/ros/humble/setup.bash
  source /opt/ros/humble/setup.bash
fi

if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WORKSPACE_DIR}/install/setup.bash"
else
  set -u
  echo "[test-att] warning: ${WORKSPACE_DIR}/install/setup.bash not found"
  echo "[test-att] build first with: colcon build --packages-select px4ctrl"
fi
set -u

echo "[test-att] rc=${RC_TOPIC} imu=${IMU_TOPIC} odom=${ODOM_TOPIC} setpoint=${SETPOINT_TOPIC}"
echo "[test-att] CH${TEST_ATT_CHANNEL} active_threshold=${ACTIVE_THRESHOLD} rate=${RATE_HZ}Hz"
echo "[test-att] stick deadzone=${STICK_DEADZONE} expo=${STICK_EXPO} reverse roll=${ROLL_REVERSE} pitch=${PITCH_REVERSE} yaw=${YAW_REVERSE} throttle=${THROTTLE_REVERSE}"
echo "[test-att] rate limits dps: roll=${MAX_ROLL_RATE_DPS} pitch=${MAX_PITCH_RATE_DPS} yaw=${MAX_YAW_RATE_DPS}; angle limits deg: roll=${MAX_ROLL_DEG} pitch=${MAX_PITCH_DEG}"
echo "[test-att] thrust base=${THRUST_BASE} min=${THRUST_MIN} max=${THRUST_MAX} slew=${THRUST_SLEW_PER_S}/s"
echo "[test-att] this script does not arm, set mode, start px4ctrl, or publish /position_cmd."

exec ros2 run px4ctrl test_attitude_mode_node --ros-args \
  -p rc_topic:="${RC_TOPIC}" \
  -p imu_topic:="${IMU_TOPIC}" \
  -p odom_topic:="${ODOM_TOPIC}" \
  -p setpoint_topic:="${SETPOINT_TOPIC}" \
  -p rate_hz:="${RATE_HZ_P}" \
  -p test_att_channel:="${TEST_ATT_CHANNEL}" \
  -p active_threshold:="${ACTIVE_THRESHOLD_P}" \
  -p rc_timeout_s:="${RC_TIMEOUT_S_P}" \
  -p imu_timeout_s:="${IMU_TIMEOUT_S_P}" \
  -p odom_timeout_s:="${ODOM_TIMEOUT_S_P}" \
  -p stick_deadzone:="${STICK_DEADZONE_P}" \
  -p stick_expo:="${STICK_EXPO_P}" \
  -p roll_reverse:="${ROLL_REVERSE}" \
  -p pitch_reverse:="${PITCH_REVERSE}" \
  -p yaw_reverse:="${YAW_REVERSE}" \
  -p throttle_reverse:="${THROTTLE_REVERSE}" \
  -p max_roll_rate_dps:="${MAX_ROLL_RATE_DPS_P}" \
  -p max_pitch_rate_dps:="${MAX_PITCH_RATE_DPS_P}" \
  -p max_yaw_rate_dps:="${MAX_YAW_RATE_DPS_P}" \
  -p max_roll_deg:="${MAX_ROLL_DEG_P}" \
  -p max_pitch_deg:="${MAX_PITCH_DEG_P}" \
  -p thrust_base:="${THRUST_BASE_P}" \
  -p thrust_min:="${THRUST_MIN_P}" \
  -p thrust_max:="${THRUST_MAX_P}" \
  -p thrust_slew_per_s:="${THRUST_SLEW_PER_S_P}" \
  -p frame_id:="${FRAME_ID}" \
  "$@"
