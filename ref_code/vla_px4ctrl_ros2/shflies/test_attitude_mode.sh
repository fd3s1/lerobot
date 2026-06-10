#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

RC_TOPIC="${RC_TOPIC:-/mavros/rc/in}"
IMU_TOPIC="${IMU_TOPIC:-/mavros/imu/data}"
ODOM_TOPIC="${ODOM_TOPIC:-/mavros/local_position/odom}"
STATE_TOPIC="${STATE_TOPIC:-/mavros/state}"
SETPOINT_TOPIC="${SETPOINT_TOPIC:-/mavros/setpoint_raw/attitude}"
SET_MODE_SERVICE="${SET_MODE_SERVICE:-/mavros/set_mode}"
ARMING_SERVICE="${ARMING_SERVICE:-/mavros/cmd/arming}"
RATE_HZ="${RATE_HZ:-100.0}"

TEST_ATT_CHANNEL="${TEST_ATT_CHANNEL:-11}"
ACTIVE_THRESHOLD="${ACTIVE_THRESHOLD:-0.75}"
RC_TIMEOUT_S="${RC_TIMEOUT_S:-0.3}"
IMU_TIMEOUT_S="${IMU_TIMEOUT_S:-0.3}"
ODOM_TIMEOUT_S="${ODOM_TIMEOUT_S:-0.3}"
STATE_TIMEOUT_S="${STATE_TIMEOUT_S:-1.0}"
AUTO_ARM_ENABLE="${AUTO_ARM_ENABLE:-true}"
AUTO_OFFBOARD_ENABLE="${AUTO_OFFBOARD_ENABLE:-true}"
REQUIRE_OFFBOARD_FOR_AUTO_ARM="${REQUIRE_OFFBOARD_FOR_AUTO_ARM:-true}"
RESTORE_MODE_ON_INACTIVE="${RESTORE_MODE_ON_INACTIVE:-true}"
DISARM_ON_INACTIVE="${DISARM_ON_INACTIVE:-false}"
ENTER_CONFIRM_ENABLE="${ENTER_CONFIRM_ENABLE:-true}"
ARM_REQUEST_DELAY_S="${ARM_REQUEST_DELAY_S:-0.2}"
ARM_REQUEST_PERIOD_S="${ARM_REQUEST_PERIOD_S:-1.0}"
OFFBOARD_REQUEST_DELAY_S="${OFFBOARD_REQUEST_DELAY_S:-0.5}"
OFFBOARD_REQUEST_PERIOD_S="${OFFBOARD_REQUEST_PERIOD_S:-1.0}"

STICK_DEADZONE="${STICK_DEADZONE:-0.08}"
STICK_EXPO="${STICK_EXPO:-1.7}"
ROLL_REVERSE="${ROLL_REVERSE:-false}"
PITCH_REVERSE="${PITCH_REVERSE:-true}"
YAW_REVERSE="${YAW_REVERSE:-false}"
THROTTLE_REVERSE="${THROTTLE_REVERSE:-false}"

MAX_ROLL_RATE_DPS="${MAX_ROLL_RATE_DPS:-45.0}"
MAX_PITCH_RATE_DPS="${MAX_PITCH_RATE_DPS:-45.0}"
MAX_YAW_RATE_DPS="${MAX_YAW_RATE_DPS:-60.0}"
MAX_ROLL_DEG="${MAX_ROLL_DEG:-35.0}"
MAX_PITCH_DEG="${MAX_PITCH_DEG:-35.0}"
ATTITUDE_KANG_ROLL="${ATTITUDE_KANG_ROLL:-6.0}"
ATTITUDE_KANG_PITCH="${ATTITUDE_KANG_PITCH:-6.0}"
ATTITUDE_KANG_YAW="${ATTITUDE_KANG_YAW:-3.0}"
MAX_BODYRATE_ROLL_DPS="${MAX_BODYRATE_ROLL_DPS:-143.0}"
MAX_BODYRATE_PITCH_DPS="${MAX_BODYRATE_PITCH_DPS:-143.0}"
MAX_BODYRATE_YAW_DPS="${MAX_BODYRATE_YAW_DPS:-86.0}"

THRUST_BASE="${THRUST_BASE:-0.35}"
THRUST_MIN="${THRUST_MIN:-0.20}"
THRUST_MAX="${THRUST_MAX:-0.45}"
THRUST_RAMP_PER_S="${THRUST_RAMP_PER_S:-0.05}"
THRUST_SLEW_PER_S="${THRUST_SLEW_PER_S:-0.20}"
REQUIRE_ARMED_FOR_THRUST_RAMP="${REQUIRE_ARMED_FOR_THRUST_RAMP:-true}"
REQUIRE_OFFBOARD_FOR_THRUST_RAMP="${REQUIRE_OFFBOARD_FOR_THRUST_RAMP:-true}"
SETPOINT_OUTPUT_MODE="${SETPOINT_OUTPUT_MODE:-attitude_bodyrate}"
SETPOINT_ALIGNMENT_MODE="${SETPOINT_ALIGNMENT_MODE:-direct_imu}"
FRAME_ID="${FRAME_ID:-map}"

as_float() {
  printf "%.6f" "$1"
}

RATE_HZ_P="$(as_float "${RATE_HZ}")"
ACTIVE_THRESHOLD_P="$(as_float "${ACTIVE_THRESHOLD}")"
RC_TIMEOUT_S_P="$(as_float "${RC_TIMEOUT_S}")"
IMU_TIMEOUT_S_P="$(as_float "${IMU_TIMEOUT_S}")"
ODOM_TIMEOUT_S_P="$(as_float "${ODOM_TIMEOUT_S}")"
STATE_TIMEOUT_S_P="$(as_float "${STATE_TIMEOUT_S}")"
ARM_REQUEST_DELAY_S_P="$(as_float "${ARM_REQUEST_DELAY_S}")"
ARM_REQUEST_PERIOD_S_P="$(as_float "${ARM_REQUEST_PERIOD_S}")"
OFFBOARD_REQUEST_DELAY_S_P="$(as_float "${OFFBOARD_REQUEST_DELAY_S}")"
OFFBOARD_REQUEST_PERIOD_S_P="$(as_float "${OFFBOARD_REQUEST_PERIOD_S}")"
STICK_DEADZONE_P="$(as_float "${STICK_DEADZONE}")"
STICK_EXPO_P="$(as_float "${STICK_EXPO}")"
MAX_ROLL_RATE_DPS_P="$(as_float "${MAX_ROLL_RATE_DPS}")"
MAX_PITCH_RATE_DPS_P="$(as_float "${MAX_PITCH_RATE_DPS}")"
MAX_YAW_RATE_DPS_P="$(as_float "${MAX_YAW_RATE_DPS}")"
MAX_ROLL_DEG_P="$(as_float "${MAX_ROLL_DEG}")"
MAX_PITCH_DEG_P="$(as_float "${MAX_PITCH_DEG}")"
ATTITUDE_KANG_ROLL_P="$(as_float "${ATTITUDE_KANG_ROLL}")"
ATTITUDE_KANG_PITCH_P="$(as_float "${ATTITUDE_KANG_PITCH}")"
ATTITUDE_KANG_YAW_P="$(as_float "${ATTITUDE_KANG_YAW}")"
MAX_BODYRATE_ROLL_DPS_P="$(as_float "${MAX_BODYRATE_ROLL_DPS}")"
MAX_BODYRATE_PITCH_DPS_P="$(as_float "${MAX_BODYRATE_PITCH_DPS}")"
MAX_BODYRATE_YAW_DPS_P="$(as_float "${MAX_BODYRATE_YAW_DPS}")"
THRUST_BASE_P="$(as_float "${THRUST_BASE}")"
THRUST_MIN_P="$(as_float "${THRUST_MIN}")"
THRUST_MAX_P="$(as_float "${THRUST_MAX}")"
THRUST_RAMP_PER_S_P="$(as_float "${THRUST_RAMP_PER_S}")"
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

echo "[test-att] rc=${RC_TOPIC} imu=${IMU_TOPIC} odom=${ODOM_TOPIC} state=${STATE_TOPIC}"
echo "[test-att] setpoint=${SETPOINT_TOPIC} set_mode=${SET_MODE_SERVICE} arming=${ARMING_SERVICE}"
echo "[test-att] enter_confirm=${ENTER_CONFIRM_ENABLE} auto_offboard=${AUTO_OFFBOARD_ENABLE} auto_arm=${AUTO_ARM_ENABLE} require_offboard_for_auto_arm=${REQUIRE_OFFBOARD_FOR_AUTO_ARM}"
echo "[test-att] restore_mode_on_inactive=${RESTORE_MODE_ON_INACTIVE} disarm_on_inactive=${DISARM_ON_INACTIVE}"
echo "[test-att] request delay/period: offboard=${OFFBOARD_REQUEST_DELAY_S}/${OFFBOARD_REQUEST_PERIOD_S}s arm=${ARM_REQUEST_DELAY_S}/${ARM_REQUEST_PERIOD_S}s"
echo "[test-att] CH${TEST_ATT_CHANNEL} active_threshold=${ACTIVE_THRESHOLD} rate=${RATE_HZ}Hz"
echo "[test-att] stick deadzone=${STICK_DEADZONE} expo=${STICK_EXPO} reverse roll=${ROLL_REVERSE} pitch=${PITCH_REVERSE} yaw=${YAW_REVERSE} throttle=${THROTTLE_REVERSE}"
echo "[test-att] rate limits dps: roll=${MAX_ROLL_RATE_DPS} pitch=${MAX_PITCH_RATE_DPS} yaw=${MAX_YAW_RATE_DPS}; angle limits deg: roll=${MAX_ROLL_DEG} pitch=${MAX_PITCH_DEG}"
echo "[test-att] attitude feedback KAng: roll=${ATTITUDE_KANG_ROLL} pitch=${ATTITUDE_KANG_PITCH} yaw=${ATTITUDE_KANG_YAW}; bodyrate clamp dps: roll=${MAX_BODYRATE_ROLL_DPS} pitch=${MAX_BODYRATE_PITCH_DPS} yaw=${MAX_BODYRATE_YAW_DPS}"
echo "[test-att] thrust base=${THRUST_BASE} min=${THRUST_MIN} max=${THRUST_MAX} ramp=${THRUST_RAMP_PER_S}/s slew=${THRUST_SLEW_PER_S}/s"
echo "[test-att] setpoint_output_mode=${SETPOINT_OUTPUT_MODE} setpoint_alignment_mode=${SETPOINT_ALIGNMENT_MODE}"
echo "[test-att] attitude mode: /test_att/reference_rpy is FCU setpoint, /test_att/desired_rpy is stick-integrated target"
echo "[test-att] bodyrate mode: direct RC bodyrate; attitude_bodyrate mode: IMU attitude feedback -> bodyrate setpoint"
echo "[test-att] this script does not start px4ctrl or publish /position_cmd."
echo "[test-att] trigger sequence: press Enter in this terminal, then raise CH${TEST_ATT_CHANNEL}; node streams setpoint, requests OFFBOARD, arms, and ramps thrust."

exec ros2 run px4ctrl test_attitude_mode_node --ros-args \
  -p rc_topic:="${RC_TOPIC}" \
  -p imu_topic:="${IMU_TOPIC}" \
  -p odom_topic:="${ODOM_TOPIC}" \
  -p state_topic:="${STATE_TOPIC}" \
  -p setpoint_topic:="${SETPOINT_TOPIC}" \
  -p set_mode_service:="${SET_MODE_SERVICE}" \
  -p arming_service:="${ARMING_SERVICE}" \
  -p rate_hz:="${RATE_HZ_P}" \
  -p test_att_channel:="${TEST_ATT_CHANNEL}" \
  -p active_threshold:="${ACTIVE_THRESHOLD_P}" \
  -p rc_timeout_s:="${RC_TIMEOUT_S_P}" \
  -p imu_timeout_s:="${IMU_TIMEOUT_S_P}" \
  -p odom_timeout_s:="${ODOM_TIMEOUT_S_P}" \
  -p state_timeout_s:="${STATE_TIMEOUT_S_P}" \
  -p auto_arm_enable:="${AUTO_ARM_ENABLE}" \
  -p auto_offboard_enable:="${AUTO_OFFBOARD_ENABLE}" \
  -p require_offboard_for_auto_arm:="${REQUIRE_OFFBOARD_FOR_AUTO_ARM}" \
  -p restore_mode_on_inactive:="${RESTORE_MODE_ON_INACTIVE}" \
  -p disarm_on_inactive:="${DISARM_ON_INACTIVE}" \
  -p enter_confirm_enable:="${ENTER_CONFIRM_ENABLE}" \
  -p arm_request_delay_s:="${ARM_REQUEST_DELAY_S_P}" \
  -p arm_request_period_s:="${ARM_REQUEST_PERIOD_S_P}" \
  -p offboard_request_delay_s:="${OFFBOARD_REQUEST_DELAY_S_P}" \
  -p offboard_request_period_s:="${OFFBOARD_REQUEST_PERIOD_S_P}" \
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
  -p attitude_kang_roll:="${ATTITUDE_KANG_ROLL_P}" \
  -p attitude_kang_pitch:="${ATTITUDE_KANG_PITCH_P}" \
  -p attitude_kang_yaw:="${ATTITUDE_KANG_YAW_P}" \
  -p max_bodyrate_roll_dps:="${MAX_BODYRATE_ROLL_DPS_P}" \
  -p max_bodyrate_pitch_dps:="${MAX_BODYRATE_PITCH_DPS_P}" \
  -p max_bodyrate_yaw_dps:="${MAX_BODYRATE_YAW_DPS_P}" \
  -p thrust_base:="${THRUST_BASE_P}" \
  -p thrust_min:="${THRUST_MIN_P}" \
  -p thrust_max:="${THRUST_MAX_P}" \
  -p thrust_ramp_per_s:="${THRUST_RAMP_PER_S_P}" \
  -p thrust_slew_per_s:="${THRUST_SLEW_PER_S_P}" \
  -p require_armed_for_thrust_ramp:="${REQUIRE_ARMED_FOR_THRUST_RAMP}" \
  -p require_offboard_for_thrust_ramp:="${REQUIRE_OFFBOARD_FOR_THRUST_RAMP}" \
  -p setpoint_output_mode:="${SETPOINT_OUTPUT_MODE}" \
  -p setpoint_alignment_mode:="${SETPOINT_ALIGNMENT_MODE}" \
  -p frame_id:="${FRAME_ID}" \
  "$@"
