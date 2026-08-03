#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

GRASP_PARAMS_FILE="${GRASP_PARAMS_FILE:-${SCRIPT_DIR}/grasp_params.env}"
if [[ -n "${GRASP_PARAMS_FILE}" && -f "${GRASP_PARAMS_FILE}" ]]; then
  # shellcheck source=/dev/null
  source "${GRASP_PARAMS_FILE}"
fi

TARGET_POSE_TOPIC="${TARGET_POSE_TOPIC:-/strawberry_bear/pose}"
BOX_POSE_TOPIC="${BOX_POSE_TOPIC:-/box1/pose}"
DRONE_POSE_TOPIC="${DRONE_POSE_TOPIC:-/mavros/local_position/odom}"
ARRIVAL_POSE_TOPIC="${ARRIVAL_POSE_TOPIC:-/mavros/vision_pose/pose}"
CMD_TOPIC="${CMD_TOPIC:-/position_cmd}"
TRAJ_PLANNER_ENABLE="${TRAJ_PLANNER_ENABLE:-false}"
TRAJ_PLANNER_INPUT_TOPIC="${TRAJ_PLANNER_INPUT_TOPIC:-/position_cmd_raw}"
GRIPPER_TOPIC="${GRIPPER_TOPIC:-/gripper/command}"
GRIPPER_COMMAND_PAIR_TOPIC="${GRIPPER_COMMAND_PAIR_TOPIC:-/gripper/command_pair}"
GRIPPER_FEEDBACK_TOPIC="${GRIPPER_FEEDBACK_TOPIC:-/gripper/feedback}"
HLS_STATUS_TOPIC="${HLS_STATUS_TOPIC:-/hls_gripper/status}"
ATTITUDE_TOPIC="${ATTITUDE_TOPIC:-/mavros/imu/data}"
TAKEOFF_LAND_TOPIC="${TAKEOFF_LAND_TOPIC:-/px4ctrl/takeoff_land}"
PX4CTRL_STATE_TOPIC="${PX4CTRL_STATE_TOPIC:-/px4ctrl/state}"
MAVROS_STATE_TOPIC="${MAVROS_STATE_TOPIC:-/mavros/state}"
TAKEOFF_MODE="${TAKEOFF_MODE:-auto}"
RC_TOPIC="${RC_TOPIC:-/mavros/rc/in}"
RC_TIMEOUT_S="${RC_TIMEOUT_S:-0.5}"
RC_STALE_ACTION="${RC_STALE_ACTION:-warn}"
CH10_INDEX="${CH10_INDEX:-9}"
CH10_OPEN_PWM="${CH10_OPEN_PWM:-1300}"
CH10_CLOSE_PWM="${CH10_CLOSE_PWM:-1700}"
FORCE_OPEN_BELOW_Z="${FORCE_OPEN_BELOW_Z:-disabled}"
FORCE_OPEN_BELOW_Z_GRACE_S="${FORCE_OPEN_BELOW_Z_GRACE_S:-2.0}"

START_GRIPPER_MANAGER="${START_GRIPPER_MANAGER:-true}"
GRIPPER_MANAGER_TYPE="${GRIPPER_MANAGER_TYPE:-hls}"
GRIPPER_MANAGER_PORT="${GRIPPER_MANAGER_PORT:-/dev/ttyACM1}"
HLS_GRAVITY_COMP_PATH="${HLS_GRAVITY_COMP_PATH:-${WORKSPACE_DIR}/install/hls_gripper/share/hls_gripper/config/gravity_compensation.json}"
HLS_SDK_ROOT="${HLS_SDK_ROOT:-}"
HLS_DRY_RUN="${HLS_DRY_RUN:-false}"
HLS_OPEN_SPEED="${HLS_OPEN_SPEED:-24}"
HLS_OPEN_ACC="${HLS_OPEN_ACC:-6}"
HLS_OPEN_TORQUE_LIMIT="${HLS_OPEN_TORQUE_LIMIT:-160}"
HLS_LOW_CURRENT="${HLS_LOW_CURRENT:-28}"
HLS_LIFT_CURRENT="${HLS_LIFT_CURRENT:-1000}"
HLS_MAX_CURRENT="${HLS_MAX_CURRENT:-1700}"
HLS_CENTER_HOLD_CURRENT="${HLS_CENTER_HOLD_CURRENT:-24}"
HLS_CENTER_PUSH_CURRENT="${HLS_CENTER_PUSH_CURRENT:-115}"
HLS_GRIP_CHASE_MIN_CURRENT="${HLS_GRIP_CHASE_MIN_CURRENT:-36}"
HLS_GRIP_CHASE_POSITION_ENABLE="${HLS_GRIP_CHASE_POSITION_ENABLE:-true}"
HLS_GRIP_CHASE_POSITION_SPEED="${HLS_GRIP_CHASE_POSITION_SPEED:-5}"
HLS_GRIP_CHASE_POSITION_ACC="${HLS_GRIP_CHASE_POSITION_ACC:-2}"
HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT="${HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT:-80}"
HLS_GRIP_CHASE_SLIP_RATIO="${HLS_GRIP_CHASE_SLIP_RATIO:-0.03}"
HLS_GRIP_CHASE_POSITION_PERIOD_S="${HLS_GRIP_CHASE_POSITION_PERIOD_S:-0.50}"
HLS_GRIP_CHASE_POSITION_PULSE_S="${HLS_GRIP_CHASE_POSITION_PULSE_S:-0.15}"
HLS_CENTER_TIMEOUT_ACTION="${HLS_CENTER_TIMEOUT_ACTION:-final_grip}"
HLS_CENTER_TIMEOUT_S="${HLS_CENTER_TIMEOUT_S:-3.0}"
HLS_CENTER_STABLE_TIME_S="${HLS_CENTER_STABLE_TIME_S:-0.30}"
HLS_FINAL_GRIP_RAMP_S="${HLS_FINAL_GRIP_RAMP_S:-0.8}"
HLS_MAX_TEMP="${HLS_MAX_TEMP:-85.0}"
HLS_TEMP_WARN_THRESHOLD="${HLS_TEMP_WARN_THRESHOLD:-75.0}"
HLS_LEFT_CURRENT_INWARD_SIGN="${HLS_LEFT_CURRENT_INWARD_SIGN:-1}"
HLS_RIGHT_CURRENT_INWARD_SIGN="${HLS_RIGHT_CURRENT_INWARD_SIGN:-1}"
HLS_MOTION_PROFILE="${HLS_MOTION_PROFILE:-p0}"
HLS_MOTION_PROFILE_INDEX="${HLS_MOTION_PROFILE_INDEX:-}"
HLS_SEARCH_SPEED="${HLS_SEARCH_SPEED:-}"
HLS_SEARCH_ACC="${HLS_SEARCH_ACC:-}"
HLS_SEARCH_TORQUE_LIMIT="${HLS_SEARCH_TORQUE_LIMIT:-}"
HLS_CENTER_GAIN_M_PER_RATIO="${HLS_CENTER_GAIN_M_PER_RATIO:-0.0}"
HLS_CENTER_ERROR_GAIN="${HLS_CENTER_ERROR_GAIN:-2.0}"
HLS_CENTER_DEADBAND_M="${HLS_CENTER_DEADBAND_M:-0.03}"
HLS_CENTER_BIAS="${HLS_CENTER_BIAS:-0.0}"
HLS_CENTER_SIGN="${HLS_CENTER_SIGN:-1.0}"
HLS_CONTACT_CURRENT_THRESHOLD="${HLS_CONTACT_CURRENT_THRESHOLD:-40.0}"
HLS_CONTACT_EXIT_THRESHOLD="${HLS_CONTACT_EXIT_THRESHOLD:-18.0}"
HLS_CONTACT_STRONG_THRESHOLD="${HLS_CONTACT_STRONG_THRESHOLD:-65.0}"
HLS_CONTACT_CONFIRM_CYCLES="${HLS_CONTACT_CONFIRM_CYCLES:-2}"
HLS_SEARCH_CONTACT_CURRENT_THRESHOLD="${HLS_SEARCH_CONTACT_CURRENT_THRESHOLD:-50.0}"
HLS_SEARCH_CONTACT_STRONG_THRESHOLD="${HLS_SEARCH_CONTACT_STRONG_THRESHOLD:-175.0}"
HLS_SEARCH_CONTACT_CONFIRM_CYCLES="${HLS_SEARCH_CONTACT_CONFIRM_CYCLES:-3}"
HLS_SEARCH_CONTACT_MIN_CLOSE_RATIO="${HLS_SEARCH_CONTACT_MIN_CLOSE_RATIO:-0.12}"
HLS_SINGLE_CONTACT_OFFSET_LIMIT_M="${HLS_SINGLE_CONTACT_OFFSET_LIMIT_M:-0.12}"
HLS_SINGLE_CONTACT_TIMEOUT_S="${HLS_SINGLE_CONTACT_TIMEOUT_S:-20.0}"
HLS_SINGLE_CONTACT_LIMIT_RATIO="${HLS_SINGLE_CONTACT_LIMIT_RATIO:-0.97}"
HLS_DRY_RUN_CONTACT_PATTERN="${HLS_DRY_RUN_CONTACT_PATTERN:-both}"
HLS_LEFT_OPEN="${HLS_LEFT_OPEN:-}"
HLS_LEFT_CLEAR="${HLS_LEFT_CLEAR:-}"
HLS_LEFT_CLOSE="${HLS_LEFT_CLOSE:-}"
HLS_RIGHT_OPEN="${HLS_RIGHT_OPEN:-}"
HLS_RIGHT_CLEAR="${HLS_RIGHT_CLEAR:-}"
HLS_RIGHT_CLOSE="${HLS_RIGHT_CLOSE:-}"
HLS_MANAGER_STARTUP_TIMEOUT_S="${HLS_MANAGER_STARTUP_TIMEOUT_S:-5.0}"

RATE_HZ="${RATE_HZ:-20}"
POSE_TIMEOUT_S="${POSE_TIMEOUT_S:-1.5}"
MAX_SPEED="${MAX_SPEED:-0.30}"
APPROACH_SPEED="${APPROACH_SPEED:-0.05}"
LIFT_SPEED="${LIFT_SPEED:-0.10}"
PAYLOAD_LIFT_SPEED="${PAYLOAD_LIFT_SPEED:-0.10}"
PAYLOAD_TRANSFER_SPEED="${PAYLOAD_TRANSFER_SPEED:-0.30}"
POST_GRASP_SETTLE_S="${POST_GRASP_SETTLE_S:-0.10}"
POST_LIFT_SETTLE_S="${POST_LIFT_SETTLE_S:-0.10}"
SMOOTH_TRAJECTORY="${SMOOTH_TRAJECTORY:-true}"
PAYLOAD_LIFT_FORWARD_COMP_M="${PAYLOAD_LIFT_FORWARD_COMP_M:-0.0}"
PAYLOAD_LIFT_COMP_X="${PAYLOAD_LIFT_COMP_X:-0.0}"
PAYLOAD_LIFT_COMP_Y="${PAYLOAD_LIFT_COMP_Y:-0.0}"
PAYLOAD_LIFT_COMP_Z="${PAYLOAD_LIFT_COMP_Z:-0.0}"
GRIPPER_X_OFFSET_M="${GRIPPER_X_OFFSET_M:-0.0}"
GRIPPER_Y_OFFSET_M="${GRIPPER_Y_OFFSET_M:-0.0}"
GRIPPER_Z_OFFSET_M="${GRIPPER_Z_OFFSET_M:-0.25}"
TARGET_HEIGHT_M="${TARGET_HEIGHT_M:-0.30}"
TARGET_GRASP_HEIGHT_M="${TARGET_GRASP_HEIGHT_M:-0.17}"
TARGET_POSE_Z_REFERENCE="${TARGET_POSE_Z_REFERENCE:-center}"
TARGET_HOVER_CLEARANCE_M="${TARGET_HOVER_CLEARANCE_M:-0.45}"
BOX_LENGTH_M="${BOX_LENGTH_M:-0.65}"
BOX_WIDTH_M="${BOX_WIDTH_M:-0.41}"
BOX_HEIGHT_M="${BOX_HEIGHT_M:-0.14}"
BOX_HOVER_GRIPPER_CLEARANCE_M="${BOX_HOVER_GRIPPER_CLEARANCE_M:-0.55}"
BOX_PLACE_BOTTOM_CLEARANCE_M="${BOX_PLACE_BOTTOM_CLEARANCE_M:-0.24}"
TARGET_OFFSET_X_WAS_SET="${TARGET_OFFSET_X+x}"
TARGET_OFFSET_X="${TARGET_OFFSET_X:--0.02}"
TARGET_OFFSET_Y="${TARGET_OFFSET_Y:-0.0}"
TARGET_OFFSET_Z="${TARGET_OFFSET_Z:-0.0}"
TARGET_GRASP_X_BIAS_WAS_SET="${TARGET_GRASP_X_BIAS_M+x}"
TARGET_GRASP_X_BIAS_M="${TARGET_GRASP_X_BIAS_M:--0.02}"
TARGET_PREHOVER_MAP_X_BIAS_WAS_SET="${TARGET_PREHOVER_MAP_X_BIAS_M+x}"
TARGET_PREHOVER_MAP_X_BIAS_M="${TARGET_PREHOVER_MAP_X_BIAS_M:--0.15}"
TARGET_HOVER_MAP_X_BIAS_WAS_SET="${TARGET_HOVER_MAP_X_BIAS_M+x}"
TARGET_HOVER_MAP_X_BIAS_M="${TARGET_HOVER_MAP_X_BIAS_M:-0.0}"
TARGET_GRASP_MAP_X_BIAS_WAS_SET="${TARGET_GRASP_MAP_X_BIAS_M+x}"
TARGET_GRASP_MAP_X_BIAS_M="${TARGET_GRASP_MAP_X_BIAS_M:-0.0}"
BOX_OFFSET_X="${BOX_OFFSET_X:-0.0}"
BOX_OFFSET_Y="${BOX_OFFSET_Y:-0.0}"
BOX_OFFSET_Z="${BOX_OFFSET_Z:-0.0}"
TARGET_HOVER_Z_OFFSET="${TARGET_HOVER_Z_OFFSET:-}"
TARGET_GRASP_Z_OFFSET="${TARGET_GRASP_Z_OFFSET:-}"
TARGET_GRASP_Z_BIAS_M="${TARGET_GRASP_Z_BIAS_M:-0.0}"
BOX_HOVER_Z_OFFSET="${BOX_HOVER_Z_OFFSET:-}"
BOX_PLACE_Z_OFFSET="${BOX_PLACE_Z_OFFSET:-}"
TRANSFER_DRONE_Z_M="${TRANSFER_DRONE_Z_M:-}"
RELEASE_RETREAT_UP_M="${RELEASE_RETREAT_UP_M:-0.3}"
RELEASE_RETREAT_FORWARD_M="${RELEASE_RETREAT_FORWARD_M:-1.0}"
RELEASE_RETREAT_FRAME="${RELEASE_RETREAT_FRAME:-body_forward}"
RETREAT_SPEED="${RETREAT_SPEED:-0.10}"
LANDING_MODE="${LANDING_MODE:-cmd}"
CMD_LAND_SPEED="${CMD_LAND_SPEED:-0.25}"
CMD_LAND_Z="${CMD_LAND_Z:--0.3}"
CMD_LAND_Z_OFFSET_M="${CMD_LAND_Z_OFFSET_M:-0.0}"
COMMAND_STOP_BEFORE_LAND_S="${COMMAND_STOP_BEFORE_LAND_S:-1.2}"
AUTO_LAND_TIMEOUT_S="${AUTO_LAND_TIMEOUT_S:-45.0}"
NO_LAND="${NO_LAND:-false}"
ENABLE_RECORD="${ENABLE_RECORD:-false}"
RECORD_STATUS_TOPIC="${RECORD_STATUS_TOPIC:-/lerobot_record/status}"
RECORD_GATE_TOPIC="${RECORD_GATE_TOPIC:-/auto_hls_grasp_place/record_gate}"
RECORD_GATE_VALUE="${RECORD_GATE_VALUE:-START}"
RECORD_STOP_TOPIC="${RECORD_STOP_TOPIC:-/auto_hls_grasp_place/record_stop}"
RECORD_STOP_VALUE="${RECORD_STOP_VALUE:-STOP}"
RECORD_STOP_HOLD_S="${RECORD_STOP_HOLD_S:-3.0}"
POST_RELEASE_HOLD_STOP_S="${POST_RELEASE_HOLD_STOP_S:-0.0}"
RECORD_READY_TIMEOUT_S="${RECORD_READY_TIMEOUT_S:-60.0}"
RECORD_DURATION_S="${RECORD_DURATION_S:-${EPISODE_TIME_S:-30}}"
RECORD_START_HOLD_S="${RECORD_START_HOLD_S:-0.06}"

HLS_GRASP_TIMEOUT_S="${HLS_GRASP_TIMEOUT_S:-25.0}"
HLS_GRASP_MAX_RETRIES="${HLS_GRASP_MAX_RETRIES:-2}"
HLS_STATUS_TIMEOUT_S="${HLS_STATUS_TIMEOUT_S:-0.8}"
CENTER_DEADBAND_M="${CENTER_DEADBAND_M:-0.03}"
CENTER_KP="${CENTER_KP:-1.2}"
CENTER_VMAX_MPS="${CENTER_VMAX_MPS:-0.10}"
CENTER_OFFSET_MAX_M="${CENTER_OFFSET_MAX_M:-0.30}"
HLS_OFFSET_LIMIT_GRACE_S="${HLS_OFFSET_LIMIT_GRACE_S:-5.0}"
CENTER_COMMAND_SIGN="${CENTER_COMMAND_SIGN:-1.0}"
HLS_STATUS_CENTER_MISMATCH_TOL_M="${HLS_STATUS_CENTER_MISMATCH_TOL_M:-0.015}"
SINGLE_CONTACT_VMAX_MPS="${SINGLE_CONTACT_VMAX_MPS:-0.015}"
SINGLE_CONTACT_OFFSET_MAX_M="${SINGLE_CONTACT_OFFSET_MAX_M:-0.10}"
SINGLE_CONTACT_BODY_Y_SIGN="${SINGLE_CONTACT_BODY_Y_SIGN:-1.0}"
ABORT_RISE_M="${ABORT_RISE_M:-0.25}"
ABORT_RISE_SPEED="${ABORT_RISE_SPEED:-0.12}"
OPEN_COMMAND="${OPEN_COMMAND:-100.0}"
CLOSE_COMMAND="${CLOSE_COMMAND:-0.0}"
PRE_GRASP_HOLD_S="${PRE_GRASP_HOLD_S:-0.05}"
RELEASE_AT_BOX_HOVER="${RELEASE_AT_BOX_HOVER:-true}"

WAYPOINT_ARRIVAL_TOLERANCE_M="${WAYPOINT_ARRIVAL_TOLERANCE_M:-0.12}"
WAYPOINT_ARRIVAL_XY_TOLERANCE_M="${WAYPOINT_ARRIVAL_XY_TOLERANCE_M:-${WAYPOINT_ARRIVAL_TOLERANCE_M}}"
WAYPOINT_ARRIVAL_Z_TOLERANCE_M="${WAYPOINT_ARRIVAL_Z_TOLERANCE_M:-${WAYPOINT_ARRIVAL_TOLERANCE_M}}"
TARGET_ARRIVAL_XY_TOLERANCE_M="${TARGET_ARRIVAL_XY_TOLERANCE_M:-0.035}"
TARGET_ARRIVAL_Z_TOLERANCE_M="${TARGET_ARRIVAL_Z_TOLERANCE_M:-0.035}"
TARGET_ARRIVAL_MAX_POSITIVE_X_ERROR_M="${TARGET_ARRIVAL_MAX_POSITIVE_X_ERROR_M:-0.010}"
BOX_ARRIVAL_XY_TOLERANCE_M="${BOX_ARRIVAL_XY_TOLERANCE_M:-0.15}"
BOX_ARRIVAL_Z_TOLERANCE_M="${BOX_ARRIVAL_Z_TOLERANCE_M:-0.10}"
PREGRASP_ARRIVAL_XY_TOLERANCE_M="${PREGRASP_ARRIVAL_XY_TOLERANCE_M:-0.060}"
PREGRASP_ARRIVAL_Z_TOLERANCE_M="${PREGRASP_ARRIVAL_Z_TOLERANCE_M:-0.050}"
PREGRASP_Z_SPEED_MPS="${PREGRASP_Z_SPEED_MPS:-0.10}"
WAYPOINT_ARRIVAL_SETTLE_S="${WAYPOINT_ARRIVAL_SETTLE_S:-0.4}"
TARGET_ARRIVAL_SETTLE_S="${TARGET_ARRIVAL_SETTLE_S:-0.05}"
PREGRASP_ARRIVAL_SETTLE_S="${PREGRASP_ARRIVAL_SETTLE_S:-0.05}"
BOX_ARRIVAL_SETTLE_S="${BOX_ARRIVAL_SETTLE_S:-0.0}"
WAYPOINT_ARRIVAL_TIMEOUT_S="${WAYPOINT_ARRIVAL_TIMEOUT_S:-30.0}"
MOCAP_CORRECTION_ENABLE="${MOCAP_CORRECTION_ENABLE:-false}"
MOCAP_CORRECTION_MAX_XY_M="${MOCAP_CORRECTION_MAX_XY_M:-0.25}"
MOCAP_CORRECTION_MAX_Z_M="${MOCAP_CORRECTION_MAX_Z_M:-0.15}"
MOCAP_CORRECTION_VXY_MPS="${MOCAP_CORRECTION_VXY_MPS:-0.08}"
MOCAP_CORRECTION_VZ_MPS="${MOCAP_CORRECTION_VZ_MPS:-0.04}"
MOCAP_CORRECTION_HLS_XY_VMAX_MPS="${MOCAP_CORRECTION_HLS_XY_VMAX_MPS:-0.02}"
MOCAP_CORRECTION_FREEZE_Z_ON_CONTACT="${MOCAP_CORRECTION_FREEZE_Z_ON_CONTACT:-true}"
MOCAP_CORRECTION_BODY_FRAME="${MOCAP_CORRECTION_BODY_FRAME:-true}"
BODY_FRAME_YAW_SOURCE="${BODY_FRAME_YAW_SOURCE:-drone_control}"
BODY_FRAME_YAW_OFFSET_RAD="${BODY_FRAME_YAW_OFFSET_RAD:-0.0}"
TARGET_YAW_ALIGN_ENABLE="${TARGET_YAW_ALIGN_ENABLE:-true}"
TARGET_YAW_ALIGN_OFFSET_RAD="${TARGET_YAW_ALIGN_OFFSET_RAD:-0.0}"
TARGET_YAW_ALIGN_RATE_DPS="${TARGET_YAW_ALIGN_RATE_DPS:-30.0}"
TARGET_YAW_ALIGN_TOL_DEG="${TARGET_YAW_ALIGN_TOL_DEG:-12.0}"
TARGET_YAW_ALIGN_MIN_DISTANCE_M="${TARGET_YAW_ALIGN_MIN_DISTANCE_M:-0.20}"
TARGET_YAW_ALIGN_MAX_DURATION_S="${TARGET_YAW_ALIGN_MAX_DURATION_S:-5.0}"
POST_TAKEOFF_SETTLE_S="${POST_TAKEOFF_SETTLE_S:-2.0}"
CONFIRM_BEFORE_TAKEOFF="${CONFIRM_BEFORE_TAKEOFF:-false}"
POSE_PREFLIGHT_TIMEOUT_S="${POSE_PREFLIGHT_TIMEOUT_S:-6}"
POSE_PREFLIGHT_RETRIES="${POSE_PREFLIGHT_RETRIES:-3}"
POSE_PREFLIGHT_REQUIRED="${POSE_PREFLIGHT_REQUIRED:-false}"
SKIP_POSE_PREFLIGHT="${SKIP_POSE_PREFLIGHT:-false}"
PREFLIGHT_CHECKER="${PREFLIGHT_CHECKER:-${WORKSPACE_DIR}/src/px4ctrl/scripts/topic_liveness_check.py}"
PREFLIGHT_PYTHON="${PREFLIGHT_PYTHON:-/usr/bin/python3}"

gripper_manager_pid=""

bool_is_true() {
  case "${1,,}" in
    1|true|yes|y|on) return 0 ;;
    *) return 1 ;;
  esac
}

if bool_is_true "${TRAJ_PLANNER_ENABLE}" && [[ "${CMD_TOPIC}" == "/position_cmd" ]]; then
  CMD_TOPIC="${TRAJ_PLANNER_INPUT_TOPIC}"
fi
if bool_is_true "${TRAJ_PLANNER_ENABLE}"; then
  if [[ -z "${TARGET_OFFSET_X_WAS_SET}" ]]; then
    TARGET_OFFSET_X="0.0"
  fi
  if [[ -z "${TARGET_GRASP_X_BIAS_WAS_SET}" ]]; then
    TARGET_GRASP_X_BIAS_M="0.0"
  fi
  if [[ -z "${TARGET_PREHOVER_MAP_X_BIAS_WAS_SET}" ]]; then
    TARGET_PREHOVER_MAP_X_BIAS_M="-0.15"
  fi
  if [[ -z "${TARGET_HOVER_MAP_X_BIAS_WAS_SET}" ]]; then
    TARGET_HOVER_MAP_X_BIAS_M="-0.01"
  fi
  if [[ -z "${TARGET_GRASP_MAP_X_BIAS_WAS_SET}" ]]; then
    TARGET_GRASP_MAP_X_BIAS_M="-0.01"
  fi
fi

ros_double() {
  case "$1" in
    *.*|*e*|*E*) printf '%s' "$1" ;;
    *) printf '%s.0' "$1" ;;
  esac
}

cleanup() {
  if [[ -n "${gripper_manager_pid}" ]] && kill -0 "${gripper_manager_pid}" 2>/dev/null; then
    echo "[auto-hls-grasp-place] stopping gripper manager ${gripper_manager_pid}"
    kill -TERM -- "-${gripper_manager_pid}" 2>/dev/null || true
    for _ in {1..20}; do
      if ! kill -0 "${gripper_manager_pid}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    if kill -0 "${gripper_manager_pid}" 2>/dev/null; then
      kill -KILL -- "-${gripper_manager_pid}" 2>/dev/null || true
    fi
    wait "${gripper_manager_pid}" 2>/dev/null || true
  fi
}

on_signal() {
  cleanup
  exit 130
}

run_pose_preflight_pass() {
  echo "[auto-hls-grasp-place] checking required pose topics in parallel"
  PYTHONNOUSERSITE=1 "${PREFLIGHT_PYTHON}" "${PREFLIGHT_CHECKER}" \
    --timeout "${POSE_PREFLIGHT_TIMEOUT_S}" \
    --log-prefix "[auto-hls-grasp-place]" \
    --topic "control drone pose:${DRONE_POSE_TOPIC}:nav_msgs/msg/Odometry" \
    --topic "arrival check pose:${ARRIVAL_POSE_TOPIC}:geometry_msgs/msg/PoseStamped" \
    --topic "target pose:${TARGET_POSE_TOPIC}:geometry_msgs/msg/PoseStamped" \
    --topic "box pose:${BOX_POSE_TOPIC}:geometry_msgs/msg/PoseStamped"
}

wait_for_gripper_manager_ready() {
  local status_prefix="${HLS_STATUS_TOPIC%/status}"
  if [[ "${status_prefix}" == "${HLS_STATUS_TOPIC}" ]]; then
    status_prefix="${HLS_STATUS_TOPIC%/}"
  fi
  local snapshot_topic="${status_prefix}/status_snapshot"
  local check_log
  check_log="$(mktemp /tmp/auto_hls_status_check.XXXXXX)"

  echo "[auto-hls-grasp-place] waiting for HLS status snapshot: ${snapshot_topic}"
  set +e
  PYTHONNOUSERSITE=1 "${PREFLIGHT_PYTHON}" "${PREFLIGHT_CHECKER}" \
    --timeout "${HLS_MANAGER_STARTUP_TIMEOUT_S}" \
    --log-prefix "[auto-hls-grasp-place]" \
    --topic "HLS gripper status snapshot:${snapshot_topic}:std_msgs/msg/String" \
    >"${check_log}" 2>&1 &
  local checker_pid="$!"

  while kill -0 "${checker_pid}" 2>/dev/null; do
    if ! kill -0 "${gripper_manager_pid}" 2>/dev/null; then
      wait "${gripper_manager_pid}" 2>/dev/null
      local manager_status="$?"
      kill "${checker_pid}" 2>/dev/null || true
      wait "${checker_pid}" 2>/dev/null || true
      cat "${check_log}" >&2
      rm -f "${check_log}"
      set -e
      echo "[auto-hls-grasp-place] ERROR: HLS gripper manager exited before status became live (status=${manager_status})." >&2
      echo "[auto-hls-grasp-place] Check port=${GRIPPER_MANAGER_PORT}, servo power, servo IDs 1/2, cable, and whether another process is using the bus." >&2
      return 1
    fi
    sleep 0.1
  done

  wait "${checker_pid}"
  local checker_status="$?"
  cat "${check_log}"
  rm -f "${check_log}"
  set -e

  if [[ "${checker_status}" -ne 0 ]]; then
    echo "[auto-hls-grasp-place] ERROR: no fresh HLS gripper status before automatic flight." >&2
    echo "[auto-hls-grasp-place] Check port=${GRIPPER_MANAGER_PORT}, servo power, servo IDs 1/2, cable, and whether another process is using the bus." >&2
    return 1
  fi

  echo "[auto-hls-grasp-place] HLS gripper status is live."
  return 0
}

trap cleanup EXIT
trap on_signal INT TERM HUP

echo "[auto-hls-grasp-place] target topic: ${TARGET_POSE_TOPIC}"
echo "[auto-hls-grasp-place] box topic: ${BOX_POSE_TOPIC}"
echo "[auto-hls-grasp-place] control drone pose topic: ${DRONE_POSE_TOPIC}"
echo "[auto-hls-grasp-place] arrival check pose topic: ${ARRIVAL_POSE_TOPIC}"
echo "[auto-hls-grasp-place] cmd topic: ${CMD_TOPIC}"
echo "[auto-hls-grasp-place] rc safety: topic=${RC_TOPIC} timeout=${RC_TIMEOUT_S}s stale_action=${RC_STALE_ACTION} ch10_index=${CH10_INDEX} open<=${CH10_OPEN_PWM} close>=${CH10_CLOSE_PWM} force_open_below_z=${FORCE_OPEN_BELOW_Z} low_z_grace=${FORCE_OPEN_BELOW_Z_GRACE_S}s mavros_state=${MAVROS_STATE_TOPIC}"
echo "[auto-hls-grasp-place] gripper topics: scalar=${GRIPPER_TOPIC} pair=${GRIPPER_COMMAND_PAIR_TOPIC} feedback=${GRIPPER_FEEDBACK_TOPIC} status=${HLS_STATUS_TOPIC}"
echo "[auto-hls-grasp-place] manager: type=${GRIPPER_MANAGER_TYPE} start=${START_GRIPPER_MANAGER} port=${GRIPPER_MANAGER_PORT} dry_run=${HLS_DRY_RUN}"
echo "[auto-hls-grasp-place] hls manager startup timeout: ${HLS_MANAGER_STARTUP_TIMEOUT_S}s"
echo "[auto-hls-grasp-place] hls sdk root: ${HLS_SDK_ROOT:-<auto>}"
echo "[auto-hls-grasp-place] hls open profile: speed=${HLS_OPEN_SPEED} acc=${HLS_OPEN_ACC} torque=${HLS_OPEN_TORQUE_LIMIT}"
echo "[auto-hls-grasp-place] hls current: low=${HLS_LOW_CURRENT} lift=${HLS_LIFT_CURRENT} max=${HLS_MAX_CURRENT} center_hold=${HLS_CENTER_HOLD_CURRENT:-<low>} center_push=${HLS_CENTER_PUSH_CURRENT:-<auto>} chase_min=${HLS_GRIP_CHASE_MIN_CURRENT:-<center_push>} center_timeout=${HLS_CENTER_TIMEOUT_ACTION}/${HLS_CENTER_TIMEOUT_S}s center_stable=${HLS_CENTER_STABLE_TIME_S}s final_grip_ramp=${HLS_FINAL_GRIP_RAMP_S}s"
echo "[auto-hls-grasp-place] hls chase position: enable=${HLS_GRIP_CHASE_POSITION_ENABLE} speed=${HLS_GRIP_CHASE_POSITION_SPEED} acc=${HLS_GRIP_CHASE_POSITION_ACC} torque=${HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT} slip=${HLS_GRIP_CHASE_SLIP_RATIO} period=${HLS_GRIP_CHASE_POSITION_PERIOD_S}s pulse=${HLS_GRIP_CHASE_POSITION_PULSE_S}s"
echo "[auto-hls-grasp-place] hls temperature: warn=${HLS_TEMP_WARN_THRESHOLD}C fault=${HLS_MAX_TEMP}C"
echo "[auto-hls-grasp-place] hls current signs: left=${HLS_LEFT_CURRENT_INWARD_SIGN:-<contact-metric>} right=${HLS_RIGHT_CURRENT_INWARD_SIGN:-<contact-metric>}"
echo "[auto-hls-grasp-place] hls motion profile: name=${HLS_MOTION_PROFILE:-<json-default>} index=${HLS_MOTION_PROFILE_INDEX:-<none>}"
echo "[auto-hls-grasp-place] hls search override: speed=${HLS_SEARCH_SPEED:-<profile>} acc=${HLS_SEARCH_ACC:-<profile>} torque=${HLS_SEARCH_TORQUE_LIMIT:-<profile>}"
echo "[auto-hls-grasp-place] hls contact: search_enter=${HLS_SEARCH_CONTACT_CURRENT_THRESHOLD} search_strong=${HLS_SEARCH_CONTACT_STRONG_THRESHOLD} search_confirm=${HLS_SEARCH_CONTACT_CONFIRM_CYCLES} search_min_close_ratio=${HLS_SEARCH_CONTACT_MIN_CLOSE_RATIO} hold_enter=${HLS_CONTACT_CURRENT_THRESHOLD} exit=${HLS_CONTACT_EXIT_THRESHOLD} hold_strong=${HLS_CONTACT_STRONG_THRESHOLD} hold_confirm=${HLS_CONTACT_CONFIRM_CYCLES}"
echo "[auto-hls-grasp-place] centering: kp=${CENTER_KP} vmax=${CENTER_VMAX_MPS} deadband=${CENTER_DEADBAND_M} offset_max=${CENTER_OFFSET_MAX_M} limit_grace=${HLS_OFFSET_LIMIT_GRACE_S}s sign=${CENTER_COMMAND_SIGN} hls_deadband=${HLS_CENTER_DEADBAND_M} hls_error_gain=${HLS_CENTER_ERROR_GAIN} snapshot_mismatch_tol=${HLS_STATUS_CENTER_MISMATCH_TOL_M}"
echo "[auto-hls-grasp-place] single-contact: vmax=${SINGLE_CONTACT_VMAX_MPS} offset_max=${SINGLE_CONTACT_OFFSET_MAX_M} sign=${SINGLE_CONTACT_BODY_Y_SIGN} timeout=${HLS_SINGLE_CONTACT_TIMEOUT_S} limit_ratio=${HLS_SINGLE_CONTACT_LIMIT_RATIO} hls_offset_limit=${HLS_SINGLE_CONTACT_OFFSET_LIMIT_M}"
echo "[auto-hls-grasp-place] planning offsets: target=(${TARGET_OFFSET_X}, ${TARGET_OFFSET_Y}, ${TARGET_OFFSET_Z})m box=(${BOX_OFFSET_X}, ${BOX_OFFSET_Y}, ${BOX_OFFSET_Z})m"
echo "[auto-hls-grasp-place] grasp body bias: target_grasp_x_bias=${TARGET_GRASP_X_BIAS_M}m target_grasp_z_bias=${TARGET_GRASP_Z_BIAS_M}m"
echo "[auto-hls-grasp-place] optional z offsets: target_hover=${TARGET_HOVER_Z_OFFSET:-<auto>} target_grasp=${TARGET_GRASP_Z_OFFSET:-<auto>} target_grasp_bias=${TARGET_GRASP_Z_BIAS_M} box_hover=${BOX_HOVER_Z_OFFSET:-<auto>} box_place=${BOX_PLACE_Z_OFFSET:-<auto>} transfer_z=${TRANSFER_DRONE_Z_M:-<auto>}"
echo "[auto-hls-grasp-place] trajectory planner: enable=${TRAJ_PLANNER_ENABLE} cmd_topic=${CMD_TOPIC} raw_topic=${TRAJ_PLANNER_INPUT_TOPIC}"
echo "[auto-hls-grasp-place] target map-X bias: prehover=${TARGET_PREHOVER_MAP_X_BIAS_M}m hover=${TARGET_HOVER_MAP_X_BIAS_M}m grasp=${TARGET_GRASP_MAP_X_BIAS_M}m"
echo "[auto-hls-grasp-place] motion speed: max=${MAX_SPEED} approach=${APPROACH_SPEED} lift=${LIFT_SPEED} payload_lift=${PAYLOAD_LIFT_SPEED} payload_transfer=${PAYLOAD_TRANSFER_SPEED} retreat=${RETREAT_SPEED} smooth=${SMOOTH_TRAJECTORY}"
echo "[auto-hls-grasp-place] release retreat: up=${RELEASE_RETREAT_UP_M}m forward=${RELEASE_RETREAT_FORWARD_M}m frame=${RELEASE_RETREAT_FRAME}"
echo "[auto-hls-grasp-place] landing: mode=${LANDING_MODE} cmd_z=${CMD_LAND_Z:-<relative>} cmd_speed=${CMD_LAND_SPEED} command_stop_before_land=${COMMAND_STOP_BEFORE_LAND_S}s auto_land_timeout=${AUTO_LAND_TIMEOUT_S}s no_land=${NO_LAND}"
echo "[auto-hls-grasp-place] actual arrival gate: waypoint_xy=${WAYPOINT_ARRIVAL_XY_TOLERANCE_M}m waypoint_z=${WAYPOINT_ARRIVAL_Z_TOLERANCE_M}m target_xy=${TARGET_ARRIVAL_XY_TOLERANCE_M}m target_z=${TARGET_ARRIVAL_Z_TOLERANCE_M}m target_max_positive_x=${TARGET_ARRIVAL_MAX_POSITIVE_X_ERROR_M}m pregrasp_xy=${PREGRASP_ARRIVAL_XY_TOLERANCE_M}m pregrasp_z=${PREGRASP_ARRIVAL_Z_TOLERANCE_M}m box_xy=${BOX_ARRIVAL_XY_TOLERANCE_M}m box_z=${BOX_ARRIVAL_Z_TOLERANCE_M}m settle_default=${WAYPOINT_ARRIVAL_SETTLE_S}s target_settle=${TARGET_ARRIVAL_SETTLE_S}s pregrasp_settle=${PREGRASP_ARRIVAL_SETTLE_S}s box_settle=${BOX_ARRIVAL_SETTLE_S}s timeout=${WAYPOINT_ARRIVAL_TIMEOUT_S}s pregrasp_z_speed=${PREGRASP_Z_SPEED_MPS}m/s"
echo "[auto-hls-grasp-place] release timing: release_at_box_hover=${RELEASE_AT_BOX_HOVER} pre_grasp_hold=${PRE_GRASP_HOLD_S}s post_grasp=${POST_GRASP_SETTLE_S}s post_lift=${POST_LIFT_SETTLE_S}s"
echo "[auto-hls-grasp-place] mocap correction: enable=${MOCAP_CORRECTION_ENABLE} max_xy=${MOCAP_CORRECTION_MAX_XY_M}m max_z=${MOCAP_CORRECTION_MAX_Z_M}m vxy=${MOCAP_CORRECTION_VXY_MPS}m/s vz=${MOCAP_CORRECTION_VZ_MPS}m/s hls_xy_vmax=${MOCAP_CORRECTION_HLS_XY_VMAX_MPS}m/s freeze_z_on_contact=${MOCAP_CORRECTION_FREEZE_Z_ON_CONTACT} body_frame=${MOCAP_CORRECTION_BODY_FRAME} yaw_source=${BODY_FRAME_YAW_SOURCE} yaw_offset_rad=${BODY_FRAME_YAW_OFFSET_RAD}"
echo "[auto-hls-grasp-place] target yaw align: enable=${TARGET_YAW_ALIGN_ENABLE} offset=${TARGET_YAW_ALIGN_OFFSET_RAD}rad rate=${TARGET_YAW_ALIGN_RATE_DPS}deg/s tol=${TARGET_YAW_ALIGN_TOL_DEG}deg min_distance=${TARGET_YAW_ALIGN_MIN_DISTANCE_M}m max_duration=${TARGET_YAW_ALIGN_MAX_DURATION_S}s"
echo "[auto-hls-grasp-place] hls retry: max_retries=${HLS_GRASP_MAX_RETRIES}"
echo "[auto-hls-grasp-place] takeoff mode: ${TAKEOFF_MODE}"
echo "[auto-hls-grasp-place] takeoff confirmation: ${CONFIRM_BEFORE_TAKEOFF}"
echo "[auto-hls-grasp-place] record gate: enable=${ENABLE_RECORD} status=${RECORD_STATUS_TOPIC} gate=${RECORD_GATE_TOPIC} value=${RECORD_GATE_VALUE} stop=${RECORD_STOP_TOPIC} stop_value=${RECORD_STOP_VALUE} duration=${RECORD_DURATION_S}s post_release_hold_stop=${POST_RELEASE_HOLD_STOP_S}s"
echo "[auto-hls-grasp-place] preflight checker: ${PREFLIGHT_PYTHON} ${PREFLIGHT_CHECKER}"

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

if ! bool_is_true "${SKIP_POSE_PREFLIGHT}"; then
  preflight_ok=false
  for ((attempt = 1; attempt <= POSE_PREFLIGHT_RETRIES; attempt++)); do
    echo "[auto-hls-grasp-place] preflight pose check attempt ${attempt}/${POSE_PREFLIGHT_RETRIES}"
    if run_pose_preflight_pass; then
      preflight_ok=true
      break
    fi
    if (( attempt < POSE_PREFLIGHT_RETRIES )); then
      echo "[auto-hls-grasp-place] pose preflight did not see every required topic; retrying after 1s." >&2
      sleep 1
    fi
  done
  if [[ "${preflight_ok}" != "true" ]] && bool_is_true "${POSE_PREFLIGHT_REQUIRED}"; then
    exit 1
  fi
fi

if bool_is_true "${START_GRIPPER_MANAGER}"; then
  if [[ "${GRIPPER_MANAGER_TYPE}" != "hls" ]]; then
    echo "[auto-hls-grasp-place] ERROR: this wrapper requires GRIPPER_MANAGER_TYPE=hls or START_GRIPPER_MANAGER=false." >&2
    exit 1
  fi
  hls_args=(
    --ros-args
    -p port:="${GRIPPER_MANAGER_PORT}"
    -p command_topic:="${GRIPPER_TOPIC}"
    -p command_pair_topic:="${GRIPPER_COMMAND_PAIR_TOPIC}"
    -p feedback_topic:="${GRIPPER_FEEDBACK_TOPIC}"
    -p status_topic:="${HLS_STATUS_TOPIC}"
    -p attitude_topic:="${ATTITUDE_TOPIC}"
    -p dry_run:="${HLS_DRY_RUN}"
    -p open_speed:="${HLS_OPEN_SPEED}"
    -p open_acc:="${HLS_OPEN_ACC}"
    -p open_torque_limit:="${HLS_OPEN_TORQUE_LIMIT}"
    -p low_current:="${HLS_LOW_CURRENT}"
    -p lift_current:="${HLS_LIFT_CURRENT}"
    -p max_current:="$(ros_double "${HLS_MAX_CURRENT}")"
    -p grip_chase_position_enable:="${HLS_GRIP_CHASE_POSITION_ENABLE}"
    -p grip_chase_position_speed:="${HLS_GRIP_CHASE_POSITION_SPEED}"
    -p grip_chase_position_acc:="${HLS_GRIP_CHASE_POSITION_ACC}"
    -p grip_chase_position_torque_limit:="${HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT}"
    -p grip_chase_slip_ratio:="$(ros_double "${HLS_GRIP_CHASE_SLIP_RATIO}")"
    -p grip_chase_position_period_s:="$(ros_double "${HLS_GRIP_CHASE_POSITION_PERIOD_S}")"
    -p grip_chase_position_pulse_s:="$(ros_double "${HLS_GRIP_CHASE_POSITION_PULSE_S}")"
    -p max_temp:="$(ros_double "${HLS_MAX_TEMP}")"
    -p temp_warn_threshold:="$(ros_double "${HLS_TEMP_WARN_THRESHOLD}")"
    -p center_timeout_action:="${HLS_CENTER_TIMEOUT_ACTION}"
    -p center_timeout_s:="$(ros_double "${HLS_CENTER_TIMEOUT_S}")"
    -p center_stable_time_s:="$(ros_double "${HLS_CENTER_STABLE_TIME_S}")"
    -p final_grip_ramp_s:="$(ros_double "${HLS_FINAL_GRIP_RAMP_S}")"
    -p center_gain_m_per_ratio:="${HLS_CENTER_GAIN_M_PER_RATIO}"
    -p center_error_gain:="$(ros_double "${HLS_CENTER_ERROR_GAIN}")"
    -p center_deadband_m:="$(ros_double "${HLS_CENTER_DEADBAND_M}")"
    -p center_bias:="${HLS_CENTER_BIAS}"
    -p center_sign:="${HLS_CENTER_SIGN}"
    -p contact_current_threshold:="$(ros_double "${HLS_CONTACT_CURRENT_THRESHOLD}")"
    -p contact_exit_threshold:="$(ros_double "${HLS_CONTACT_EXIT_THRESHOLD}")"
    -p contact_strong_threshold:="$(ros_double "${HLS_CONTACT_STRONG_THRESHOLD}")"
    -p contact_confirm_cycles:="${HLS_CONTACT_CONFIRM_CYCLES}"
    -p search_contact_current_threshold:="$(ros_double "${HLS_SEARCH_CONTACT_CURRENT_THRESHOLD}")"
    -p search_contact_strong_threshold:="$(ros_double "${HLS_SEARCH_CONTACT_STRONG_THRESHOLD}")"
    -p search_contact_confirm_cycles:="${HLS_SEARCH_CONTACT_CONFIRM_CYCLES}"
    -p search_contact_min_close_ratio:="$(ros_double "${HLS_SEARCH_CONTACT_MIN_CLOSE_RATIO}")"
    -p single_contact_offset_limit_m:="$(ros_double "${HLS_SINGLE_CONTACT_OFFSET_LIMIT_M}")"
    -p single_contact_timeout_s:="${HLS_SINGLE_CONTACT_TIMEOUT_S}"
    -p single_contact_limit_ratio:="${HLS_SINGLE_CONTACT_LIMIT_RATIO}"
    -p dry_run_contact_pattern:="${HLS_DRY_RUN_CONTACT_PATTERN}"
  )
  if [[ -n "${HLS_GRAVITY_COMP_PATH}" ]]; then
    hls_args+=(-p gravity_comp_path:="${HLS_GRAVITY_COMP_PATH}")
  fi
  if [[ -n "${HLS_SDK_ROOT}" ]]; then
    hls_args+=(-p sdk_root:="${HLS_SDK_ROOT}")
  fi
  if [[ -n "${HLS_MOTION_PROFILE}" ]]; then
    hls_args+=(-p motion_profile:="${HLS_MOTION_PROFILE}")
  fi
  if [[ -n "${HLS_MOTION_PROFILE_INDEX}" ]]; then
    hls_args+=(-p motion_profile_index:="${HLS_MOTION_PROFILE_INDEX}")
  fi
  if [[ -n "${HLS_SEARCH_SPEED}" ]]; then
    hls_args+=(-p search_speed:="${HLS_SEARCH_SPEED}")
  fi
  if [[ -n "${HLS_SEARCH_ACC}" ]]; then
    hls_args+=(-p search_acc:="${HLS_SEARCH_ACC}")
  fi
  if [[ -n "${HLS_SEARCH_TORQUE_LIMIT}" ]]; then
    hls_args+=(-p search_torque_limit:="${HLS_SEARCH_TORQUE_LIMIT}")
  fi
  if [[ -n "${HLS_CENTER_HOLD_CURRENT}" ]]; then
    hls_args+=(-p center_hold_current:="${HLS_CENTER_HOLD_CURRENT}")
  fi
  if [[ -n "${HLS_CENTER_PUSH_CURRENT}" ]]; then
    hls_args+=(-p center_push_current:="${HLS_CENTER_PUSH_CURRENT}")
  fi
  if [[ -n "${HLS_GRIP_CHASE_MIN_CURRENT}" ]]; then
    hls_args+=(-p grip_chase_min_current:="${HLS_GRIP_CHASE_MIN_CURRENT}")
  fi
  if [[ -n "${HLS_LEFT_CURRENT_INWARD_SIGN}" ]]; then
    hls_args+=(-p left_current_inward_sign:="${HLS_LEFT_CURRENT_INWARD_SIGN}")
  fi
  if [[ -n "${HLS_RIGHT_CURRENT_INWARD_SIGN}" ]]; then
    hls_args+=(-p right_current_inward_sign:="${HLS_RIGHT_CURRENT_INWARD_SIGN}")
  fi
  if [[ -n "${HLS_LEFT_OPEN}" ]]; then
    hls_args+=(-p left_open:="${HLS_LEFT_OPEN}")
  fi
  if [[ -n "${HLS_LEFT_CLEAR}" ]]; then
    hls_args+=(-p left_clear:="${HLS_LEFT_CLEAR}")
  fi
  if [[ -n "${HLS_LEFT_CLOSE}" ]]; then
    hls_args+=(-p left_close:="${HLS_LEFT_CLOSE}")
  fi
  if [[ -n "${HLS_RIGHT_OPEN}" ]]; then
    hls_args+=(-p right_open:="${HLS_RIGHT_OPEN}")
  fi
  if [[ -n "${HLS_RIGHT_CLEAR}" ]]; then
    hls_args+=(-p right_clear:="${HLS_RIGHT_CLEAR}")
  fi
  if [[ -n "${HLS_RIGHT_CLOSE}" ]]; then
    hls_args+=(-p right_close:="${HLS_RIGHT_CLOSE}")
  fi

  setsid ros2 run hls_gripper hls_gripper_node.py "${hls_args[@]}" &
  gripper_manager_pid="$!"
  wait_for_gripper_manager_ready
fi

auto_args=(
  --drone-pose-topic "${DRONE_POSE_TOPIC}"
  --arrival-pose-topic "${ARRIVAL_POSE_TOPIC}"
  --target-pose-topic "${TARGET_POSE_TOPIC}"
  --box-pose-topic "${BOX_POSE_TOPIC}"
  --cmd-topic "${CMD_TOPIC}"
  --gripper-topic "${GRIPPER_TOPIC}"
  --gripper-command-pair-topic "${GRIPPER_COMMAND_PAIR_TOPIC}"
  --gripper-feedback-topic "${GRIPPER_FEEDBACK_TOPIC}"
  --takeoff-land-topic "${TAKEOFF_LAND_TOPIC}"
  --px4ctrl-state-topic "${PX4CTRL_STATE_TOPIC}"
  --rate-hz "${RATE_HZ}"
  --pose-timeout-s "${POSE_TIMEOUT_S}"
  --takeoff-mode "${TAKEOFF_MODE}"
  --max-speed "${MAX_SPEED}"
  --approach-speed "${APPROACH_SPEED}"
  --lift-speed "${LIFT_SPEED}"
  --payload-lift-speed "${PAYLOAD_LIFT_SPEED}"
  --payload-transfer-speed "${PAYLOAD_TRANSFER_SPEED}"
  --post-grasp-settle-s "${POST_GRASP_SETTLE_S}"
  --post-lift-settle-s "${POST_LIFT_SETTLE_S}"
  --payload-lift-forward-comp-m "${PAYLOAD_LIFT_FORWARD_COMP_M}"
  --payload-lift-comp-x "${PAYLOAD_LIFT_COMP_X}"
  --payload-lift-comp-y "${PAYLOAD_LIFT_COMP_Y}"
  --payload-lift-comp-z "${PAYLOAD_LIFT_COMP_Z}"
  --gripper-x-offset-m "${GRIPPER_X_OFFSET_M}"
  --gripper-y-offset-m "${GRIPPER_Y_OFFSET_M}"
  --gripper-z-offset-m "${GRIPPER_Z_OFFSET_M}"
  --target-height-m "${TARGET_HEIGHT_M}"
  --target-grasp-height-m "${TARGET_GRASP_HEIGHT_M}"
  --target-pose-z-reference "${TARGET_POSE_Z_REFERENCE}"
  --target-hover-clearance-m "${TARGET_HOVER_CLEARANCE_M}"
  --box-length-m "${BOX_LENGTH_M}"
  --box-width-m "${BOX_WIDTH_M}"
  --box-height-m "${BOX_HEIGHT_M}"
  --box-hover-gripper-clearance-m "${BOX_HOVER_GRIPPER_CLEARANCE_M}"
  --box-place-bottom-clearance-m "${BOX_PLACE_BOTTOM_CLEARANCE_M}"
  --target-offset-x "${TARGET_OFFSET_X}"
  --target-offset-y "${TARGET_OFFSET_Y}"
  --target-offset-z "${TARGET_OFFSET_Z}"
  --target-grasp-x-bias-m "${TARGET_GRASP_X_BIAS_M}"
  --target-prehover-map-x-bias-m "${TARGET_PREHOVER_MAP_X_BIAS_M}"
  --target-hover-map-x-bias-m "${TARGET_HOVER_MAP_X_BIAS_M}"
  --target-grasp-map-x-bias-m "${TARGET_GRASP_MAP_X_BIAS_M}"
  --target-grasp-z-bias-m "${TARGET_GRASP_Z_BIAS_M}"
  --box-offset-x "${BOX_OFFSET_X}"
  --box-offset-y "${BOX_OFFSET_Y}"
  --box-offset-z "${BOX_OFFSET_Z}"
  --release-retreat-up-m "${RELEASE_RETREAT_UP_M}"
  --release-retreat-forward-m "${RELEASE_RETREAT_FORWARD_M}"
  --release-retreat-frame "${RELEASE_RETREAT_FRAME}"
  --post-release-hold-stop-s "${POST_RELEASE_HOLD_STOP_S}"
  --retreat-speed "${RETREAT_SPEED}"
  --landing-mode "${LANDING_MODE}"
  --cmd-land-speed "${CMD_LAND_SPEED}"
  --cmd-land-z-offset-m "${CMD_LAND_Z_OFFSET_M}"
  --command-stop-before-land-s "${COMMAND_STOP_BEFORE_LAND_S}"
  --auto-land-timeout-s "${AUTO_LAND_TIMEOUT_S}"
  --post-takeoff-settle-s "${POST_TAKEOFF_SETTLE_S}"
  --waypoint-arrival-tolerance-m "${WAYPOINT_ARRIVAL_TOLERANCE_M}"
  --waypoint-arrival-xy-tolerance-m "${WAYPOINT_ARRIVAL_XY_TOLERANCE_M}"
  --waypoint-arrival-z-tolerance-m "${WAYPOINT_ARRIVAL_Z_TOLERANCE_M}"
  --target-arrival-xy-tolerance-m "${TARGET_ARRIVAL_XY_TOLERANCE_M}"
  --target-arrival-z-tolerance-m "${TARGET_ARRIVAL_Z_TOLERANCE_M}"
  --target-arrival-max-positive-x-error-m "${TARGET_ARRIVAL_MAX_POSITIVE_X_ERROR_M}"
  --box-arrival-xy-tolerance-m "${BOX_ARRIVAL_XY_TOLERANCE_M}"
  --box-arrival-z-tolerance-m "${BOX_ARRIVAL_Z_TOLERANCE_M}"
  --pregrasp-arrival-xy-tolerance-m "${PREGRASP_ARRIVAL_XY_TOLERANCE_M}"
  --pregrasp-arrival-z-tolerance-m "${PREGRASP_ARRIVAL_Z_TOLERANCE_M}"
  --pregrasp-z-speed-mps "${PREGRASP_Z_SPEED_MPS}"
  --waypoint-arrival-settle-s "${WAYPOINT_ARRIVAL_SETTLE_S}"
  --target-arrival-settle-s "${TARGET_ARRIVAL_SETTLE_S}"
  --pregrasp-arrival-settle-s "${PREGRASP_ARRIVAL_SETTLE_S}"
  --box-arrival-settle-s "${BOX_ARRIVAL_SETTLE_S}"
  --waypoint-arrival-timeout-s "${WAYPOINT_ARRIVAL_TIMEOUT_S}"
  --mocap-correction-max-xy-m "${MOCAP_CORRECTION_MAX_XY_M}"
  --mocap-correction-max-z-m "${MOCAP_CORRECTION_MAX_Z_M}"
  --mocap-correction-vxy-mps "${MOCAP_CORRECTION_VXY_MPS}"
  --mocap-correction-vz-mps "${MOCAP_CORRECTION_VZ_MPS}"
  --mocap-correction-hls-xy-vmax-mps "${MOCAP_CORRECTION_HLS_XY_VMAX_MPS}"
  --body-frame-yaw-source "${BODY_FRAME_YAW_SOURCE}"
  --body-frame-yaw-offset-rad "${BODY_FRAME_YAW_OFFSET_RAD}"
  --target-yaw-align-offset-rad "${TARGET_YAW_ALIGN_OFFSET_RAD}"
  --target-yaw-align-rate-dps "${TARGET_YAW_ALIGN_RATE_DPS}"
  --target-yaw-align-tol-deg "${TARGET_YAW_ALIGN_TOL_DEG}"
  --target-yaw-align-min-distance-m "${TARGET_YAW_ALIGN_MIN_DISTANCE_M}"
  --target-yaw-align-max-duration-s "${TARGET_YAW_ALIGN_MAX_DURATION_S}"
  --pre-grasp-hold-s "${PRE_GRASP_HOLD_S}"
  --hls-status-topic "${HLS_STATUS_TOPIC}"
  --hls-status-timeout-s "${HLS_STATUS_TIMEOUT_S}"
  --hls-grasp-timeout-s "${HLS_GRASP_TIMEOUT_S}"
  --hls-grasp-max-retries "${HLS_GRASP_MAX_RETRIES}"
  --mavros-state-topic "${MAVROS_STATE_TOPIC}"
  --rc-topic "${RC_TOPIC}"
  --rc-timeout-s "${RC_TIMEOUT_S}"
  --rc-stale-action "${RC_STALE_ACTION}"
  --ch10-index "${CH10_INDEX}"
  --ch10-open-pwm "${CH10_OPEN_PWM}"
  --ch10-close-pwm "${CH10_CLOSE_PWM}"
  --force-open-below-z "${FORCE_OPEN_BELOW_Z}"
  --force-open-below-z-grace-s "${FORCE_OPEN_BELOW_Z_GRACE_S}"
  --open-command "${OPEN_COMMAND}"
  --close-command "${CLOSE_COMMAND}"
  --center-deadband-m "${CENTER_DEADBAND_M}"
  --center-kp "${CENTER_KP}"
  --center-vmax-mps "${CENTER_VMAX_MPS}"
  --center-offset-max-m "${CENTER_OFFSET_MAX_M}"
  --offset-limit-grace-s "${HLS_OFFSET_LIMIT_GRACE_S}"
  --center-command-sign "${CENTER_COMMAND_SIGN}"
  --hls-status-center-mismatch-tol-m "${HLS_STATUS_CENTER_MISMATCH_TOL_M}"
  --single-contact-vmax-mps "${SINGLE_CONTACT_VMAX_MPS}"
  --single-contact-offset-max-m "${SINGLE_CONTACT_OFFSET_MAX_M}"
  --single-contact-body-y-sign "${SINGLE_CONTACT_BODY_Y_SIGN}"
  --abort-rise-m "${ABORT_RISE_M}"
  --abort-rise-speed "${ABORT_RISE_SPEED}"
)

if [[ -n "${CMD_LAND_Z}" ]]; then
  auto_args+=(--cmd-land-z "${CMD_LAND_Z}")
fi
if [[ -n "${TARGET_HOVER_Z_OFFSET}" ]]; then
  auto_args+=(--target-hover-z-offset "${TARGET_HOVER_Z_OFFSET}")
fi
if [[ -n "${TARGET_GRASP_Z_OFFSET}" ]]; then
  auto_args+=(--target-grasp-z-offset "${TARGET_GRASP_Z_OFFSET}")
fi
if [[ -n "${BOX_HOVER_Z_OFFSET}" ]]; then
  auto_args+=(--box-hover-z-offset "${BOX_HOVER_Z_OFFSET}")
fi
if [[ -n "${BOX_PLACE_Z_OFFSET}" ]]; then
  auto_args+=(--box-place-z-offset "${BOX_PLACE_Z_OFFSET}")
fi
if [[ -n "${TRANSFER_DRONE_Z_M}" ]]; then
  auto_args+=(--transfer-drone-z-m "${TRANSFER_DRONE_Z_M}")
fi
if bool_is_true "${SMOOTH_TRAJECTORY}"; then
  auto_args+=(--smooth-trajectory)
else
  auto_args+=(--no-smooth-trajectory)
fi
if bool_is_true "${TRAJ_PLANNER_ENABLE}"; then
  auto_args+=(--trajectory-planner-enable)
else
  auto_args+=(--no-trajectory-planner-enable)
fi
if bool_is_true "${MOCAP_CORRECTION_ENABLE}"; then
  auto_args+=(--mocap-correction-enable)
else
  auto_args+=(--no-mocap-correction-enable)
fi
if bool_is_true "${MOCAP_CORRECTION_FREEZE_Z_ON_CONTACT}"; then
  auto_args+=(--mocap-correction-freeze-z-on-contact)
else
  auto_args+=(--no-mocap-correction-freeze-z-on-contact)
fi
if bool_is_true "${MOCAP_CORRECTION_BODY_FRAME}"; then
  auto_args+=(--mocap-correction-body-frame)
else
  auto_args+=(--no-mocap-correction-body-frame)
fi
if bool_is_true "${TARGET_YAW_ALIGN_ENABLE}"; then
  auto_args+=(--target-yaw-align-enable)
else
  auto_args+=(--no-target-yaw-align-enable)
fi
if bool_is_true "${CONFIRM_BEFORE_TAKEOFF}"; then
  auto_args+=(--confirm-before-takeoff)
else
  auto_args+=(--no-confirm-before-takeoff)
fi
if bool_is_true "${RELEASE_AT_BOX_HOVER}"; then
  auto_args+=(--release-at-box-hover)
else
  auto_args+=(--no-release-at-box-hover)
fi
if bool_is_true "${ENABLE_RECORD}"; then
  auto_args+=(
    --enable-record
    --record-status-topic "${RECORD_STATUS_TOPIC}"
    --record-gate-topic "${RECORD_GATE_TOPIC}"
    --record-gate-value "${RECORD_GATE_VALUE}"
    --record-stop-topic "${RECORD_STOP_TOPIC}"
    --record-stop-value "${RECORD_STOP_VALUE}"
    --record-stop-hold-s "${RECORD_STOP_HOLD_S}"
    --record-ready-timeout-s "${RECORD_READY_TIMEOUT_S}"
    --record-duration-s "${RECORD_DURATION_S}"
    --record-start-hold-s "${RECORD_START_HOLD_S}"
  )
fi
if bool_is_true "${NO_LAND}"; then
  auto_args+=(--no-land)
fi

ros2 run px4ctrl auto_hls_grasp_place.py "${auto_args[@]}"
