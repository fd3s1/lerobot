#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

START_STACK="${START_STACK:-true}"
START_PX4CTRL="${START_PX4CTRL:-true}"
STACK_STARTUP_WAIT_S="${STACK_STARTUP_WAIT_S:-8}"
WAIT_FOR_ENTER="${WAIT_FOR_ENTER:-false}"
CONFIRM_BEFORE_TAKEOFF="${CONFIRM_BEFORE_TAKEOFF:-false}"
TAKEOFF_MODE="${TAKEOFF_MODE:-manual}"
KEEP_STACK_ON_INTERRUPT="${KEEP_STACK_ON_INTERRUPT:-false}"
CLEANUP_STACK_ON_EXIT="${CLEANUP_STACK_ON_EXIT:-true}"

VRPN_SOURCE_TOPIC="${VRPN_SOURCE_TOPIC:-/vla_drone1/pose}"
MAVROS_VISION_TOPIC="${MAVROS_VISION_TOPIC:-/mavros/vision_pose/pose}"
TARGET_POSE_TOPIC="${TARGET_POSE_TOPIC:-/strawberry_bear/pose}"
BOX_POSE_TOPIC="${BOX_POSE_TOPIC:-/box1/pose}"
DRONE_POSE_TOPIC="${DRONE_POSE_TOPIC:-/mavros/local_position/odom}"
ARRIVAL_POSE_TOPIC="${ARRIVAL_POSE_TOPIC:-/mavros/vision_pose/pose}"
RC_TOPIC="${RC_TOPIC:-/mavros/rc/in}"
PX4CTRL_STATE_TOPIC="${PX4CTRL_STATE_TOPIC:-/px4ctrl/state}"
MAVROS_STATE_TOPIC="${MAVROS_STATE_TOPIC:-/mavros/state}"
POSE_PREFLIGHT_TIMEOUT_S="${POSE_PREFLIGHT_TIMEOUT_S:-6}"
POSE_PREFLIGHT_REQUIRED="${POSE_PREFLIGHT_REQUIRED:-true}"
SKIP_POSE_PREFLIGHT="${SKIP_POSE_PREFLIGHT:-false}"

GRIPPER_MANAGER_PORT="${GRIPPER_MANAGER_PORT:-/dev/ttyACM1}"
HLS_GRAVITY_COMP_PATH="${HLS_GRAVITY_COMP_PATH:-${WORKSPACE_DIR}/src/hls_gripper/config/gravity_compensation.json}"
HLS_MOTION_PROFILE="${HLS_MOTION_PROFILE:-p3}"
HLS_SEARCH_SPEED="${HLS_SEARCH_SPEED:-10}"
HLS_SEARCH_ACC="${HLS_SEARCH_ACC:-4}"
HLS_SEARCH_TORQUE_LIMIT="${HLS_SEARCH_TORQUE_LIMIT:-145}"
HLS_LEFT_CURRENT_INWARD_SIGN="${HLS_LEFT_CURRENT_INWARD_SIGN:-1}"
HLS_RIGHT_CURRENT_INWARD_SIGN="${HLS_RIGHT_CURRENT_INWARD_SIGN:-1}"
HLS_LOW_CURRENT="${HLS_LOW_CURRENT:-28}"
HLS_CENTER_HOLD_CURRENT="${HLS_CENTER_HOLD_CURRENT:-28}"
HLS_CENTER_PUSH_CURRENT="${HLS_CENTER_PUSH_CURRENT:-66}"
HLS_GRIP_CHASE_MIN_CURRENT="${HLS_GRIP_CHASE_MIN_CURRENT:-66}"
HLS_LIFT_CURRENT="${HLS_LIFT_CURRENT:-76}"
HLS_GRIP_CHASE_POSITION_ENABLE="${HLS_GRIP_CHASE_POSITION_ENABLE:-true}"
HLS_GRIP_CHASE_POSITION_SPEED="${HLS_GRIP_CHASE_POSITION_SPEED:-16}"
HLS_GRIP_CHASE_POSITION_ACC="${HLS_GRIP_CHASE_POSITION_ACC:-6}"
HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT="${HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT:-145}"
HLS_GRIP_CHASE_SLIP_RATIO="${HLS_GRIP_CHASE_SLIP_RATIO:-0.03}"
HLS_GRIP_CHASE_POSITION_PERIOD_S="${HLS_GRIP_CHASE_POSITION_PERIOD_S:-0.05}"
HLS_GRIP_CHASE_POSITION_PULSE_S="${HLS_GRIP_CHASE_POSITION_PULSE_S:-0.80}"
HLS_CENTER_TIMEOUT_ACTION="${HLS_CENTER_TIMEOUT_ACTION:-final_grip}"
HLS_CENTER_ERROR_GAIN="${HLS_CENTER_ERROR_GAIN:-2.0}"
HLS_SINGLE_CONTACT_OFFSET_LIMIT_M="${HLS_SINGLE_CONTACT_OFFSET_LIMIT_M:-0.12}"

RC_TIMEOUT_S="${RC_TIMEOUT_S:-0.5}"
RC_STALE_ACTION="${RC_STALE_ACTION:-warn}"
CH10_INDEX="${CH10_INDEX:-9}"
CH10_OPEN_PWM="${CH10_OPEN_PWM:-1300}"
CH10_CLOSE_PWM="${CH10_CLOSE_PWM:-1700}"
FORCE_OPEN_BELOW_Z="${FORCE_OPEN_BELOW_Z:-0.20}"

TARGET_OFFSET_X="${TARGET_OFFSET_X:-0.0}"
TARGET_OFFSET_Y="${TARGET_OFFSET_Y:-0.0}"
TARGET_OFFSET_Z="${TARGET_OFFSET_Z:-0.0}"
BOX_OFFSET_X="${BOX_OFFSET_X:-0.0}"
BOX_OFFSET_Y="${BOX_OFFSET_Y:-0.0}"
BOX_OFFSET_Z="${BOX_OFFSET_Z:-0.0}"
TARGET_HOVER_Z_OFFSET="${TARGET_HOVER_Z_OFFSET:-}"
TARGET_GRASP_Z_OFFSET="${TARGET_GRASP_Z_OFFSET:-}"
BOX_HOVER_Z_OFFSET="${BOX_HOVER_Z_OFFSET:-}"
BOX_PLACE_Z_OFFSET="${BOX_PLACE_Z_OFFSET:-}"

WAYPOINT_ARRIVAL_TOLERANCE_M="${WAYPOINT_ARRIVAL_TOLERANCE_M:-0.08}"
WAYPOINT_ARRIVAL_SETTLE_S="${WAYPOINT_ARRIVAL_SETTLE_S:-0.4}"
WAYPOINT_ARRIVAL_TIMEOUT_S="${WAYPOINT_ARRIVAL_TIMEOUT_S:-15.0}"
LANDING_MODE="${LANDING_MODE:-cmd}"
CMD_LAND_Z="${CMD_LAND_Z:--0.3}"
CMD_LAND_SPEED="${CMD_LAND_SPEED:-0.25}"
NO_LAND="${NO_LAND:-false}"

STACK_PID=""
AUTO_STATUS=0

bool_is_true() {
  case "${1,,}" in
    1|true|yes|y|on) return 0 ;;
    *) return 1 ;;
  esac
}

cleanup_stack() {
  if [[ -z "${STACK_PID}" ]]; then
    return
  fi
  if bool_is_true "${CLEANUP_STACK_ON_EXIT}"; then
    echo "[auto-hls-ude-test] stopping mocap/MAVROS/bridge/px4ctrl stack"
    kill -TERM -- "-${STACK_PID}" 2>/dev/null || kill -TERM "${STACK_PID}" 2>/dev/null || true
    sleep 1
    kill -KILL -- "-${STACK_PID}" 2>/dev/null || kill -KILL "${STACK_PID}" 2>/dev/null || true
  else
    echo "[auto-hls-ude-test] leaving stack running. Stop it later with: kill -TERM -- -${STACK_PID}"
  fi
}

on_interrupt() {
  echo
  if bool_is_true "${KEEP_STACK_ON_INTERRUPT}"; then
    echo "[auto-hls-ude-test] interrupted; leaving stack running because KEEP_STACK_ON_INTERRUPT=true"
    CLEANUP_STACK_ON_EXIT="false"
  else
    echo "[auto-hls-ude-test] interrupted; stopping stack"
    CLEANUP_STACK_ON_EXIT="true"
  fi
  AUTO_STATUS=130
  exit 130
}

check_topic_once() {
  local label="$1"
  local topic="$2"
  local qos_args=("${@:3}")

  echo "[auto-hls-ude-test] checking ${label}: ${topic}"
  if timeout "${POSE_PREFLIGHT_TIMEOUT_S}s" \
    ros2 topic echo --once "${qos_args[@]}" "${topic}" >/dev/null 2>&1; then
    echo "[auto-hls-ude-test] ${label} is live: ${topic}"
    return 0
  fi
  echo "[auto-hls-ude-test] WARNING: no fresh ${label} on ${topic}" >&2
  return 1
}

trap on_interrupt INT TERM
trap cleanup_stack EXIT

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
  echo "[auto-hls-ude-test] ERROR: ${WORKSPACE_DIR}/install/setup.bash not found" >&2
  echo "[auto-hls-ude-test] build first with: colcon build" >&2
  exit 1
fi
set -u

echo "[auto-hls-ude-test] workspace: ${WORKSPACE_DIR}"
echo "[auto-hls-ude-test] drone vrpn: ${VRPN_SOURCE_TOPIC} -> ${MAVROS_VISION_TOPIC}"
echo "[auto-hls-ude-test] control drone pose topic: ${DRONE_POSE_TOPIC}"
echo "[auto-hls-ude-test] arrival check pose topic: ${ARRIVAL_POSE_TOPIC}"
echo "[auto-hls-ude-test] target topic: ${TARGET_POSE_TOPIC}"
echo "[auto-hls-ude-test] box topic: ${BOX_POSE_TOPIC}"
echo "[auto-hls-ude-test] landing: mode=${LANDING_MODE} cmd_z=${CMD_LAND_Z} no_land=${NO_LAND}"
echo "[auto-hls-ude-test] CH10 safety: topic=${RC_TOPIC} timeout=${RC_TIMEOUT_S}s stale_action=${RC_STALE_ACTION} index=${CH10_INDEX} open<=${CH10_OPEN_PWM}"
echo "[auto-hls-ude-test] offsets: target=(${TARGET_OFFSET_X}, ${TARGET_OFFSET_Y}, ${TARGET_OFFSET_Z}) box=(${BOX_OFFSET_X}, ${BOX_OFFSET_Y}, ${BOX_OFFSET_Z})"
echo "[auto-hls-ude-test] takeoff: mode=${TAKEOFF_MODE} inner_confirm=${CONFIRM_BEFORE_TAKEOFF} outer_wait=${WAIT_FOR_ENTER}"

if bool_is_true "${START_STACK}"; then
  echo "[auto-hls-ude-test] starting mocap/MAVROS/bridge/px4ctrl stack"
  export START_PX4CTRL
  export VRPN_SOURCE_TOPIC
  export MAVROS_VISION_TOPIC
  setsid bash "${SCRIPT_DIR}/run_mocap_mavros.sh" &
  STACK_PID="$!"
  echo "[auto-hls-ude-test] stack process group: ${STACK_PID}"
  sleep "${STACK_STARTUP_WAIT_S}"
else
  echo "[auto-hls-ude-test] START_STACK=false; using already-running stack"
fi

if ! bool_is_true "${SKIP_POSE_PREFLIGHT}"; then
  preflight_failed=false
  check_topic_once "control drone pose" "${DRONE_POSE_TOPIC}" --qos-reliability best_effort || preflight_failed=true
  check_topic_once "arrival check pose" "${ARRIVAL_POSE_TOPIC}" --qos-reliability best_effort || preflight_failed=true
  check_topic_once "target pose" "${TARGET_POSE_TOPIC}" --qos-reliability best_effort || preflight_failed=true
  check_topic_once "box pose" "${BOX_POSE_TOPIC}" --qos-reliability best_effort || preflight_failed=true
  check_topic_once "RC input" "${RC_TOPIC}" --qos-reliability best_effort || preflight_failed=true
  check_topic_once "px4ctrl state" "${PX4CTRL_STATE_TOPIC}" || preflight_failed=true
  check_topic_once "MAVROS state" "${MAVROS_STATE_TOPIC}" || preflight_failed=true
  if [[ "${preflight_failed}" == "true" ]]; then
    echo "[auto-hls-ude-test] One-shot preflight did not see every required topic." >&2
    if bool_is_true "${POSE_PREFLIGHT_REQUIRED}"; then
      exit 1
    fi
  fi
fi

if bool_is_true "${WAIT_FOR_ENTER}"; then
  echo
  echo "[auto-hls-ude-test] Keep RC in hover+command mode, sticks centered, CH10 not low/open."
  echo "[auto-hls-ude-test] Press Enter to start preflight pose locking. A second inner confirmation may be used before TAKEOFF."
  read -r _
fi

export TARGET_POSE_TOPIC BOX_POSE_TOPIC DRONE_POSE_TOPIC ARRIVAL_POSE_TOPIC RC_TOPIC RC_TIMEOUT_S RC_STALE_ACTION
export CH10_INDEX CH10_OPEN_PWM CH10_CLOSE_PWM FORCE_OPEN_BELOW_Z
export GRIPPER_MANAGER_PORT HLS_GRAVITY_COMP_PATH HLS_MOTION_PROFILE
export HLS_SEARCH_SPEED HLS_SEARCH_ACC HLS_SEARCH_TORQUE_LIMIT
export HLS_LEFT_CURRENT_INWARD_SIGN HLS_RIGHT_CURRENT_INWARD_SIGN
export HLS_LOW_CURRENT HLS_CENTER_HOLD_CURRENT HLS_CENTER_PUSH_CURRENT HLS_GRIP_CHASE_MIN_CURRENT HLS_LIFT_CURRENT
export HLS_GRIP_CHASE_POSITION_ENABLE HLS_GRIP_CHASE_POSITION_SPEED HLS_GRIP_CHASE_POSITION_ACC
export HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT HLS_GRIP_CHASE_SLIP_RATIO HLS_GRIP_CHASE_POSITION_PERIOD_S
export HLS_GRIP_CHASE_POSITION_PULSE_S HLS_CENTER_TIMEOUT_ACTION HLS_CENTER_ERROR_GAIN
export HLS_SINGLE_CONTACT_OFFSET_LIMIT_M
export TARGET_OFFSET_X TARGET_OFFSET_Y TARGET_OFFSET_Z BOX_OFFSET_X BOX_OFFSET_Y BOX_OFFSET_Z
export TARGET_HOVER_Z_OFFSET TARGET_GRASP_Z_OFFSET BOX_HOVER_Z_OFFSET BOX_PLACE_Z_OFFSET
export WAYPOINT_ARRIVAL_TOLERANCE_M WAYPOINT_ARRIVAL_SETTLE_S WAYPOINT_ARRIVAL_TIMEOUT_S
export CONFIRM_BEFORE_TAKEOFF TAKEOFF_MODE
export LANDING_MODE CMD_LAND_Z CMD_LAND_SPEED NO_LAND POSE_PREFLIGHT_REQUIRED SKIP_POSE_PREFLIGHT POSE_PREFLIGHT_TIMEOUT_S

set +e
bash "${SCRIPT_DIR}/auto_hls_grasp_place.sh"
AUTO_STATUS="$?"
set -e

exit "${AUTO_STATUS}"
