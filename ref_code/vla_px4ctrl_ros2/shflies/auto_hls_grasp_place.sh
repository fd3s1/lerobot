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
GRIPPER_TOPIC="${GRIPPER_TOPIC:-/gripper/command}"
GRIPPER_COMMAND_PAIR_TOPIC="${GRIPPER_COMMAND_PAIR_TOPIC:-/gripper/command_pair}"
GRIPPER_FEEDBACK_TOPIC="${GRIPPER_FEEDBACK_TOPIC:-/gripper/feedback}"
HLS_STATUS_TOPIC="${HLS_STATUS_TOPIC:-/hls_gripper/status}"
ATTITUDE_TOPIC="${ATTITUDE_TOPIC:-/mavros/imu/data}"
TAKEOFF_LAND_TOPIC="${TAKEOFF_LAND_TOPIC:-/px4ctrl/takeoff_land}"
PX4CTRL_STATE_TOPIC="${PX4CTRL_STATE_TOPIC:-/px4ctrl/state}"
TAKEOFF_MODE="${TAKEOFF_MODE:-auto}"
RC_TOPIC="${RC_TOPIC:-/mavros/rc/in}"
RC_TIMEOUT_S="${RC_TIMEOUT_S:-0.5}"
RC_STALE_ACTION="${RC_STALE_ACTION:-warn}"
CH10_INDEX="${CH10_INDEX:-9}"
CH10_OPEN_PWM="${CH10_OPEN_PWM:-1300}"
CH10_CLOSE_PWM="${CH10_CLOSE_PWM:-1700}"
FORCE_OPEN_BELOW_Z="${FORCE_OPEN_BELOW_Z:-0.20}"

START_GRIPPER_MANAGER="${START_GRIPPER_MANAGER:-true}"
GRIPPER_MANAGER_TYPE="${GRIPPER_MANAGER_TYPE:-hls}"
GRIPPER_MANAGER_PORT="${GRIPPER_MANAGER_PORT:-/dev/ttyACM1}"
HLS_GRAVITY_COMP_PATH="${HLS_GRAVITY_COMP_PATH:-}"
HLS_SDK_ROOT="${HLS_SDK_ROOT:-}"
HLS_DRY_RUN="${HLS_DRY_RUN:-false}"
HLS_OPEN_SPEED="${HLS_OPEN_SPEED:-24}"
HLS_OPEN_ACC="${HLS_OPEN_ACC:-6}"
HLS_OPEN_TORQUE_LIMIT="${HLS_OPEN_TORQUE_LIMIT:-120}"
HLS_LOW_CURRENT="${HLS_LOW_CURRENT:-28}"
HLS_LIFT_CURRENT="${HLS_LIFT_CURRENT:-76}"
HLS_CENTER_HOLD_CURRENT="${HLS_CENTER_HOLD_CURRENT:-28}"
HLS_CENTER_PUSH_CURRENT="${HLS_CENTER_PUSH_CURRENT:-66}"
HLS_GRIP_CHASE_MIN_CURRENT="${HLS_GRIP_CHASE_MIN_CURRENT:-66}"
HLS_GRIP_CHASE_POSITION_ENABLE="${HLS_GRIP_CHASE_POSITION_ENABLE:-true}"
HLS_GRIP_CHASE_POSITION_SPEED="${HLS_GRIP_CHASE_POSITION_SPEED:-16}"
HLS_GRIP_CHASE_POSITION_ACC="${HLS_GRIP_CHASE_POSITION_ACC:-6}"
HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT="${HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT:-145}"
HLS_GRIP_CHASE_SLIP_RATIO="${HLS_GRIP_CHASE_SLIP_RATIO:-0.03}"
HLS_GRIP_CHASE_POSITION_PERIOD_S="${HLS_GRIP_CHASE_POSITION_PERIOD_S:-0.05}"
HLS_GRIP_CHASE_POSITION_PULSE_S="${HLS_GRIP_CHASE_POSITION_PULSE_S:-0.80}"
HLS_CENTER_TIMEOUT_ACTION="${HLS_CENTER_TIMEOUT_ACTION:-final_grip}"
HLS_MAX_TEMP="${HLS_MAX_TEMP:-85.0}"
HLS_TEMP_WARN_THRESHOLD="${HLS_TEMP_WARN_THRESHOLD:-75.0}"
HLS_LEFT_CURRENT_INWARD_SIGN="${HLS_LEFT_CURRENT_INWARD_SIGN:-}"
HLS_RIGHT_CURRENT_INWARD_SIGN="${HLS_RIGHT_CURRENT_INWARD_SIGN:-}"
HLS_MOTION_PROFILE="${HLS_MOTION_PROFILE:-}"
HLS_MOTION_PROFILE_INDEX="${HLS_MOTION_PROFILE_INDEX:-}"
HLS_SEARCH_SPEED="${HLS_SEARCH_SPEED:-9}"
HLS_SEARCH_ACC="${HLS_SEARCH_ACC:-}"
HLS_SEARCH_TORQUE_LIMIT="${HLS_SEARCH_TORQUE_LIMIT:-145}"
HLS_CENTER_GAIN_M_PER_RATIO="${HLS_CENTER_GAIN_M_PER_RATIO:-0.0}"
HLS_CENTER_ERROR_GAIN="${HLS_CENTER_ERROR_GAIN:-2.0}"
HLS_CENTER_BIAS="${HLS_CENTER_BIAS:-0.0}"
HLS_CENTER_SIGN="${HLS_CENTER_SIGN:-1.0}"
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

RATE_HZ="${RATE_HZ:-20}"
MAX_SPEED="${MAX_SPEED:-0.6}"
APPROACH_SPEED="${APPROACH_SPEED:-0.3}"
LIFT_SPEED="${LIFT_SPEED:-0.4}"
PAYLOAD_LIFT_SPEED="${PAYLOAD_LIFT_SPEED:-0.08}"
PAYLOAD_TRANSFER_SPEED="${PAYLOAD_TRANSFER_SPEED:-0.14}"
POST_GRASP_SETTLE_S="${POST_GRASP_SETTLE_S:-0.8}"
POST_LIFT_SETTLE_S="${POST_LIFT_SETTLE_S:-1.0}"
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
BOX_PLACE_BOTTOM_CLEARANCE_M="${BOX_PLACE_BOTTOM_CLEARANCE_M:-0.03}"
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
RELEASE_RETREAT_UP_M="${RELEASE_RETREAT_UP_M:-0.3}"
RELEASE_RETREAT_FORWARD_M="${RELEASE_RETREAT_FORWARD_M:-1.0}"
RETREAT_SPEED="${RETREAT_SPEED:-0.4}"
LANDING_MODE="${LANDING_MODE:-cmd}"
CMD_LAND_SPEED="${CMD_LAND_SPEED:-0.25}"
CMD_LAND_Z="${CMD_LAND_Z:--0.3}"
CMD_LAND_Z_OFFSET_M="${CMD_LAND_Z_OFFSET_M:-0.0}"
NO_LAND="${NO_LAND:-false}"

HLS_GRASP_TIMEOUT_S="${HLS_GRASP_TIMEOUT_S:-12.0}"
HLS_STATUS_TIMEOUT_S="${HLS_STATUS_TIMEOUT_S:-0.8}"
CENTER_DEADBAND_M="${CENTER_DEADBAND_M:-0.005}"
CENTER_KP="${CENTER_KP:-0.8}"
CENTER_VMAX_MPS="${CENTER_VMAX_MPS:-0.03}"
CENTER_OFFSET_MAX_M="${CENTER_OFFSET_MAX_M:-0.08}"
CENTER_COMMAND_SIGN="${CENTER_COMMAND_SIGN:-1.0}"
SINGLE_CONTACT_VMAX_MPS="${SINGLE_CONTACT_VMAX_MPS:-0.015}"
SINGLE_CONTACT_OFFSET_MAX_M="${SINGLE_CONTACT_OFFSET_MAX_M:-0.10}"
SINGLE_CONTACT_BODY_Y_SIGN="${SINGLE_CONTACT_BODY_Y_SIGN:-1.0}"
ABORT_RISE_M="${ABORT_RISE_M:-0.25}"
ABORT_RISE_SPEED="${ABORT_RISE_SPEED:-0.12}"
OPEN_COMMAND="${OPEN_COMMAND:-100.0}"
CLOSE_COMMAND="${CLOSE_COMMAND:-0.0}"

WAYPOINT_ARRIVAL_TOLERANCE_M="${WAYPOINT_ARRIVAL_TOLERANCE_M:-0.08}"
WAYPOINT_ARRIVAL_SETTLE_S="${WAYPOINT_ARRIVAL_SETTLE_S:-0.4}"
WAYPOINT_ARRIVAL_TIMEOUT_S="${WAYPOINT_ARRIVAL_TIMEOUT_S:-15.0}"
CONFIRM_BEFORE_TAKEOFF="${CONFIRM_BEFORE_TAKEOFF:-false}"
POSE_PREFLIGHT_TIMEOUT_S="${POSE_PREFLIGHT_TIMEOUT_S:-6}"
POSE_PREFLIGHT_REQUIRED="${POSE_PREFLIGHT_REQUIRED:-false}"
SKIP_POSE_PREFLIGHT="${SKIP_POSE_PREFLIGHT:-false}"

gripper_manager_pid=""

bool_is_true() {
  case "${1,,}" in
    1|true|yes|y|on) return 0 ;;
    *) return 1 ;;
  esac
}

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

check_pose_topic_once() {
  local label="$1"
  local topic="$2"

  echo "[auto-hls-grasp-place] checking ${label} pose topic: ${topic}"
  if timeout "${POSE_PREFLIGHT_TIMEOUT_S}s" \
    ros2 topic echo --once --qos-reliability best_effort "${topic}" >/dev/null 2>&1; then
    echo "[auto-hls-grasp-place] ${label} pose topic is live: ${topic}"
    return 0
  fi
  echo "[auto-hls-grasp-place] WARNING: no fresh ${label} pose on ${topic}" >&2
  return 1
}

trap cleanup EXIT
trap on_signal INT TERM HUP

echo "[auto-hls-grasp-place] target topic: ${TARGET_POSE_TOPIC}"
echo "[auto-hls-grasp-place] box topic: ${BOX_POSE_TOPIC}"
echo "[auto-hls-grasp-place] control drone pose topic: ${DRONE_POSE_TOPIC}"
echo "[auto-hls-grasp-place] arrival check pose topic: ${ARRIVAL_POSE_TOPIC}"
echo "[auto-hls-grasp-place] cmd topic: ${CMD_TOPIC}"
echo "[auto-hls-grasp-place] rc safety: topic=${RC_TOPIC} timeout=${RC_TIMEOUT_S}s stale_action=${RC_STALE_ACTION} ch10_index=${CH10_INDEX} open<=${CH10_OPEN_PWM} close>=${CH10_CLOSE_PWM} force_open_below_z=${FORCE_OPEN_BELOW_Z}"
echo "[auto-hls-grasp-place] gripper topics: scalar=${GRIPPER_TOPIC} pair=${GRIPPER_COMMAND_PAIR_TOPIC} feedback=${GRIPPER_FEEDBACK_TOPIC} status=${HLS_STATUS_TOPIC}"
echo "[auto-hls-grasp-place] manager: type=${GRIPPER_MANAGER_TYPE} start=${START_GRIPPER_MANAGER} port=${GRIPPER_MANAGER_PORT} dry_run=${HLS_DRY_RUN}"
echo "[auto-hls-grasp-place] hls sdk root: ${HLS_SDK_ROOT:-<auto>}"
echo "[auto-hls-grasp-place] hls open profile: speed=${HLS_OPEN_SPEED} acc=${HLS_OPEN_ACC} torque=${HLS_OPEN_TORQUE_LIMIT}"
echo "[auto-hls-grasp-place] hls current: low=${HLS_LOW_CURRENT} lift=${HLS_LIFT_CURRENT} center_hold=${HLS_CENTER_HOLD_CURRENT:-<low>} center_push=${HLS_CENTER_PUSH_CURRENT:-<auto>} chase_min=${HLS_GRIP_CHASE_MIN_CURRENT:-<center_push>} center_timeout=${HLS_CENTER_TIMEOUT_ACTION}"
echo "[auto-hls-grasp-place] hls chase position: enable=${HLS_GRIP_CHASE_POSITION_ENABLE} speed=${HLS_GRIP_CHASE_POSITION_SPEED} acc=${HLS_GRIP_CHASE_POSITION_ACC} torque=${HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT} slip=${HLS_GRIP_CHASE_SLIP_RATIO} period=${HLS_GRIP_CHASE_POSITION_PERIOD_S}s pulse=${HLS_GRIP_CHASE_POSITION_PULSE_S}s"
echo "[auto-hls-grasp-place] hls temperature: warn=${HLS_TEMP_WARN_THRESHOLD}C fault=${HLS_MAX_TEMP}C"
echo "[auto-hls-grasp-place] hls current signs: left=${HLS_LEFT_CURRENT_INWARD_SIGN:-<contact-metric>} right=${HLS_RIGHT_CURRENT_INWARD_SIGN:-<contact-metric>}"
echo "[auto-hls-grasp-place] hls motion profile: name=${HLS_MOTION_PROFILE:-<json-default>} index=${HLS_MOTION_PROFILE_INDEX:-<none>}"
echo "[auto-hls-grasp-place] hls search override: speed=${HLS_SEARCH_SPEED:-<profile>} acc=${HLS_SEARCH_ACC:-<profile>} torque=${HLS_SEARCH_TORQUE_LIMIT:-<profile>}"
echo "[auto-hls-grasp-place] centering: kp=${CENTER_KP} vmax=${CENTER_VMAX_MPS} deadband=${CENTER_DEADBAND_M} offset_max=${CENTER_OFFSET_MAX_M} sign=${CENTER_COMMAND_SIGN} hls_error_gain=${HLS_CENTER_ERROR_GAIN}"
echo "[auto-hls-grasp-place] single-contact: vmax=${SINGLE_CONTACT_VMAX_MPS} offset_max=${SINGLE_CONTACT_OFFSET_MAX_M} sign=${SINGLE_CONTACT_BODY_Y_SIGN} timeout=${HLS_SINGLE_CONTACT_TIMEOUT_S} limit_ratio=${HLS_SINGLE_CONTACT_LIMIT_RATIO} hls_offset_limit=${HLS_SINGLE_CONTACT_OFFSET_LIMIT_M}"
echo "[auto-hls-grasp-place] planning offsets: target=(${TARGET_OFFSET_X}, ${TARGET_OFFSET_Y}, ${TARGET_OFFSET_Z})m box=(${BOX_OFFSET_X}, ${BOX_OFFSET_Y}, ${BOX_OFFSET_Z})m"
echo "[auto-hls-grasp-place] optional z offsets: target_hover=${TARGET_HOVER_Z_OFFSET:-<auto>} target_grasp=${TARGET_GRASP_Z_OFFSET:-<auto>} box_hover=${BOX_HOVER_Z_OFFSET:-<auto>} box_place=${BOX_PLACE_Z_OFFSET:-<auto>}"
echo "[auto-hls-grasp-place] actual arrival gate: tol=${WAYPOINT_ARRIVAL_TOLERANCE_M}m settle=${WAYPOINT_ARRIVAL_SETTLE_S}s timeout=${WAYPOINT_ARRIVAL_TIMEOUT_S}s"
echo "[auto-hls-grasp-place] takeoff mode: ${TAKEOFF_MODE}"
echo "[auto-hls-grasp-place] takeoff confirmation: ${CONFIRM_BEFORE_TAKEOFF}"
echo "[auto-hls-grasp-place] no LeRobot record process will be started."

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

if ! bool_is_true "${SKIP_POSE_PREFLIGHT}"; then
  preflight_failed=false
  check_pose_topic_once "control drone" "${DRONE_POSE_TOPIC}" || preflight_failed=true
  check_pose_topic_once "arrival check drone" "${ARRIVAL_POSE_TOPIC}" || preflight_failed=true
  check_pose_topic_once "target" "${TARGET_POSE_TOPIC}" || preflight_failed=true
  check_pose_topic_once "box" "${BOX_POSE_TOPIC}" || preflight_failed=true
  if [[ "${preflight_failed}" == "true" ]] && bool_is_true "${POSE_PREFLIGHT_REQUIRED}"; then
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
    -p center_gain_m_per_ratio:="${HLS_CENTER_GAIN_M_PER_RATIO}"
    -p center_error_gain:="$(ros_double "${HLS_CENTER_ERROR_GAIN}")"
    -p center_bias:="${HLS_CENTER_BIAS}"
    -p center_sign:="${HLS_CENTER_SIGN}"
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
  sleep 1.0
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
  --box-offset-x "${BOX_OFFSET_X}"
  --box-offset-y "${BOX_OFFSET_Y}"
  --box-offset-z "${BOX_OFFSET_Z}"
  --release-retreat-up-m "${RELEASE_RETREAT_UP_M}"
  --release-retreat-forward-m "${RELEASE_RETREAT_FORWARD_M}"
  --retreat-speed "${RETREAT_SPEED}"
  --landing-mode "${LANDING_MODE}"
  --cmd-land-speed "${CMD_LAND_SPEED}"
  --cmd-land-z-offset-m "${CMD_LAND_Z_OFFSET_M}"
  --waypoint-arrival-tolerance-m "${WAYPOINT_ARRIVAL_TOLERANCE_M}"
  --waypoint-arrival-settle-s "${WAYPOINT_ARRIVAL_SETTLE_S}"
  --waypoint-arrival-timeout-s "${WAYPOINT_ARRIVAL_TIMEOUT_S}"
  --hls-status-topic "${HLS_STATUS_TOPIC}"
  --hls-status-timeout-s "${HLS_STATUS_TIMEOUT_S}"
  --hls-grasp-timeout-s "${HLS_GRASP_TIMEOUT_S}"
  --rc-topic "${RC_TOPIC}"
  --rc-timeout-s "${RC_TIMEOUT_S}"
  --rc-stale-action "${RC_STALE_ACTION}"
  --ch10-index "${CH10_INDEX}"
  --ch10-open-pwm "${CH10_OPEN_PWM}"
  --ch10-close-pwm "${CH10_CLOSE_PWM}"
  --force-open-below-z "${FORCE_OPEN_BELOW_Z}"
  --open-command "${OPEN_COMMAND}"
  --close-command "${CLOSE_COMMAND}"
  --center-deadband-m "${CENTER_DEADBAND_M}"
  --center-kp "${CENTER_KP}"
  --center-vmax-mps "${CENTER_VMAX_MPS}"
  --center-offset-max-m "${CENTER_OFFSET_MAX_M}"
  --center-command-sign "${CENTER_COMMAND_SIGN}"
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
if bool_is_true "${SMOOTH_TRAJECTORY}"; then
  auto_args+=(--smooth-trajectory)
else
  auto_args+=(--no-smooth-trajectory)
fi
if bool_is_true "${CONFIRM_BEFORE_TAKEOFF}"; then
  auto_args+=(--confirm-before-takeoff)
else
  auto_args+=(--no-confirm-before-takeoff)
fi
if bool_is_true "${NO_LAND}"; then
  auto_args+=(--no-land)
fi

ros2 run px4ctrl auto_hls_grasp_place.py "${auto_args[@]}"
