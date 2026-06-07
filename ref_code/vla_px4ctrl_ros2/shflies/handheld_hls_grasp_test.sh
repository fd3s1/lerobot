#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

RC_TOPIC="${RC_TOPIC:-/mavros/rc/in}"
HLS_STATUS_TOPIC="${HLS_STATUS_TOPIC:-/hls_gripper/status}"
HLS_COMMAND_TOPIC="${HLS_COMMAND_TOPIC:-/handheld_hls_grasp_test/command}"
HLS_COMMAND_PAIR_TOPIC="${HLS_COMMAND_PAIR_TOPIC:-/handheld_hls_grasp_test/ignore_pair}"
GRIPPER_FEEDBACK_TOPIC="${GRIPPER_FEEDBACK_TOPIC:-/gripper/feedback}"
ATTITUDE_TOPIC="${ATTITUDE_TOPIC:-/mavros/imu/data}"

START_GRIPPER_MANAGER="${START_GRIPPER_MANAGER:-true}"
GRIPPER_MANAGER_PORT="${GRIPPER_MANAGER_PORT:-/dev/ttyACM1}"
HLS_GRAVITY_COMP_PATH="${HLS_GRAVITY_COMP_PATH:-}"
HLS_SDK_ROOT="${HLS_SDK_ROOT:-}"
HLS_DRY_RUN="${HLS_DRY_RUN:-false}"
HLS_OPEN_SPEED="${HLS_OPEN_SPEED:-24}"
HLS_OPEN_ACC="${HLS_OPEN_ACC:-6}"
HLS_OPEN_TORQUE_LIMIT="${HLS_OPEN_TORQUE_LIMIT:-120}"
HLS_LOW_CURRENT="${HLS_LOW_CURRENT:-25}"
HLS_LIFT_CURRENT="${HLS_LIFT_CURRENT:-70}"
HLS_CENTER_HOLD_CURRENT="${HLS_CENTER_HOLD_CURRENT:-}"
HLS_CENTER_PUSH_CURRENT="${HLS_CENTER_PUSH_CURRENT:-}"
HLS_GRIP_CHASE_MIN_CURRENT="${HLS_GRIP_CHASE_MIN_CURRENT:-}"
HLS_GRIP_CHASE_POSITION_ENABLE="${HLS_GRIP_CHASE_POSITION_ENABLE:-true}"
HLS_GRIP_CHASE_POSITION_SPEED="${HLS_GRIP_CHASE_POSITION_SPEED:-16}"
HLS_GRIP_CHASE_POSITION_ACC="${HLS_GRIP_CHASE_POSITION_ACC:-6}"
HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT="${HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT:-120}"
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
HLS_SEARCH_SPEED="${HLS_SEARCH_SPEED:-}"
HLS_SEARCH_ACC="${HLS_SEARCH_ACC:-}"
HLS_SEARCH_TORQUE_LIMIT="${HLS_SEARCH_TORQUE_LIMIT:-}"
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
RC_TIMEOUT_S="${RC_TIMEOUT_S:-0.8}"
STATUS_TIMEOUT_S="${STATUS_TIMEOUT_S:-2.0}"
CH10_INDEX="${CH10_INDEX:-9}"
CH10_OPEN_PWM="${CH10_OPEN_PWM:-1300}"
CH10_CLOSE_PWM="${CH10_CLOSE_PWM:-1700}"
OPEN_COMMAND="${OPEN_COMMAND:-100.0}"
CLOSE_COMMAND="${CLOSE_COMMAND:-0.0}"
PUBLISH_PERIOD_S="${PUBLISH_PERIOD_S:-0.5}"
GRASP_MODE_STABLE_S="${GRASP_MODE_STABLE_S:-0.45}"
OPEN_MODE_STABLE_S="${OPEN_MODE_STABLE_S:-0.0}"
HOLD_MODE_TIMEOUT_S="${HOLD_MODE_TIMEOUT_S:-1.5}"
RC_STALE_MODE="${RC_STALE_MODE:-open}"
CSV_PATH="${CSV_PATH:-}"

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
    echo "[handheld-hls-grasp-test] stopping HLS gripper manager ${gripper_manager_pid}"
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

trap cleanup EXIT
trap on_signal INT TERM HUP

echo "[handheld-hls-grasp-test] rc topic: ${RC_TOPIC} ch10_index=${CH10_INDEX}"
echo "[handheld-hls-grasp-test] command topic: ${HLS_COMMAND_TOPIC}"
echo "[handheld-hls-grasp-test] status topic: ${HLS_STATUS_TOPIC}"
echo "[handheld-hls-grasp-test] feedback topic: ${GRIPPER_FEEDBACK_TOPIC}"
echo "[handheld-hls-grasp-test] manager: start=${START_GRIPPER_MANAGER} port=${GRIPPER_MANAGER_PORT} dry_run=${HLS_DRY_RUN}"
echo "[handheld-hls-grasp-test] sdk root: ${HLS_SDK_ROOT:-<auto>}"
echo "[handheld-hls-grasp-test] open profile: speed=${HLS_OPEN_SPEED} acc=${HLS_OPEN_ACC} torque=${HLS_OPEN_TORQUE_LIMIT}"
echo "[handheld-hls-grasp-test] current: low=${HLS_LOW_CURRENT} lift=${HLS_LIFT_CURRENT} center_hold=${HLS_CENTER_HOLD_CURRENT:-<low>} center_push=${HLS_CENTER_PUSH_CURRENT:-<auto>} chase_min=${HLS_GRIP_CHASE_MIN_CURRENT:-<center_push>} center_timeout=${HLS_CENTER_TIMEOUT_ACTION}"
echo "[handheld-hls-grasp-test] chase position: enable=${HLS_GRIP_CHASE_POSITION_ENABLE} speed=${HLS_GRIP_CHASE_POSITION_SPEED} acc=${HLS_GRIP_CHASE_POSITION_ACC} torque=${HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT} slip=${HLS_GRIP_CHASE_SLIP_RATIO} period=${HLS_GRIP_CHASE_POSITION_PERIOD_S}s pulse=${HLS_GRIP_CHASE_POSITION_PULSE_S}s"
echo "[handheld-hls-grasp-test] temperature: warn=${HLS_TEMP_WARN_THRESHOLD}C fault=${HLS_MAX_TEMP}C"
echo "[handheld-hls-grasp-test] current signs: left=${HLS_LEFT_CURRENT_INWARD_SIGN:-<contact-metric>} right=${HLS_RIGHT_CURRENT_INWARD_SIGN:-<contact-metric>}"
echo "[handheld-hls-grasp-test] motion profile: name=${HLS_MOTION_PROFILE:-<json-default>} index=${HLS_MOTION_PROFILE_INDEX:-<none>}"
echo "[handheld-hls-grasp-test] search override: speed=${HLS_SEARCH_SPEED:-<profile>} acc=${HLS_SEARCH_ACC:-<profile>} torque=${HLS_SEARCH_TORQUE_LIMIT:-<profile>}"
echo "[handheld-hls-grasp-test] center: gain=${HLS_CENTER_GAIN_M_PER_RATIO} error_gain=${HLS_CENTER_ERROR_GAIN} bias=${HLS_CENTER_BIAS} sign=${HLS_CENTER_SIGN}"
echo "[handheld-hls-grasp-test] single-contact: timeout=${HLS_SINGLE_CONTACT_TIMEOUT_S} limit_ratio=${HLS_SINGLE_CONTACT_LIMIT_RATIO} offset_limit=${HLS_SINGLE_CONTACT_OFFSET_LIMIT_M} dry_run_pattern=${HLS_DRY_RUN_CONTACT_PATTERN}"
echo "[handheld-hls-grasp-test] rc latch: stale=${RC_STALE_MODE} grasp_stable=${GRASP_MODE_STABLE_S}s open_stable=${OPEN_MODE_STABLE_S}s hold_timeout=${HOLD_MODE_TIMEOUT_S}s publish_period=${PUBLISH_PERIOD_S}s"
echo "[handheld-hls-grasp-test] no /position_cmd, takeoff/land, or record data will be published."

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

if bool_is_true "${START_GRIPPER_MANAGER}"; then
  hls_args=(
    --ros-args
    -p port:="${GRIPPER_MANAGER_PORT}"
    -p command_topic:="${HLS_COMMAND_TOPIC}"
    -p command_pair_topic:="${HLS_COMMAND_PAIR_TOPIC}"
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

test_args=(
  --rc-topic "${RC_TOPIC}"
  --status-topic "${HLS_STATUS_TOPIC}"
  --command-topic "${HLS_COMMAND_TOPIC}"
  --rate-hz "${RATE_HZ}"
  --rc-timeout-s "${RC_TIMEOUT_S}"
  --status-timeout-s "${STATUS_TIMEOUT_S}"
  --ch10-index "${CH10_INDEX}"
  --ch10-open-pwm "${CH10_OPEN_PWM}"
  --ch10-close-pwm "${CH10_CLOSE_PWM}"
  --open-command "${OPEN_COMMAND}"
  --close-command "${CLOSE_COMMAND}"
  --publish-period-s "${PUBLISH_PERIOD_S}"
  --grasp-mode-stable-s "${GRASP_MODE_STABLE_S}"
  --open-mode-stable-s "${OPEN_MODE_STABLE_S}"
  --hold-mode-timeout-s "${HOLD_MODE_TIMEOUT_S}"
  --rc-stale-mode "${RC_STALE_MODE}"
)
if [[ -n "${CSV_PATH}" ]]; then
  test_args+=(--csv-path "${CSV_PATH}")
fi

ros2 run px4ctrl handheld_hls_grasp_test.py "${test_args[@]}"
