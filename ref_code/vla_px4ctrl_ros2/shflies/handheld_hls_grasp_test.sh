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
HLS_DRY_RUN="${HLS_DRY_RUN:-false}"
HLS_LOW_CURRENT="${HLS_LOW_CURRENT:-40}"
HLS_LIFT_CURRENT="${HLS_LIFT_CURRENT:-120}"
HLS_MOTION_PROFILE="${HLS_MOTION_PROFILE:-}"
HLS_MOTION_PROFILE_INDEX="${HLS_MOTION_PROFILE_INDEX:-}"
HLS_CENTER_GAIN_M_PER_RATIO="${HLS_CENTER_GAIN_M_PER_RATIO:-0.0}"
HLS_CENTER_BIAS="${HLS_CENTER_BIAS:-0.0}"
HLS_CENTER_SIGN="${HLS_CENTER_SIGN:-1.0}"
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
CH10_INDEX="${CH10_INDEX:-9}"
CH10_OPEN_PWM="${CH10_OPEN_PWM:-1300}"
CH10_CLOSE_PWM="${CH10_CLOSE_PWM:-1700}"
OPEN_COMMAND="${OPEN_COMMAND:-100.0}"
CLOSE_COMMAND="${CLOSE_COMMAND:-0.0}"
CSV_PATH="${CSV_PATH:-}"

gripper_manager_pid=""

bool_is_true() {
  case "${1,,}" in
    1|true|yes|y|on) return 0 ;;
    *) return 1 ;;
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
echo "[handheld-hls-grasp-test] current: low=${HLS_LOW_CURRENT} lift=${HLS_LIFT_CURRENT}"
echo "[handheld-hls-grasp-test] motion profile: name=${HLS_MOTION_PROFILE:-<json-default>} index=${HLS_MOTION_PROFILE_INDEX:-<none>}"
echo "[handheld-hls-grasp-test] center: gain=${HLS_CENTER_GAIN_M_PER_RATIO} bias=${HLS_CENTER_BIAS} sign=${HLS_CENTER_SIGN}"
echo "[handheld-hls-grasp-test] single-contact: timeout=${HLS_SINGLE_CONTACT_TIMEOUT_S} limit_ratio=${HLS_SINGLE_CONTACT_LIMIT_RATIO} dry_run_pattern=${HLS_DRY_RUN_CONTACT_PATTERN}"
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
    -p low_current:="${HLS_LOW_CURRENT}"
    -p lift_current:="${HLS_LIFT_CURRENT}"
    -p center_gain_m_per_ratio:="${HLS_CENTER_GAIN_M_PER_RATIO}"
    -p center_bias:="${HLS_CENTER_BIAS}"
    -p center_sign:="${HLS_CENTER_SIGN}"
    -p single_contact_timeout_s:="${HLS_SINGLE_CONTACT_TIMEOUT_S}"
    -p single_contact_limit_ratio:="${HLS_SINGLE_CONTACT_LIMIT_RATIO}"
    -p dry_run_contact_pattern:="${HLS_DRY_RUN_CONTACT_PATTERN}"
  )
  if [[ -n "${HLS_GRAVITY_COMP_PATH}" ]]; then
    hls_args+=(-p gravity_comp_path:="${HLS_GRAVITY_COMP_PATH}")
  fi
  if [[ -n "${HLS_MOTION_PROFILE}" ]]; then
    hls_args+=(-p motion_profile:="${HLS_MOTION_PROFILE}")
  fi
  if [[ -n "${HLS_MOTION_PROFILE_INDEX}" ]]; then
    hls_args+=(-p motion_profile_index:="${HLS_MOTION_PROFILE_INDEX}")
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
  --ch10-index "${CH10_INDEX}"
  --ch10-open-pwm "${CH10_OPEN_PWM}"
  --ch10-close-pwm "${CH10_CLOSE_PWM}"
  --open-command "${OPEN_COMMAND}"
  --close-command "${CLOSE_COMMAND}"
)
if [[ -n "${CSV_PATH}" ]]; then
  test_args+=(--csv-path "${CSV_PATH}")
fi

ros2 run px4ctrl handheld_hls_grasp_test.py "${test_args[@]}"
