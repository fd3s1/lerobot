#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

GRASP_PARAMS_FILE="${GRASP_PARAMS_FILE:-${SCRIPT_DIR}/grasp_params.env}"
if [[ -n "${GRASP_PARAMS_FILE}" && -f "${GRASP_PARAMS_FILE}" ]]; then
  # shellcheck source=/dev/null
  source "${GRASP_PARAMS_FILE}"
fi

DRONE_POSE_TOPIC="${DRONE_POSE_TOPIC:-/mavros/vision_pose/pose}"
TARGET_POSE_TOPIC="${TARGET_POSE_TOPIC:-/strawberry_bear/pose}"
BOX_POSE_TOPIC="${BOX_POSE_TOPIC:-/box1/pose}"
RC_TOPIC="${RC_TOPIC:-/mavros/rc/in}"
GRIPPER_COMMAND_PAIR_TOPIC="${GRIPPER_COMMAND_PAIR_TOPIC:-/gripper/command_pair}"
GRIPPER_FEEDBACK_TOPIC="${GRIPPER_FEEDBACK_TOPIC:-/gripper/feedback}"
HANDHELD_GRIPPER_SCALAR_TOPIC="${HANDHELD_GRIPPER_SCALAR_TOPIC:-/handheld_grasp_tune/ignore_scalar_command}"

START_GRIPPER_MANAGER="${START_GRIPPER_MANAGER:-true}"
GRIPPER_MANAGER_PORT="${GRIPPER_MANAGER_PORT:-/dev/ttyACM1}"
GRIPPER_MANAGER_FEEDBACK_RATE_HZ="${GRIPPER_MANAGER_FEEDBACK_RATE_HZ:-20.0}"
GRIPPER_MANAGER_TORQUE_LIMIT="${GRIPPER_MANAGER_TORQUE_LIMIT:-0}"
GRIPPER_MANAGER_GOAL_VELOCITY="${GRIPPER_MANAGER_GOAL_VELOCITY:-0}"
GRIPPER_MANAGER_ACCELERATION="${GRIPPER_MANAGER_ACCELERATION:-0}"

RATE_HZ="${RATE_HZ:-20}"
POSE_TIMEOUT_S="${POSE_TIMEOUT_S:-0.8}"
GRIPPER_FEEDBACK_TIMEOUT_S="${GRIPPER_FEEDBACK_TIMEOUT_S:-0.8}"
CH10_INDEX="${CH10_INDEX:-9}"
CH10_THRESHOLD="${CH10_THRESHOLD:-1500}"

GRIPPER_OPEN="${GRIPPER_OPEN:-100.0}"
MANUAL_OVERRIDE_POS="${MANUAL_OVERRIDE_POS:-15.0}"
GRIPPER_Z_OFFSET_M="${GRIPPER_Z_OFFSET_M:-0.25}"
TARGET_HEIGHT_M="${TARGET_HEIGHT_M:-0.30}"
TARGET_GRASP_HEIGHT_M="${TARGET_GRASP_HEIGHT_M:-0.17}"
TARGET_POSE_Z_REFERENCE="${TARGET_POSE_Z_REFERENCE:-center}"
TARGET_HOVER_CLEARANCE_M="${TARGET_HOVER_CLEARANCE_M:-0.45}"
BOX_HEIGHT_M="${BOX_HEIGHT_M:-0.14}"
BOX_HOVER_GRIPPER_CLEARANCE_M="${BOX_HOVER_GRIPPER_CLEARANCE_M:-0.55}"
BOX_PLACE_BOTTOM_CLEARANCE_M="${BOX_PLACE_BOTTOM_CLEARANCE_M:-0.03}"

TARGET_OFFSET_X="${TARGET_OFFSET_X:-0.0}"
TARGET_OFFSET_Y="${TARGET_OFFSET_Y:-0.0}"
TARGET_OFFSET_Z="${TARGET_OFFSET_Z:-0.0}"
BOX_OFFSET_X="${BOX_OFFSET_X:-0.0}"
BOX_OFFSET_Y="${BOX_OFFSET_Y:-0.0}"
BOX_OFFSET_Z="${BOX_OFFSET_Z:-0.0}"

GRASP_STEP_SIZE="${GRASP_STEP_SIZE:-3.0}"
GRASP_STEP_SETTLE_S="${GRASP_STEP_SETTLE_S:-0.10}"
GRASP_OPEN_TIMEOUT_S="${GRASP_OPEN_TIMEOUT_S:-2.0}"
GRASP_OPEN_TOLERANCE="${GRASP_OPEN_TOLERANCE:-5.0}"
GRASP_STEP_TIMEOUT_S="${GRASP_STEP_TIMEOUT_S:-0.80}"
GRASP_GOAL_TOLERANCE="${GRASP_GOAL_TOLERANCE:-3.0}"
GRASP_CLOSE_MIN="${GRASP_CLOSE_MIN:-15.0}"
GRASP_CONTACT_CURRENT_DELTA="${GRASP_CONTACT_CURRENT_DELTA:-250}"
GRASP_CONTACT_LOAD_DELTA="${GRASP_CONTACT_LOAD_DELTA:-800}"
GRASP_POSITION_ERROR_THRESHOLD="${GRASP_POSITION_ERROR_THRESHOLD:-20.0}"
GRASP_ANGLE_CONTACT_DELTA="${GRASP_ANGLE_CONTACT_DELTA:-20.0}"
GRASP_STALL_DELTA="${GRASP_STALL_DELTA:-0.25}"
GRASP_CONTACT_MIN_CLOSE_DELTA="${GRASP_CONTACT_MIN_CLOSE_DELTA:-15.0}"
GRASP_CONTACT_CONFIRM_STEPS="${GRASP_CONTACT_CONFIRM_STEPS:-2}"
GRASP_BALANCE_LOAD_DIFF="${GRASP_BALANCE_LOAD_DIFF:-60}"
GRASP_BALANCE_STEP="${GRASP_BALANCE_STEP:-1.5}"
GRASP_MAX_BALANCE_STEPS="${GRASP_MAX_BALANCE_STEPS:-8}"
GRASP_ANGLE_BALANCE_DIFF="${GRASP_ANGLE_BALANCE_DIFF:-5.0}"

gripper_manager_pid=""

cleanup() {
  if [[ -n "${gripper_manager_pid}" ]] && kill -0 "${gripper_manager_pid}" 2>/dev/null; then
    echo "[handheld-grasp-tune] stopping gripper manager ${gripper_manager_pid}"
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

echo "[handheld-grasp-tune] drone topic: ${DRONE_POSE_TOPIC}"
echo "[handheld-grasp-tune] target topic: ${TARGET_POSE_TOPIC}"
echo "[handheld-grasp-tune] box topic: ${BOX_POSE_TOPIC}"
echo "[handheld-grasp-tune] rc topic: ${RC_TOPIC} ch10_index=${CH10_INDEX} threshold=${CH10_THRESHOLD}"
echo "[handheld-grasp-tune] gripper pair topic: ${GRIPPER_COMMAND_PAIR_TOPIC}"
echo "[handheld-grasp-tune] gripper feedback topic: ${GRIPPER_FEEDBACK_TOPIC}"
echo "[handheld-grasp-tune] gripper manager: start=${START_GRIPPER_MANAGER} port=${GRIPPER_MANAGER_PORT}"
echo "[handheld-grasp-tune] scalar command is isolated at: ${HANDHELD_GRIPPER_SCALAR_TOPIC}"
echo "[handheld-grasp-tune] grasp params file: ${GRASP_PARAMS_FILE:-<none>}"
echo "[handheld-grasp-tune] soft grasp: step=${GRASP_STEP_SIZE} settle=${GRASP_STEP_SETTLE_S}s step_timeout=${GRASP_STEP_TIMEOUT_S}s goal_tol=${GRASP_GOAL_TOLERANCE} open_timeout=${GRASP_OPEN_TIMEOUT_S}s open_tol=${GRASP_OPEN_TOLERANCE} min=${GRASP_CLOSE_MIN} angle_lag=${GRASP_ANGLE_CONTACT_DELTA} stall=${GRASP_STALL_DELTA} min_close=${GRASP_CONTACT_MIN_CLOSE_DELTA}"
echo "[handheld-grasp-tune] offsets: target=(${TARGET_OFFSET_X}, ${TARGET_OFFSET_Y}, ${TARGET_OFFSET_Z}) box=(${BOX_OFFSET_X}, ${BOX_OFFSET_Y}, ${BOX_OFFSET_Z})"
echo "[handheld-grasp-tune] CH10 low=automatic soft grasp, CH10 high=manual override pair=${MANUAL_OVERRIDE_POS}"
echo "[handheld-grasp-tune] no record, no takeoff, no /position_cmd will be published."

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

existing_gripper_nodes="$(ros2 node list 2>/dev/null | grep -E '(^|/)feetech_gripper_node($|_)' || true)"
if [[ -n "${existing_gripper_nodes}" ]]; then
  echo "[handheld-grasp-tune] WARNING: existing feetech_gripper_node detected before starting:" >&2
  echo "${existing_gripper_nodes}" >&2
  echo "[handheld-grasp-tune] Stop old gripper nodes first, otherwise CH10 scalar /gripper/command may look like manual control." >&2
fi

if [[ "${START_GRIPPER_MANAGER,,}" =~ ^(1|true|yes|y|on)$ ]]; then
  setsid ros2 run px4ctrl feetech_gripper_node.py --ros-args \
    -p port:="${GRIPPER_MANAGER_PORT}" \
    -p command_topic:="${HANDHELD_GRIPPER_SCALAR_TOPIC}" \
    -p command_pair_topic:="${GRIPPER_COMMAND_PAIR_TOPIC}" \
    -p feedback_topic:="${GRIPPER_FEEDBACK_TOPIC}" \
    -p feedback_rate_hz:="${GRIPPER_MANAGER_FEEDBACK_RATE_HZ}" \
    -p torque_limit:="${GRIPPER_MANAGER_TORQUE_LIMIT}" \
    -p goal_velocity:="${GRIPPER_MANAGER_GOAL_VELOCITY}" \
    -p acceleration:="${GRIPPER_MANAGER_ACCELERATION}" &
  gripper_manager_pid="$!"
  sleep 1.0
fi

ros2 run px4ctrl handheld_grasp_tune.py \
  --drone-pose-topic "${DRONE_POSE_TOPIC}" \
  --target-pose-topic "${TARGET_POSE_TOPIC}" \
  --box-pose-topic "${BOX_POSE_TOPIC}" \
  --rc-topic "${RC_TOPIC}" \
  --gripper-command-pair-topic "${GRIPPER_COMMAND_PAIR_TOPIC}" \
  --gripper-feedback-topic "${GRIPPER_FEEDBACK_TOPIC}" \
  --rate-hz "${RATE_HZ}" \
  --pose-timeout-s "${POSE_TIMEOUT_S}" \
  --feedback-timeout-s "${GRIPPER_FEEDBACK_TIMEOUT_S}" \
  --ch10-index "${CH10_INDEX}" \
  --ch10-threshold "${CH10_THRESHOLD}" \
  --gripper-open "${GRIPPER_OPEN}" \
  --manual-override-pos "${MANUAL_OVERRIDE_POS}" \
  --gripper-z-offset-m "${GRIPPER_Z_OFFSET_M}" \
  --target-height-m "${TARGET_HEIGHT_M}" \
  --target-grasp-height-m "${TARGET_GRASP_HEIGHT_M}" \
  --target-pose-z-reference "${TARGET_POSE_Z_REFERENCE}" \
  --target-hover-clearance-m "${TARGET_HOVER_CLEARANCE_M}" \
  --box-height-m "${BOX_HEIGHT_M}" \
  --box-hover-gripper-clearance-m "${BOX_HOVER_GRIPPER_CLEARANCE_M}" \
  --box-place-bottom-clearance-m "${BOX_PLACE_BOTTOM_CLEARANCE_M}" \
  --target-offset-x "${TARGET_OFFSET_X}" \
  --target-offset-y "${TARGET_OFFSET_Y}" \
  --target-offset-z "${TARGET_OFFSET_Z}" \
  --box-offset-x "${BOX_OFFSET_X}" \
  --box-offset-y "${BOX_OFFSET_Y}" \
  --box-offset-z "${BOX_OFFSET_Z}" \
  --grasp-step-size "${GRASP_STEP_SIZE}" \
  --grasp-step-settle-s "${GRASP_STEP_SETTLE_S}" \
  --grasp-open-timeout-s "${GRASP_OPEN_TIMEOUT_S}" \
  --grasp-open-tolerance "${GRASP_OPEN_TOLERANCE}" \
  --grasp-step-timeout-s "${GRASP_STEP_TIMEOUT_S}" \
  --grasp-goal-tolerance "${GRASP_GOAL_TOLERANCE}" \
  --grasp-close-min "${GRASP_CLOSE_MIN}" \
  --grasp-contact-current-delta "${GRASP_CONTACT_CURRENT_DELTA}" \
  --grasp-contact-load-delta "${GRASP_CONTACT_LOAD_DELTA}" \
  --grasp-position-error-threshold "${GRASP_POSITION_ERROR_THRESHOLD}" \
  --grasp-angle-contact-delta "${GRASP_ANGLE_CONTACT_DELTA}" \
  --grasp-stall-delta "${GRASP_STALL_DELTA}" \
  --grasp-contact-min-close-delta "${GRASP_CONTACT_MIN_CLOSE_DELTA}" \
  --grasp-contact-confirm-steps "${GRASP_CONTACT_CONFIRM_STEPS}" \
  --grasp-balance-load-diff "${GRASP_BALANCE_LOAD_DIFF}" \
  --grasp-balance-step "${GRASP_BALANCE_STEP}" \
  --grasp-max-balance-steps "${GRASP_MAX_BALANCE_STEPS}" \
  --grasp-angle-balance-diff "${GRASP_ANGLE_BALANCE_DIFF}"
