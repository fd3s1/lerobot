#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONDA_ENV="${CONDA_ENV:-vla-drone-v044}"
TARGET_POSE_TOPIC="${TARGET_POSE_TOPIC:-/strawberry_bear/pose}"
BOX_POSE_TOPIC="${BOX_POSE_TOPIC:-/box1/pose}"
DRONE_POSE_TOPIC="${DRONE_POSE_TOPIC:-/mavros/vision_pose/pose}"
CMD_TOPIC="${CMD_TOPIC:-/position_cmd}"
GRIPPER_TOPIC="${GRIPPER_TOPIC:-/gripper/command}"
TAKEOFF_LAND_TOPIC="${TAKEOFF_LAND_TOPIC:-/px4ctrl/takeoff_land}"
PX4CTRL_STATE_TOPIC="${PX4CTRL_STATE_TOPIC:-/px4ctrl/state}"
RECORD_STATUS_TOPIC="${RECORD_STATUS_TOPIC:-/lerobot_record/status}"
RECORD_GATE_TOPIC="${RECORD_GATE_TOPIC:-/auto_grasp_dataset/record_gate}"
RECORD_GATE_VALUE="${RECORD_GATE_VALUE:-START}"

EPISODE_TIME_S="${EPISODE_TIME_S:-30}"
MAX_SPEED="${MAX_SPEED:-0.6}"
APPROACH_SPEED="${APPROACH_SPEED:-0.3}"
LIFT_SPEED="${LIFT_SPEED:-0.4}"
RETREAT_SPEED="${RETREAT_SPEED:-0.6}"
RATE_HZ="${RATE_HZ:-20}"
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
RELEASE_RETREAT_UP_M="${RELEASE_RETREAT_UP_M:-0.3}"
RELEASE_RETREAT_FORWARD_M="${RELEASE_RETREAT_FORWARD_M:-2.0}"
LANDING_MODE="${LANDING_MODE:-cmd}"
CMD_LAND_SPEED="${CMD_LAND_SPEED:-0.25}"
CMD_LAND_Z="${CMD_LAND_Z:-}"
CMD_LAND_Z_OFFSET_M="${CMD_LAND_Z_OFFSET_M:-0.0}"
GRIPPER_CLOSE_DURATION_S="${GRIPPER_CLOSE_DURATION_S:-1.5}"
GRIPPER_OPEN_DURATION_S="${GRIPPER_OPEN_DURATION_S:-0.4}"
NO_LAND="${NO_LAND:-false}"

bool_is_true() {
  case "${1,,}" in
    1|true|yes|y|on) return 0 ;;
    *) return 1 ;;
  esac
}

record_pid=""
record_uses_setsid=false

cleanup() {
  if [[ -z "${record_pid}" ]]; then
    return
  fi

  if kill -0 "${record_pid}" 2>/dev/null; then
    echo "[auto-record-grasp-place] stopping record process ${record_pid}"
    if [[ "${record_uses_setsid}" == "true" ]]; then
      kill -TERM -- "-${record_pid}" 2>/dev/null || true
    else
      pkill -TERM -P "${record_pid}" 2>/dev/null || true
      kill -TERM "${record_pid}" 2>/dev/null || true
    fi

    for _ in {1..20}; do
      if ! kill -0 "${record_pid}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done

    if kill -0 "${record_pid}" 2>/dev/null; then
      if [[ "${record_uses_setsid}" == "true" ]]; then
        kill -KILL -- "-${record_pid}" 2>/dev/null || true
      else
        pkill -KILL -P "${record_pid}" 2>/dev/null || true
        kill -KILL "${record_pid}" 2>/dev/null || true
      fi
    fi

    wait "${record_pid}" 2>/dev/null || true
  fi

  record_pid=""
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM HUP

echo "[auto-record-grasp-place] target topic: ${TARGET_POSE_TOPIC}"
echo "[auto-record-grasp-place] box topic: ${BOX_POSE_TOPIC}"
echo "[auto-record-grasp-place] drone topic: ${DRONE_POSE_TOPIC}"
echo "[auto-record-grasp-place] record gate: ${RECORD_GATE_TOPIC}=${RECORD_GATE_VALUE}"
echo "[auto-record-grasp-place] record status topic: ${RECORD_STATUS_TOPIC}"
echo "[auto-record-grasp-place] episode time: ${EPISODE_TIME_S}s"
echo "[auto-record-grasp-place] speeds: max=${MAX_SPEED} approach=${APPROACH_SPEED} lift=${LIFT_SPEED} retreat=${RETREAT_SPEED}"
echo "[auto-record-grasp-place] geometry: gripper_z_offset=${GRIPPER_Z_OFFSET_M}m target_h=${TARGET_HEIGHT_M}m target_grasp_h=${TARGET_GRASP_HEIGHT_M}m target_z_ref=${TARGET_POSE_Z_REFERENCE}"
echo "[auto-record-grasp-place] box: l=${BOX_LENGTH_M}m w=${BOX_WIDTH_M}m h=${BOX_HEIGHT_M}m hover_clearance=${BOX_HOVER_GRIPPER_CLEARANCE_M}m place_bottom_clearance=${BOX_PLACE_BOTTOM_CLEARANCE_M}m"
echo "[auto-record-grasp-place] release retreat: up=${RELEASE_RETREAT_UP_M}m forward=${RELEASE_RETREAT_FORWARD_M}m"
echo "[auto-record-grasp-place] landing: mode=${LANDING_MODE} cmd_speed=${CMD_LAND_SPEED} cmd_z=${CMD_LAND_Z:-pre_takeoff_z+${CMD_LAND_Z_OFFSET_M}}"

START_GATE_TOPIC="${RECORD_GATE_TOPIC}" \
START_GATE_VALUE="${RECORD_GATE_VALUE}" \
START_GATE_STABLE_S=0.0 \
RECORD_PREWARM_STEPS=0 \
DATASET_STATUS_TOPIC="${RECORD_STATUS_TOPIC}" \
EPISODE_TIME_S="${EPISODE_TIME_S}" \
TASK="${TASK:-Auto grasp target and place into box}" \
setsid bash "${SCRIPT_DIR}/record_vla_dataset.sh" &
record_pid="$!"
record_uses_setsid=true

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

auto_args=(
  --drone-pose-topic "${DRONE_POSE_TOPIC}"
  --target-pose-topic "${TARGET_POSE_TOPIC}"
  --box-pose-topic "${BOX_POSE_TOPIC}"
  --cmd-topic "${CMD_TOPIC}"
  --gripper-topic "${GRIPPER_TOPIC}"
  --takeoff-land-topic "${TAKEOFF_LAND_TOPIC}"
  --px4ctrl-state-topic "${PX4CTRL_STATE_TOPIC}"
  --record-status-topic "${RECORD_STATUS_TOPIC}"
  --record-gate-topic "${RECORD_GATE_TOPIC}"
  --record-gate-value "${RECORD_GATE_VALUE}"
  --rate-hz "${RATE_HZ}"
  --max-speed "${MAX_SPEED}"
  --approach-speed "${APPROACH_SPEED}"
  --lift-speed "${LIFT_SPEED}"
  --retreat-speed "${RETREAT_SPEED}"
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
  --release-retreat-up-m "${RELEASE_RETREAT_UP_M}"
  --release-retreat-forward-m "${RELEASE_RETREAT_FORWARD_M}"
  --landing-mode "${LANDING_MODE}"
  --cmd-land-speed "${CMD_LAND_SPEED}"
  --cmd-land-z-offset-m "${CMD_LAND_Z_OFFSET_M}"
  --record-duration-s "${EPISODE_TIME_S}"
  --gripper-close-duration-s "${GRIPPER_CLOSE_DURATION_S}"
  --gripper-open-duration-s "${GRIPPER_OPEN_DURATION_S}"
)

if [[ -n "${CMD_LAND_Z}" ]]; then
  auto_args+=(--cmd-land-z "${CMD_LAND_Z}")
fi

if bool_is_true "${NO_LAND}"; then
  auto_args+=(--no-land)
fi

set +e
ros2 run px4ctrl auto_grasp_place_dataset.py "${auto_args[@]}"
auto_status="$?"
set -e

if [[ "${auto_status}" -ne 0 ]]; then
  cleanup
  exit "${auto_status}"
fi

wait "${record_pid}"
record_pid=""
