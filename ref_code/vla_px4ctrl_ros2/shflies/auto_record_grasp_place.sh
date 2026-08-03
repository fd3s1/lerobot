#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

GRASP_PARAMS_FILE="${GRASP_PARAMS_FILE:-${SCRIPT_DIR}/grasp_params.env}"
if [[ -n "${GRASP_PARAMS_FILE}" && -f "${GRASP_PARAMS_FILE}" ]]; then
  # shellcheck source=/dev/null
  source "${GRASP_PARAMS_FILE}"
fi

CONDA_ENV="${CONDA_ENV:-vla-drone-v044}"
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
GRIPPER_FEEDBACK_TIMEOUT_S="${GRIPPER_FEEDBACK_TIMEOUT_S:-0.5}"
START_GRIPPER_MANAGER="${START_GRIPPER_MANAGER:-true}"
GRIPPER_MANAGER_PORT="${GRIPPER_MANAGER_PORT:-/dev/ttyACM1}"
GRIPPER_MANAGER_FEEDBACK_RATE_HZ="${GRIPPER_MANAGER_FEEDBACK_RATE_HZ:-20.0}"
GRIPPER_MANAGER_TORQUE_LIMIT="${GRIPPER_MANAGER_TORQUE_LIMIT:-0}"
GRIPPER_MANAGER_GOAL_VELOCITY="${GRIPPER_MANAGER_GOAL_VELOCITY:-0}"
GRIPPER_MANAGER_ACCELERATION="${GRIPPER_MANAGER_ACCELERATION:-0}"
TAKEOFF_LAND_TOPIC="${TAKEOFF_LAND_TOPIC:-/px4ctrl/takeoff_land}"
PX4CTRL_STATE_TOPIC="${PX4CTRL_STATE_TOPIC:-/px4ctrl/state}"
TAKEOFF_MODE="${TAKEOFF_MODE:-auto}"
RECORD_STATUS_TOPIC="${RECORD_STATUS_TOPIC:-/lerobot_record/status}"
RECORD_GATE_TOPIC="${RECORD_GATE_TOPIC:-/auto_grasp_dataset/record_gate}"
RECORD_GATE_VALUE="${RECORD_GATE_VALUE:-START}"
POSE_PREFLIGHT_TIMEOUT_S="${POSE_PREFLIGHT_TIMEOUT_S:-6}"
SKIP_POSE_PREFLIGHT="${SKIP_POSE_PREFLIGHT:-false}"
POSE_PREFLIGHT_REQUIRED="${POSE_PREFLIGHT_REQUIRED:-false}"

EPISODE_TIME_S="${EPISODE_TIME_S:-30}"
MAX_SPEED="${MAX_SPEED:-0.6}"
APPROACH_SPEED="${APPROACH_SPEED:-0.3}"
LIFT_SPEED="${LIFT_SPEED:-0.4}"
PAYLOAD_LIFT_SPEED="${PAYLOAD_LIFT_SPEED:-0.10}"
PAYLOAD_TRANSFER_SPEED="${PAYLOAD_TRANSFER_SPEED:-0.16}"
POST_GRASP_SETTLE_S="${POST_GRASP_SETTLE_S:-1.5}"
POST_LIFT_SETTLE_S="${POST_LIFT_SETTLE_S:-1.5}"
SMOOTH_TRAJECTORY="${SMOOTH_TRAJECTORY:-true}"
PAYLOAD_LIFT_FORWARD_COMP_M="${PAYLOAD_LIFT_FORWARD_COMP_M:-0.0}"
PAYLOAD_LIFT_COMP_X="${PAYLOAD_LIFT_COMP_X:-0.0}"
PAYLOAD_LIFT_COMP_Y="${PAYLOAD_LIFT_COMP_Y:-0.0}"
PAYLOAD_LIFT_COMP_Z="${PAYLOAD_LIFT_COMP_Z:-0.0}"
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
TARGET_OFFSET_X="${TARGET_OFFSET_X:-0.0}"
TARGET_OFFSET_Y="${TARGET_OFFSET_Y:-0.0}"
TARGET_OFFSET_Z="${TARGET_OFFSET_Z:-0.0}"
TARGET_GRASP_X_BIAS_WAS_SET="${TARGET_GRASP_X_BIAS_M+x}"
TARGET_GRASP_X_BIAS_M="${TARGET_GRASP_X_BIAS_M:--0.02}"
TARGET_HOVER_MAP_X_BIAS_WAS_SET="${TARGET_HOVER_MAP_X_BIAS_M+x}"
TARGET_HOVER_MAP_X_BIAS_M="${TARGET_HOVER_MAP_X_BIAS_M:-0.0}"
TARGET_GRASP_MAP_X_BIAS_WAS_SET="${TARGET_GRASP_MAP_X_BIAS_M+x}"
TARGET_GRASP_MAP_X_BIAS_M="${TARGET_GRASP_MAP_X_BIAS_M:-0.0}"
BOX_OFFSET_X="${BOX_OFFSET_X:-0.0}"
BOX_OFFSET_Y="${BOX_OFFSET_Y:-0.0}"
BOX_OFFSET_Z="${BOX_OFFSET_Z:-0.0}"
RELEASE_RETREAT_UP_M="${RELEASE_RETREAT_UP_M:-0.3}"
RELEASE_RETREAT_FORWARD_M="${RELEASE_RETREAT_FORWARD_M:-2.0}"
LANDING_MODE="${LANDING_MODE:-cmd}"
CMD_LAND_SPEED="${CMD_LAND_SPEED:-0.25}"
CMD_LAND_Z="${CMD_LAND_Z:--0.3}"
CMD_LAND_Z_OFFSET_M="${CMD_LAND_Z_OFFSET_M:-0.0}"
GRASP_MODE="${GRASP_MODE:-continuous_center}"
GRIPPER_CLOSED="${GRIPPER_CLOSED:-0.0}"
GRIPPER_CLOSE_DURATION_S="${GRIPPER_CLOSE_DURATION_S:-4.0}"
GRIPPER_X_OFFSET_M="${GRIPPER_X_OFFSET_M:-0.0}"
GRIPPER_Y_OFFSET_M="${GRIPPER_Y_OFFSET_M:-0.0}"
WAYPOINT_ARRIVAL_TOLERANCE_M="${WAYPOINT_ARRIVAL_TOLERANCE_M:-0.08}"
WAYPOINT_ARRIVAL_SETTLE_S="${WAYPOINT_ARRIVAL_SETTLE_S:-0.4}"
WAYPOINT_ARRIVAL_TIMEOUT_S="${WAYPOINT_ARRIVAL_TIMEOUT_S:-15.0}"
TARGET_YAW_ALIGN_ENABLE="${TARGET_YAW_ALIGN_ENABLE:-true}"
TARGET_YAW_ALIGN_OFFSET_RAD="${TARGET_YAW_ALIGN_OFFSET_RAD:-0.0}"
TARGET_YAW_ALIGN_RATE_DPS="${TARGET_YAW_ALIGN_RATE_DPS:-30.0}"
TARGET_YAW_ALIGN_TOL_DEG="${TARGET_YAW_ALIGN_TOL_DEG:-12.0}"
TARGET_YAW_ALIGN_MIN_DISTANCE_M="${TARGET_YAW_ALIGN_MIN_DISTANCE_M:-0.20}"
TARGET_YAW_ALIGN_MAX_DURATION_S="${TARGET_YAW_ALIGN_MAX_DURATION_S:-3.0}"
GRIPPER_OPEN_DURATION_S="${GRIPPER_OPEN_DURATION_S:-0.4}"
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
NO_LAND="${NO_LAND:-false}"

bool_is_true() {
  case "${1,,}" in
    1|true|yes|y|on) return 0 ;;
    *) return 1 ;;
  esac
}

if bool_is_true "${TRAJ_PLANNER_ENABLE}"; then
  if [[ "${CMD_TOPIC}" == "/position_cmd" ]]; then
    CMD_TOPIC="${TRAJ_PLANNER_INPUT_TOPIC}"
  fi
  if [[ -z "${TARGET_GRASP_X_BIAS_WAS_SET}" ]]; then
    TARGET_GRASP_X_BIAS_M="0.0"
  fi
  if [[ -z "${TARGET_HOVER_MAP_X_BIAS_WAS_SET}" ]]; then
    TARGET_HOVER_MAP_X_BIAS_M="-0.02"
  fi
  if [[ -z "${TARGET_GRASP_MAP_X_BIAS_WAS_SET}" ]]; then
    TARGET_GRASP_MAP_X_BIAS_M="-0.02"
  fi
fi

check_pose_topic_once() {
  local label="$1"
  local topic="$2"

  echo "[auto-record-grasp-place] checking ${label} pose topic: ${topic}"
  if timeout "${POSE_PREFLIGHT_TIMEOUT_S}s" \
    ros2 topic echo --once --qos-reliability best_effort "${topic}" >/dev/null 2>&1; then
    echo "[auto-record-grasp-place] ${label} pose topic is live: ${topic}"
    return 0
  fi

  echo "[auto-record-grasp-place] WARNING: no fresh ${label} pose on ${topic} within ${POSE_PREFLIGHT_TIMEOUT_S}s during one-shot preflight" >&2
  return 1
}

record_pid=""
record_uses_setsid=false
gripper_manager_pid=""

cleanup() {
  if [[ -n "${record_pid}" ]] && kill -0 "${record_pid}" 2>/dev/null; then
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

  if [[ -n "${gripper_manager_pid}" ]] && kill -0 "${gripper_manager_pid}" 2>/dev/null; then
    echo "[auto-record-grasp-place] stopping gripper manager ${gripper_manager_pid}"
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

  gripper_manager_pid=""
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM HUP

echo "[auto-record-grasp-place] target topic: ${TARGET_POSE_TOPIC}"
echo "[auto-record-grasp-place] box topic: ${BOX_POSE_TOPIC}"
echo "[auto-record-grasp-place] control drone pose topic: ${DRONE_POSE_TOPIC}"
echo "[auto-record-grasp-place] arrival check pose topic: ${ARRIVAL_POSE_TOPIC}"
echo "[auto-record-grasp-place] gripper scalar topic: ${GRIPPER_TOPIC}"
echo "[auto-record-grasp-place] gripper pair topic: ${GRIPPER_COMMAND_PAIR_TOPIC}"
echo "[auto-record-grasp-place] gripper feedback topic: ${GRIPPER_FEEDBACK_TOPIC}"
echo "[auto-record-grasp-place] gripper manager: start=${START_GRIPPER_MANAGER} port=${GRIPPER_MANAGER_PORT} feedback_rate=${GRIPPER_MANAGER_FEEDBACK_RATE_HZ}Hz"
echo "[auto-record-grasp-place] grasp params file: ${GRASP_PARAMS_FILE:-<none>}"
echo "[auto-record-grasp-place] record gate: ${RECORD_GATE_TOPIC}=${RECORD_GATE_VALUE}"
echo "[auto-record-grasp-place] record status topic: ${RECORD_STATUS_TOPIC}"
echo "[auto-record-grasp-place] episode time: ${EPISODE_TIME_S}s"
echo "[auto-record-grasp-place] speeds: max=${MAX_SPEED} approach=${APPROACH_SPEED} lift=${LIFT_SPEED} retreat=${RETREAT_SPEED}"
echo "[auto-record-grasp-place] trajectory planner: enable=${TRAJ_PLANNER_ENABLE} cmd_topic=${CMD_TOPIC} raw_topic=${TRAJ_PLANNER_INPUT_TOPIC}"
echo "[auto-record-grasp-place] target bias: body_grasp_x=${TARGET_GRASP_X_BIAS_M} map_hover_x=${TARGET_HOVER_MAP_X_BIAS_M} map_grasp_x=${TARGET_GRASP_MAP_X_BIAS_M}"
echo "[auto-record-grasp-place] payload damping: lift_speed=${PAYLOAD_LIFT_SPEED} transfer_speed=${PAYLOAD_TRANSFER_SPEED} post_grasp_settle=${POST_GRASP_SETTLE_S}s post_lift_settle=${POST_LIFT_SETTLE_S}s smooth=${SMOOTH_TRAJECTORY}"
echo "[auto-record-grasp-place] target yaw align: enable=${TARGET_YAW_ALIGN_ENABLE} offset=${TARGET_YAW_ALIGN_OFFSET_RAD}rad rate=${TARGET_YAW_ALIGN_RATE_DPS}deg/s tol=${TARGET_YAW_ALIGN_TOL_DEG}deg min_distance=${TARGET_YAW_ALIGN_MIN_DISTANCE_M}m max_duration=${TARGET_YAW_ALIGN_MAX_DURATION_S}s"
echo "[auto-record-grasp-place] payload lift compensation: forward=${PAYLOAD_LIFT_FORWARD_COMP_M}m map=(${PAYLOAD_LIFT_COMP_X}, ${PAYLOAD_LIFT_COMP_Y}, ${PAYLOAD_LIFT_COMP_Z})m"
echo "[auto-record-grasp-place] geometry: gripper_z_offset=${GRIPPER_Z_OFFSET_M}m target_h=${TARGET_HEIGHT_M}m target_grasp_h=${TARGET_GRASP_HEIGHT_M}m target_z_ref=${TARGET_POSE_Z_REFERENCE}"
echo "[auto-record-grasp-place] box: l=${BOX_LENGTH_M}m w=${BOX_WIDTH_M}m h=${BOX_HEIGHT_M}m hover_clearance=${BOX_HOVER_GRIPPER_CLEARANCE_M}m place_bottom_clearance=${BOX_PLACE_BOTTOM_CLEARANCE_M}m"
echo "[auto-record-grasp-place] planning offsets: target=(${TARGET_OFFSET_X}, ${TARGET_OFFSET_Y}, ${TARGET_OFFSET_Z})m box=(${BOX_OFFSET_X}, ${BOX_OFFSET_Y}, ${BOX_OFFSET_Z})m"
echo "[auto-record-grasp-place] gripper body-frame xy offset: x=${GRIPPER_X_OFFSET_M}m forward, y=${GRIPPER_Y_OFFSET_M}m left"
echo "[auto-record-grasp-place] actual arrival gate: tol=${WAYPOINT_ARRIVAL_TOLERANCE_M}m settle=${WAYPOINT_ARRIVAL_SETTLE_S}s timeout=${WAYPOINT_ARRIVAL_TIMEOUT_S}s"
echo "[auto-record-grasp-place] takeoff mode: ${TAKEOFF_MODE}"
echo "[auto-record-grasp-place] grasp mode: ${GRASP_MODE} closed=${GRIPPER_CLOSED} close_duration=${GRIPPER_CLOSE_DURATION_S}s"
echo "[auto-record-grasp-place] soft grasp params: step=${GRASP_STEP_SIZE} settle=${GRASP_STEP_SETTLE_S}s step_timeout=${GRASP_STEP_TIMEOUT_S}s goal_tol=${GRASP_GOAL_TOLERANCE} open_timeout=${GRASP_OPEN_TIMEOUT_S}s open_tol=${GRASP_OPEN_TOLERANCE} min=${GRASP_CLOSE_MIN} current_delta=${GRASP_CONTACT_CURRENT_DELTA} load_delta=${GRASP_CONTACT_LOAD_DELTA} pos_err=${GRASP_POSITION_ERROR_THRESHOLD} angle_lag=${GRASP_ANGLE_CONTACT_DELTA} stall=${GRASP_STALL_DELTA} min_close=${GRASP_CONTACT_MIN_CLOSE_DELTA}"
echo "[auto-record-grasp-place] soft grasp balance: load_diff=${GRASP_BALANCE_LOAD_DIFF} angle_diff=${GRASP_ANGLE_BALANCE_DIFF} balance_step=${GRASP_BALANCE_STEP} max_balance=${GRASP_MAX_BALANCE_STEPS}"
echo "[auto-record-grasp-place] release retreat: up=${RELEASE_RETREAT_UP_M}m forward=${RELEASE_RETREAT_FORWARD_M}m"
echo "[auto-record-grasp-place] landing: mode=${LANDING_MODE} cmd_speed=${CMD_LAND_SPEED} cmd_z=${CMD_LAND_Z:-pre_takeoff_z+${CMD_LAND_Z_OFFSET_M}}"
echo "[auto-record-grasp-place] pose preflight: timeout=${POSE_PREFLIGHT_TIMEOUT_S}s required=${POSE_PREFLIGHT_REQUIRED} skip=${SKIP_POSE_PREFLIGHT}"

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
  if [[ "${preflight_failed}" == "true" ]]; then
    echo "[auto-record-grasp-place] One-shot pose preflight did not see every topic." >&2
    echo "[auto-record-grasp-place] This can be a DDS discovery delay; continuing by default." >&2
    echo "[auto-record-grasp-place] The auto task node will keep waiting for fresh poses before takeoff." >&2
    echo "[auto-record-grasp-place] To make this a hard failure, set POSE_PREFLIGHT_REQUIRED=true." >&2
    echo "[auto-record-grasp-place] To skip this advisory check, set SKIP_POSE_PREFLIGHT=true." >&2
    if bool_is_true "${POSE_PREFLIGHT_REQUIRED}"; then
      echo "[auto-record-grasp-place] Start mocap/MAVROS first, usually:" >&2
      echo "[auto-record-grasp-place]   bash shflies/run_mocap_mavros.sh" >&2
      echo "[auto-record-grasp-place] Or check actual VRPN names with:" >&2
      echo "[auto-record-grasp-place]   ros2 topic list | grep -E 'strawberry|box|pose'" >&2
      exit 1
    fi
  fi
fi

if bool_is_true "${START_GRIPPER_MANAGER}"; then
  setsid ros2 run px4ctrl feetech_gripper_node.py --ros-args \
    -p port:="${GRIPPER_MANAGER_PORT}" \
    -p command_topic:="${GRIPPER_TOPIC}" \
    -p command_pair_topic:="${GRIPPER_COMMAND_PAIR_TOPIC}" \
    -p feedback_topic:="${GRIPPER_FEEDBACK_TOPIC}" \
    -p feedback_rate_hz:="${GRIPPER_MANAGER_FEEDBACK_RATE_HZ}" \
    -p torque_limit:="${GRIPPER_MANAGER_TORQUE_LIMIT}" \
    -p goal_velocity:="${GRIPPER_MANAGER_GOAL_VELOCITY}" \
    -p acceleration:="${GRIPPER_MANAGER_ACCELERATION}" &
  gripper_manager_pid="$!"
  sleep 1.0
fi

START_GATE_TOPIC="${RECORD_GATE_TOPIC}" \
START_GATE_VALUE="${RECORD_GATE_VALUE}" \
START_GATE_STABLE_S=0.0 \
RECORD_PREWARM_STEPS=0 \
DATASET_STATUS_TOPIC="${RECORD_STATUS_TOPIC}" \
USE_ROS_GRIPPER=true \
GRIPPER_COMMAND_PAIR_TOPIC="${GRIPPER_COMMAND_PAIR_TOPIC}" \
GRIPPER_FEEDBACK_TOPIC="${GRIPPER_FEEDBACK_TOPIC}" \
GRIPPER_FEEDBACK_TIMEOUT_S="${GRIPPER_FEEDBACK_TIMEOUT_S}" \
EPISODE_TIME_S="${EPISODE_TIME_S}" \
TASK="${TASK:-Pick up the yellow paper roll from the black platform and place it into the white box}" \
setsid bash "${SCRIPT_DIR}/record_vla_dataset.sh" &
record_pid="$!"
record_uses_setsid=true

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
  --record-status-topic "${RECORD_STATUS_TOPIC}"
  --record-gate-topic "${RECORD_GATE_TOPIC}"
  --record-gate-value "${RECORD_GATE_VALUE}"
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
  --target-offset-x "${TARGET_OFFSET_X}"
  --target-offset-y "${TARGET_OFFSET_Y}"
  --target-offset-z "${TARGET_OFFSET_Z}"
  --target-grasp-x-bias-m "${TARGET_GRASP_X_BIAS_M}"
  --target-hover-map-x-bias-m "${TARGET_HOVER_MAP_X_BIAS_M}"
  --target-grasp-map-x-bias-m "${TARGET_GRASP_MAP_X_BIAS_M}"
  --box-offset-x "${BOX_OFFSET_X}"
  --box-offset-y "${BOX_OFFSET_Y}"
  --box-offset-z "${BOX_OFFSET_Z}"
  --release-retreat-up-m "${RELEASE_RETREAT_UP_M}"
  --release-retreat-forward-m "${RELEASE_RETREAT_FORWARD_M}"
  --landing-mode "${LANDING_MODE}"
  --cmd-land-speed "${CMD_LAND_SPEED}"
  --cmd-land-z-offset-m "${CMD_LAND_Z_OFFSET_M}"
  --waypoint-arrival-tolerance-m "${WAYPOINT_ARRIVAL_TOLERANCE_M}"
  --waypoint-arrival-settle-s "${WAYPOINT_ARRIVAL_SETTLE_S}"
  --waypoint-arrival-timeout-s "${WAYPOINT_ARRIVAL_TIMEOUT_S}"
  --target-yaw-align-offset-rad "${TARGET_YAW_ALIGN_OFFSET_RAD}"
  --target-yaw-align-rate-dps "${TARGET_YAW_ALIGN_RATE_DPS}"
  --target-yaw-align-tol-deg "${TARGET_YAW_ALIGN_TOL_DEG}"
  --target-yaw-align-min-distance-m "${TARGET_YAW_ALIGN_MIN_DISTANCE_M}"
  --target-yaw-align-max-duration-s "${TARGET_YAW_ALIGN_MAX_DURATION_S}"
  --record-duration-s "${EPISODE_TIME_S}"
  --grasp-mode "${GRASP_MODE}"
  --gripper-closed "${GRIPPER_CLOSED}"
  --gripper-close-duration-s "${GRIPPER_CLOSE_DURATION_S}"
  --gripper-open-duration-s "${GRIPPER_OPEN_DURATION_S}"
  --grasp-step-size "${GRASP_STEP_SIZE}"
  --grasp-step-settle-s "${GRASP_STEP_SETTLE_S}"
  --grasp-open-timeout-s "${GRASP_OPEN_TIMEOUT_S}"
  --grasp-open-tolerance "${GRASP_OPEN_TOLERANCE}"
  --grasp-step-timeout-s "${GRASP_STEP_TIMEOUT_S}"
  --grasp-goal-tolerance "${GRASP_GOAL_TOLERANCE}"
  --grasp-close-min "${GRASP_CLOSE_MIN}"
  --grasp-contact-current-delta "${GRASP_CONTACT_CURRENT_DELTA}"
  --grasp-contact-load-delta "${GRASP_CONTACT_LOAD_DELTA}"
  --grasp-position-error-threshold "${GRASP_POSITION_ERROR_THRESHOLD}"
  --grasp-angle-contact-delta "${GRASP_ANGLE_CONTACT_DELTA}"
  --grasp-stall-delta "${GRASP_STALL_DELTA}"
  --grasp-contact-min-close-delta "${GRASP_CONTACT_MIN_CLOSE_DELTA}"
  --grasp-contact-confirm-steps "${GRASP_CONTACT_CONFIRM_STEPS}"
  --grasp-balance-load-diff "${GRASP_BALANCE_LOAD_DIFF}"
  --grasp-balance-step "${GRASP_BALANCE_STEP}"
  --grasp-max-balance-steps "${GRASP_MAX_BALANCE_STEPS}"
  --grasp-angle-balance-diff "${GRASP_ANGLE_BALANCE_DIFF}"
)

if [[ -n "${CMD_LAND_Z}" ]]; then
  auto_args+=(--cmd-land-z "${CMD_LAND_Z}")
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

if bool_is_true "${TARGET_YAW_ALIGN_ENABLE}"; then
  auto_args+=(--target-yaw-align-enable)
else
  auto_args+=(--no-target-yaw-align-enable)
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
