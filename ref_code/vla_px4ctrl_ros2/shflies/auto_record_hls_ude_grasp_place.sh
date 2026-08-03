#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

ENABLE_RECORD="${ENABLE_RECORD:-true}"
AUTO_RECORD_RUN_ID="${AUTO_RECORD_RUN_ID:-$(date +%Y%m%d_%H%M%S)_$$}"
RECORD_STATUS_TOPIC="${RECORD_STATUS_TOPIC:-/lerobot_record/status_${AUTO_RECORD_RUN_ID}}"
RECORD_GATE_TOPIC="${RECORD_GATE_TOPIC:-/auto_hls_grasp_place/record_gate}"
RECORD_GATE_VALUE="${RECORD_GATE_VALUE:-START}"
RECORD_STOP_TOPIC="${RECORD_STOP_TOPIC:-/auto_hls_grasp_place/record_stop}"
RECORD_STOP_VALUE="${RECORD_STOP_VALUE:-STOP}"
RECORD_STOP_HOLD_S="${RECORD_STOP_HOLD_S:-3.0}"
RECORD_READY_TIMEOUT_S="${RECORD_READY_TIMEOUT_S:-120.0}"
RECORD_READY_LOG_PERIOD_S="${RECORD_READY_LOG_PERIOD_S:-0.0}"
WAIT_RECORD_ON_AUTO_EXIT="${WAIT_RECORD_ON_AUTO_EXIT:-true}"
RECORD_FINISH_TIMEOUT_S="${RECORD_FINISH_TIMEOUT_S:-}"
RECORD_GATE_AFTER_AUTO_GRACE_S="${RECORD_GATE_AFTER_AUTO_GRACE_S:-3.0}"
RECORD_START_HOLD_S="${RECORD_START_HOLD_S:-0.06}"
RECORD_PREWARM_STEPS="${RECORD_PREWARM_STEPS:-2}"
EPISODE_TIME_S="${EPISODE_TIME_S:-300}"
NUM_EPISODES="${NUM_EPISODES:-1}"
RESET_TIME_S="${RESET_TIME_S:-0}"
STREAMING_ENCODING="${STREAMING_ENCODING:-false}"
ENCODER_THREADS="${ENCODER_THREADS:-2}"
ENCODER_QUEUE_MAXSIZE="${ENCODER_QUEUE_MAXSIZE:-240}"
AUTO_RECORD_DATASET_VCODEC="${AUTO_RECORD_DATASET_VCODEC:-h264}"
MAX_RECORD_ATTEMPTS="${MAX_RECORD_ATTEMPTS:-0}"
AUTO_CONFIRM_BEFORE_TAKEOFF="${AUTO_CONFIRM_BEFORE_TAKEOFF:-true}"
TAKEOFF_MODE="${TAKEOFF_MODE:-auto}"
START_STACK="${START_STACK:-true}"
START_PX4CTRL="${START_PX4CTRL:-true}"
PX4CTRL_GRIPPER_RC_CHANNEL="${PX4CTRL_GRIPPER_RC_CHANNEL:-0}"
PX4CTRL_TAKEOFF_LAND_SPEED="${PX4CTRL_TAKEOFF_LAND_SPEED:-0.35}"
PX4CTRL_TAKEOFF_HEIGHT="${PX4CTRL_TAKEOFF_HEIGHT:-0.65}"
STACK_STARTUP_WAIT_S="${STACK_STARTUP_WAIT_S:-8}"
PX4CTRL_USE_BODYRATE_CTRL="${PX4CTRL_USE_BODYRATE_CTRL:-}"
PX4CTRL_UDE_ENABLE="${PX4CTRL_UDE_ENABLE:-}"
PX4CTRL_TD_ENABLE="${PX4CTRL_TD_ENABLE:-}"
PX4CTRL_CMD_FEEDFORWARD_ENABLE="${PX4CTRL_CMD_FEEDFORWARD_ENABLE:-}"
PX4CTRL_ATTITUDE_FEEDBACK_MODE="${PX4CTRL_ATTITUDE_FEEDBACK_MODE:-}"
TRAJ_PLANNER_ENABLE="${TRAJ_PLANNER_ENABLE:-false}"
TRAJ_PLANNER_INPUT_TOPIC="${TRAJ_PLANNER_INPUT_TOPIC:-/position_cmd_raw}"
TRAJ_PLANNER_OUTPUT_TOPIC="${TRAJ_PLANNER_OUTPUT_TOPIC:-/position_cmd_traj}"
TRAJ_PLANNER_RATE_HZ="${TRAJ_PLANNER_RATE_HZ:-50.0}"
TRAJ_PLANNER_MAX_SPEED_XY_MPS="${TRAJ_PLANNER_MAX_SPEED_XY_MPS:-${MAX_SPEED:-0.30}}"
TRAJ_PLANNER_MAX_ACCEL_XY_MPS2="${TRAJ_PLANNER_MAX_ACCEL_XY_MPS2:-0.35}"
TRAJ_PLANNER_MAX_SPEED_Z_MPS="${TRAJ_PLANNER_MAX_SPEED_Z_MPS:-0.15}"
TRAJ_PLANNER_MAX_ACCEL_Z_MPS2="${TRAJ_PLANNER_MAX_ACCEL_Z_MPS2:-0.25}"
TRAJ_PLANNER_CENTERING_MAX_SPEED_XY_MPS="${TRAJ_PLANNER_CENTERING_MAX_SPEED_XY_MPS:-0.10}"
TRAJ_PLANNER_CENTERING_MAX_ACCEL_XY_MPS2="${TRAJ_PLANNER_CENTERING_MAX_ACCEL_XY_MPS2:-0.20}"
TRAJ_PLANNER_MAX_YAW_RATE_RADPS="${TRAJ_PLANNER_MAX_YAW_RATE_RADPS:-0.35}"
TRAJ_PLANNER_MAX_YAW_ACCEL_RADPS2="${TRAJ_PLANNER_MAX_YAW_ACCEL_RADPS2:-0.50}"
TRAJ_PLANNER_GOAL_REPLAN_POS_THRESHOLD_M="${TRAJ_PLANNER_GOAL_REPLAN_POS_THRESHOLD_M:-0.005}"
TRAJ_PLANNER_GOAL_REPLAN_YAW_THRESHOLD_RAD="${TRAJ_PLANNER_GOAL_REPLAN_YAW_THRESHOLD_RAD:-0.02}"
TRAJ_PLANNER_GOAL_STALE_TIMEOUT_S="${TRAJ_PLANNER_GOAL_STALE_TIMEOUT_S:-0.5}"
TRAJ_PLANNER_DURATION_SCALE="${TRAJ_PLANNER_DURATION_SCALE:-1.05}"
TRAJ_PLANNER_ZERO_LATERAL_START_DERIVATIVES_FOR_VERTICAL_GOALS="${TRAJ_PLANNER_ZERO_LATERAL_START_DERIVATIVES_FOR_VERTICAL_GOALS:-true}"
TRAJ_PLANNER_VERTICAL_GOAL_XY_THRESHOLD_M="${TRAJ_PLANNER_VERTICAL_GOAL_XY_THRESHOLD_M:-0.03}"
TRAJ_PLANNER_VERTICAL_GOAL_Z_MIN_M="${TRAJ_PLANNER_VERTICAL_GOAL_Z_MIN_M:-0.04}"
TRANSFER_DRONE_Z_M="${TRANSFER_DRONE_Z_M:-0.65}"
TARGET_GRASP_X_BIAS_WAS_SET="${TARGET_GRASP_X_BIAS_M+x}"
TARGET_GRASP_X_BIAS_M="${TARGET_GRASP_X_BIAS_M:--0.02}"
TARGET_PREHOVER_MAP_X_BIAS_WAS_SET="${TARGET_PREHOVER_MAP_X_BIAS_M+x}"
TARGET_PREHOVER_MAP_X_BIAS_M="${TARGET_PREHOVER_MAP_X_BIAS_M:-0.0}"
TARGET_HOVER_MAP_X_BIAS_WAS_SET="${TARGET_HOVER_MAP_X_BIAS_M+x}"
TARGET_HOVER_MAP_X_BIAS_M="${TARGET_HOVER_MAP_X_BIAS_M:-0.0}"
TARGET_GRASP_MAP_X_BIAS_WAS_SET="${TARGET_GRASP_MAP_X_BIAS_M+x}"
TARGET_GRASP_MAP_X_BIAS_M="${TARGET_GRASP_MAP_X_BIAS_M:-0.0}"
RELEASE_RETREAT_UP_M="${RELEASE_RETREAT_UP_M:-0.0}"
RELEASE_RETREAT_FORWARD_M="${RELEASE_RETREAT_FORWARD_M:-0.60}"
RELEASE_RETREAT_FRAME="${RELEASE_RETREAT_FRAME:-map_x}"
POST_RELEASE_HOLD_STOP_S="${POST_RELEASE_HOLD_STOP_S:-0.0}"
AUTO_TAKEOFF_POLICY="${AUTO_TAKEOFF_POLICY:-}"
if [[ -z "${AUTO_TAKEOFF_POLICY}" ]]; then
  case "${POST_RELEASE_HOLD_STOP_S}" in
    0|0.0|0.00|0.000) AUTO_TAKEOFF_POLICY="always" ;;
    *) AUTO_TAKEOFF_POLICY="first_only" ;;
  esac
fi
MAX_SPEED="${MAX_SPEED:-0.30}"
APPROACH_SPEED="${APPROACH_SPEED:-0.05}"
LIFT_SPEED="${LIFT_SPEED:-0.10}"
PAYLOAD_LIFT_SPEED="${PAYLOAD_LIFT_SPEED:-0.10}"
PAYLOAD_TRANSFER_SPEED="${PAYLOAD_TRANSFER_SPEED:-0.30}"
POST_GRASP_SETTLE_S="${POST_GRASP_SETTLE_S:-0.10}"
POST_LIFT_SETTLE_S="${POST_LIFT_SETTLE_S:-0.10}"
PRE_GRASP_HOLD_S="${PRE_GRASP_HOLD_S:-0.05}"
RELEASE_AT_BOX_HOVER="${RELEASE_AT_BOX_HOVER:-true}"
RETREAT_SPEED="${RETREAT_SPEED:-0.10}"
GRIPPER_X_OFFSET_M="${GRIPPER_X_OFFSET_M:-0.0}"
GRIPPER_Y_OFFSET_M="${GRIPPER_Y_OFFSET_M:-0.0}"
GRIPPER_Z_OFFSET_M="${GRIPPER_Z_OFFSET_M:-0.25}"
LANDING_MODE="${LANDING_MODE:-auto}"
COMMAND_STOP_BEFORE_LAND_S="${COMMAND_STOP_BEFORE_LAND_S:-0.5}"
AUTO_LAND_TIMEOUT_S="${AUTO_LAND_TIMEOUT_S:-60.0}"
TARGET_YAW_ALIGN_ENABLE="${TARGET_YAW_ALIGN_ENABLE:-true}"
TARGET_YAW_ALIGN_OFFSET_RAD="${TARGET_YAW_ALIGN_OFFSET_RAD:-0.0}"
TARGET_YAW_ALIGN_RATE_DPS="${TARGET_YAW_ALIGN_RATE_DPS:-30.0}"
TARGET_YAW_ALIGN_TOL_DEG="${TARGET_YAW_ALIGN_TOL_DEG:-12.0}"
TARGET_YAW_ALIGN_MIN_DISTANCE_M="${TARGET_YAW_ALIGN_MIN_DISTANCE_M:-0.20}"
TARGET_YAW_ALIGN_MAX_DURATION_S="${TARGET_YAW_ALIGN_MAX_DURATION_S:-5.0}"

TOP_CAMERA_ENABLE="${TOP_CAMERA_ENABLE:-true}"
TOP_CAMERA_TYPE="${TOP_CAMERA_TYPE:-zmq}"
TOP_CAMERA_SERVER="${TOP_CAMERA_SERVER:-10.1.1.35}"
TOP_CAMERA_PORT="${TOP_CAMERA_PORT:-5555}"
TOP_CAMERA_NAME="${TOP_CAMERA_NAME:-top}"
TOP_CAMERA_WIDTH="${TOP_CAMERA_WIDTH:-640}"
TOP_CAMERA_HEIGHT="${TOP_CAMERA_HEIGHT:-480}"
TOP_CAMERA_FPS="${TOP_CAMERA_FPS:-30}"
CAMERA_FPS="${CAMERA_FPS:-30}"
CAMERA_FOURCC="${CAMERA_FOURCC:-MJPG}"
CAMERA_WARMUP_S="${CAMERA_WARMUP_S:-15}"
TOP_CAMERA_WARMUP_S="${TOP_CAMERA_WARMUP_S:-8}"
TOP_CAMERA_TIMEOUT_MS="${TOP_CAMERA_TIMEOUT_MS:-8000}"
SIDE_CAMERA_ENABLE="${SIDE_CAMERA_ENABLE:-false}"
SIDE_CAMERA_TYPE="${SIDE_CAMERA_TYPE:-zmq}"
SIDE_CAMERA_SERVER="${SIDE_CAMERA_SERVER:-${TOP_CAMERA_SERVER}}"
SIDE_CAMERA_PORT="${SIDE_CAMERA_PORT:-5555}"
SIDE_CAMERA_NAME="${SIDE_CAMERA_NAME:-side}"
SIDE_CAMERA_WIDTH="${SIDE_CAMERA_WIDTH:-${TOP_CAMERA_WIDTH}}"
SIDE_CAMERA_HEIGHT="${SIDE_CAMERA_HEIGHT:-${TOP_CAMERA_HEIGHT}}"
SIDE_CAMERA_FPS="${SIDE_CAMERA_FPS:-${TOP_CAMERA_FPS}}"
SIDE_CAMERA_WARMUP_S="${SIDE_CAMERA_WARMUP_S:-${TOP_CAMERA_WARMUP_S}}"
SIDE_CAMERA_TIMEOUT_MS="${SIDE_CAMERA_TIMEOUT_MS:-${TOP_CAMERA_TIMEOUT_MS}}"
TELEOP_STARTUP_TIMEOUT_S="${TELEOP_STARTUP_TIMEOUT_S:-5.0}"
MAVLINK_STREAM_RATE_CONFIG="${MAVLINK_STREAM_RATE_CONFIG:-false}"
STACK_LOG_DIR="${STACK_LOG_DIR:-${WORKSPACE_DIR}/log}"
STACK_LOG_FILE="${STACK_LOG_FILE:-${STACK_LOG_DIR}/auto_record_hls_ude_stack_${AUTO_RECORD_RUN_ID}.log}"
CMD_TRACE_ENABLE="${CMD_TRACE_ENABLE:-false}"
CMD_TRACE_DIR="${CMD_TRACE_DIR:-${STACK_LOG_DIR}/cmd_trace_${AUTO_RECORD_RUN_ID}}"
CMD_TRACE_LOG_FILE="${CMD_TRACE_LOG_FILE:-${CMD_TRACE_DIR}.log}"
CMD_TRACE_TOPICS="${CMD_TRACE_TOPICS:-/auto_hls_grasp_place/nominal_position_cmd /auto_hls_grasp_place/corrected_position_cmd /auto_hls_grasp_place/mocap_correction /auto_hls_grasp_place/body_frame_correction /auto_hls_grasp_place/arrival_error /auto_hls_grasp_place/hls_body_y_offset /position_cmd_raw /position_cmd_traj /mavros/vision_pose/pose /px4ctrl/state /gripper/feedback /hls_gripper/status_snapshot}"

GRIPPER_TOPIC="${GRIPPER_TOPIC:-/gripper/command}"
GRIPPER_COMMAND_PAIR_TOPIC="${GRIPPER_COMMAND_PAIR_TOPIC:-/gripper/command_pair}"
GRIPPER_FEEDBACK_TOPIC="${GRIPPER_FEEDBACK_TOPIC:-/gripper/feedback}"
GRIPPER_FEEDBACK_TIMEOUT_S="${GRIPPER_FEEDBACK_TIMEOUT_S:-0.5}"
CHILD_CLEANUP_TIMEOUT_S="${CHILD_CLEANUP_TIMEOUT_S:-5}"
AUTO_CLEANUP_TIMEOUT_S="${AUTO_CLEANUP_TIMEOUT_S:-8}"
STACK_CLEANUP_TIMEOUT_S="${STACK_CLEANUP_TIMEOUT_S:-8}"
RECORD_CLEANUP_TIMEOUT_S="${RECORD_CLEANUP_TIMEOUT_S:-90}"
RECORD_TERM_TIMEOUT_S="${RECORD_TERM_TIMEOUT_S:-10}"
KILL_STALE_RECORDERS_ON_START="${KILL_STALE_RECORDERS_ON_START:-true}"
STALE_RECORDER_STOP_SIGNAL="${STALE_RECORDER_STOP_SIGNAL:-INT}"
KILL_STALE_STACKS_ON_START="${KILL_STALE_STACKS_ON_START:-true}"
STALE_STACK_TERM_WAIT_S="${STALE_STACK_TERM_WAIT_S:-2}"

RECORD_PID=""
AUTO_PID=""
STACK_PID=""
CMD_TRACE_PID=""
CLEANUP_STARTED=false

bool_is_true() {
  case "${1,,}" in
    1|true|yes|y|on) return 0 ;;
    *) return 1 ;;
  esac
}

if bool_is_true "${TRAJ_PLANNER_ENABLE}"; then
  PX4CTRL_TD_ENABLE="${PX4CTRL_TD_ENABLE:-false}"
  PX4CTRL_CMD_FEEDFORWARD_ENABLE="${PX4CTRL_CMD_FEEDFORWARD_ENABLE:-true}"
  if [[ -z "${TARGET_GRASP_X_BIAS_WAS_SET}" ]]; then
    TARGET_GRASP_X_BIAS_M="0.0"
  fi
  if [[ -z "${TARGET_PREHOVER_MAP_X_BIAS_WAS_SET}" ]]; then
    TARGET_PREHOVER_MAP_X_BIAS_M="-0.16"
  fi
  if [[ -z "${TARGET_HOVER_MAP_X_BIAS_WAS_SET}" ]]; then
    TARGET_HOVER_MAP_X_BIAS_M="-0.06"
  fi
  if [[ -z "${TARGET_GRASP_MAP_X_BIAS_WAS_SET}" ]]; then
    TARGET_GRASP_MAP_X_BIAS_M="-0.06"
  fi
fi

takeoff_mode_for_attempt() {
  local attempt="$1"
  case "${AUTO_TAKEOFF_POLICY}" in
    always)
      printf '%s\n' "${TAKEOFF_MODE}"
      ;;
    first_only)
      if (( attempt <= 1 )); then
        printf '%s\n' "${TAKEOFF_MODE}"
      else
        printf 'manual\n'
      fi
      ;;
    manual|never)
      printf 'manual\n'
      ;;
    *)
      echo "[auto-record-hls-ude] ERROR: AUTO_TAKEOFF_POLICY must be always, first_only, manual, or never; got '${AUTO_TAKEOFF_POLICY}'." >&2
      exit 2
      ;;
  esac
}

wait_for_takeoff_confirmation() {
  local attempt="$1"
  local takeoff_mode="$2"
  if ! bool_is_true "${AUTO_CONFIRM_BEFORE_TAKEOFF}"; then
    return
  fi
  if [[ ! -t 0 ]]; then
    echo "[auto-record-hls-ude] ERROR: AUTO_CONFIRM_BEFORE_TAKEOFF=true requires an interactive terminal." >&2
    echo "[auto-record-hls-ude] Run from a terminal or set AUTO_CONFIRM_BEFORE_TAKEOFF=false." >&2
    exit 2
  fi

  echo
  echo "[auto-record-hls-ude] Recorder is waiting at the episode gate."
  if [[ "${takeoff_mode}" == "auto" ]]; then
    echo "[auto-record-hls-ude] Reset the field, keep CH5/CH6 high, then press Enter to start attempt ${attempt}."
    read -r -p "Press Enter to start automatic takeoff for attempt ${attempt}: " _
  else
    echo "[auto-record-hls-ude] Keep the vehicle in AUTO_HOVER, reset the field, then press Enter to start in-air attempt ${attempt}."
    read -r -p "Press Enter to start in-air CMD_CTRL attempt ${attempt}: " _
  fi
}

kill_stale_recorders() {
  if ! bool_is_true "${KILL_STALE_RECORDERS_ON_START}"; then
    return
  fi

  local patterns=(
    "${SCRIPT_DIR}/record_vla_dataset.sh"
    "lerobot-record"
  )
  local pattern pids pid pgid
  for pattern in "${patterns[@]}"; do
    pids="$(pgrep -f "${pattern}" || true)"
    if [[ -z "${pids}" ]]; then
      continue
    fi
    echo "[auto-record-hls-ude] stopping stale recorder process(es) before new run with ${STALE_RECORDER_STOP_SIGNAL}: ${pattern}"
    echo "${pids}" | sed 's/^/[auto-record-hls-ude]   stale pid /'
    for pid in ${pids}; do
      pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
      if [[ -n "${pgid}" ]]; then
        kill "-${STALE_RECORDER_STOP_SIGNAL}" -- "-${pgid}" 2>/dev/null || true
      fi
      kill "-${STALE_RECORDER_STOP_SIGNAL}" "${pid}" 2>/dev/null || true
    done
  done
}

kill_pids_for_pattern() {
  local signal="$1"
  local label="$2"
  local pattern="$3"
  local pids pid pgid
  pids="$(pgrep -f "${pattern}" || true)"
  if [[ -z "${pids}" ]]; then
    return
  fi

  echo "[auto-record-hls-ude] stopping stale ${label} with ${signal}: ${pattern}"
  echo "${pids}" | sed 's/^/[auto-record-hls-ude]   stale pid /'
  for pid in ${pids}; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
    if [[ -n "${pgid}" ]]; then
      kill "-${signal}" -- "-${pgid}" 2>/dev/null || true
    fi
    kill "-${signal}" "${pid}" 2>/dev/null || true
  done
}

kill_stale_stacks() {
  if ! bool_is_true "${START_STACK}" || ! bool_is_true "${KILL_STALE_STACKS_ON_START}"; then
    return
  fi

  local patterns=(
    "${SCRIPT_DIR}/run_mocap_mavros.sh"
    "${WORKSPACE_DIR}/src/px4ctrl/scripts/vrpn_to_mavros_vision_bridge.py"
    "${WORKSPACE_DIR}/install/px4ctrl/lib/px4ctrl/vrpn_to_mavros_vision_bridge.py"
    "ros2 run px4ctrl px4ctrl_node"
    "/px4ctrl/px4ctrl_node"
    "ros2 run px4ctrl trajectory_planner_node"
    "/px4ctrl/trajectory_planner_node"
    "ros2 launch mavros node.launch"
    "/mavros/mavros_node"
    "ros2 run vrpn_mocap client_node"
    "/vrpn_mocap/client_node"
  )

  local pattern
  for pattern in "${patterns[@]}"; do
    kill_pids_for_pattern TERM "stack process(es)" "${pattern}"
  done

  sleep "${STALE_STACK_TERM_WAIT_S}"

  for pattern in "${patterns[@]}"; do
    kill_pids_for_pattern KILL "stack process(es)" "${pattern}"
  done
}

process_group_alive() {
  local pid="$1"
  [[ -n "${pid}" ]] && (kill -0 -- "-${pid}" 2>/dev/null || kill -0 "${pid}" 2>/dev/null)
}

signal_process_group() {
  local signal="$1"
  local pid="$2"
  kill "-${signal}" -- "-${pid}" 2>/dev/null || kill "-${signal}" "${pid}" 2>/dev/null || true
}

wait_for_process_group_exit() {
  local pid="$1"
  local timeout_s="$2"
  local timeout_i="${timeout_s%.*}"
  if [[ -z "${timeout_i}" ]] || (( timeout_i < 1 )); then
    timeout_i=1
  fi
  local deadline=$((SECONDS + timeout_i))
  while (( SECONDS < deadline )); do
    if ! process_group_alive "${pid}"; then
      return 0
    fi
    sleep 0.2
  done
  ! process_group_alive "${pid}"
}

stop_process_group() {
  local label="$1"
  local pid="$2"
  local first_signal="${3:-TERM}"
  local timeout_s="${4:-${CHILD_CLEANUP_TIMEOUT_S}}"
  if [[ -z "${pid}" ]]; then
    return
  fi
  if process_group_alive "${pid}"; then
    echo "[auto-record-hls-ude] stopping ${label} process group ${pid} with ${first_signal}"
    signal_process_group "${first_signal}" "${pid}"
    wait_for_process_group_exit "${pid}" "${timeout_s}" || true
    if process_group_alive "${pid}"; then
      echo "[auto-record-hls-ude] ${label} did not exit after ${timeout_s}s; sending KILL to process group ${pid}"
      signal_process_group KILL "${pid}"
    fi
    wait "${pid}" 2>/dev/null || true
  fi
}

stop_recorder_process_group() {
  local pid="$1"
  if [[ -z "${pid}" ]]; then
    return
  fi
  if process_group_alive "${pid}"; then
    echo "[auto-record-hls-ude] asking LeRobot recorder process group ${pid} to stop gracefully with INT"
    signal_process_group INT "${pid}"
    wait_for_process_group_exit "${pid}" "${RECORD_CLEANUP_TIMEOUT_S}" || true
    if process_group_alive "${pid}"; then
      echo "[auto-record-hls-ude] recorder still alive after ${RECORD_CLEANUP_TIMEOUT_S}s; sending TERM"
      signal_process_group TERM "${pid}"
      wait_for_process_group_exit "${pid}" "${RECORD_TERM_TIMEOUT_S}" || true
    fi
    if process_group_alive "${pid}"; then
      echo "[auto-record-hls-ude] recorder still alive after TERM; sending KILL"
      signal_process_group KILL "${pid}"
    fi
    wait "${pid}" 2>/dev/null || true
  fi
}

start_cmd_trace_once() {
  if ! bool_is_true "${CMD_TRACE_ENABLE}"; then
    return
  fi
  if [[ -n "${CMD_TRACE_PID}" ]] && process_group_alive "${CMD_TRACE_PID}"; then
    return
  fi

  mkdir -p "$(dirname "${CMD_TRACE_DIR}")"
  echo "[auto-record-hls-ude] starting command trace rosbag: ${CMD_TRACE_DIR}"
  echo "[auto-record-hls-ude] command trace log: ${CMD_TRACE_LOG_FILE}"
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1090
  source "${WORKSPACE_DIR}/install/setup.bash"
  set -u
  # shellcheck disable=SC2206
  local topics=( ${CMD_TRACE_TOPICS} )
  setsid ros2 bag record -o "${CMD_TRACE_DIR}" "${topics[@]}" >"${CMD_TRACE_LOG_FILE}" 2>&1 &
  CMD_TRACE_PID="$!"
  echo "[auto-record-hls-ude] command trace process group: ${CMD_TRACE_PID}"
}

cleanup_all() {
  if [[ "${CLEANUP_STARTED}" == "true" ]]; then
    return
  fi
  CLEANUP_STARTED=true
  trap - EXIT INT TERM HUP

  stop_process_group "automatic HLS-UDE flight" "${AUTO_PID}" TERM "${AUTO_CLEANUP_TIMEOUT_S}"
  AUTO_PID=""
  stop_process_group "command trace rosbag" "${CMD_TRACE_PID}" INT "${CHILD_CLEANUP_TIMEOUT_S}"
  CMD_TRACE_PID=""
  stop_recorder_process_group "${RECORD_PID}"
  RECORD_PID=""
  stop_process_group "mocap/MAVROS/bridge/px4ctrl stack" "${STACK_PID}" TERM "${STACK_CLEANUP_TIMEOUT_S}"
  STACK_PID=""
}

on_signal() {
  cleanup_all
  exit 130
}

start_stack_once() {
  if ! bool_is_true "${START_STACK}"; then
    echo "[auto-record-hls-ude] START_STACK=false; expecting mocap/MAVROS/px4ctrl stack to already be running."
    return
  fi

  mkdir -p "${STACK_LOG_DIR}"
  echo "[auto-record-hls-ude] starting mocap/MAVROS/bridge/px4ctrl stack once"
  echo "[auto-record-hls-ude] stack output is redirected to: ${STACK_LOG_FILE}"
  START_PX4CTRL="${START_PX4CTRL}" \
  PX4CTRL_GRIPPER_RC_CHANNEL="${PX4CTRL_GRIPPER_RC_CHANNEL}" \
  PX4CTRL_TAKEOFF_LAND_SPEED="${PX4CTRL_TAKEOFF_LAND_SPEED}" \
  PX4CTRL_TAKEOFF_HEIGHT="${PX4CTRL_TAKEOFF_HEIGHT}" \
  PX4CTRL_USE_BODYRATE_CTRL="${PX4CTRL_USE_BODYRATE_CTRL}" \
  PX4CTRL_UDE_ENABLE="${PX4CTRL_UDE_ENABLE}" \
  PX4CTRL_TD_ENABLE="${PX4CTRL_TD_ENABLE}" \
  PX4CTRL_CMD_FEEDFORWARD_ENABLE="${PX4CTRL_CMD_FEEDFORWARD_ENABLE}" \
  PX4CTRL_ATTITUDE_FEEDBACK_MODE="${PX4CTRL_ATTITUDE_FEEDBACK_MODE}" \
  TRAJ_PLANNER_ENABLE="${TRAJ_PLANNER_ENABLE}" \
  TRAJ_PLANNER_INPUT_TOPIC="${TRAJ_PLANNER_INPUT_TOPIC}" \
  TRAJ_PLANNER_OUTPUT_TOPIC="${TRAJ_PLANNER_OUTPUT_TOPIC}" \
  TRAJ_PLANNER_RATE_HZ="${TRAJ_PLANNER_RATE_HZ}" \
  TRAJ_PLANNER_MAX_SPEED_XY_MPS="${TRAJ_PLANNER_MAX_SPEED_XY_MPS}" \
  TRAJ_PLANNER_MAX_ACCEL_XY_MPS2="${TRAJ_PLANNER_MAX_ACCEL_XY_MPS2}" \
  TRAJ_PLANNER_MAX_SPEED_Z_MPS="${TRAJ_PLANNER_MAX_SPEED_Z_MPS}" \
  TRAJ_PLANNER_MAX_ACCEL_Z_MPS2="${TRAJ_PLANNER_MAX_ACCEL_Z_MPS2}" \
  TRAJ_PLANNER_CENTERING_MAX_SPEED_XY_MPS="${TRAJ_PLANNER_CENTERING_MAX_SPEED_XY_MPS}" \
  TRAJ_PLANNER_CENTERING_MAX_ACCEL_XY_MPS2="${TRAJ_PLANNER_CENTERING_MAX_ACCEL_XY_MPS2}" \
  TRAJ_PLANNER_MAX_YAW_RATE_RADPS="${TRAJ_PLANNER_MAX_YAW_RATE_RADPS}" \
  TRAJ_PLANNER_MAX_YAW_ACCEL_RADPS2="${TRAJ_PLANNER_MAX_YAW_ACCEL_RADPS2}" \
  TRAJ_PLANNER_GOAL_REPLAN_POS_THRESHOLD_M="${TRAJ_PLANNER_GOAL_REPLAN_POS_THRESHOLD_M}" \
  TRAJ_PLANNER_GOAL_REPLAN_YAW_THRESHOLD_RAD="${TRAJ_PLANNER_GOAL_REPLAN_YAW_THRESHOLD_RAD}" \
  TRAJ_PLANNER_GOAL_STALE_TIMEOUT_S="${TRAJ_PLANNER_GOAL_STALE_TIMEOUT_S}" \
  TRAJ_PLANNER_DURATION_SCALE="${TRAJ_PLANNER_DURATION_SCALE}" \
  TRAJ_PLANNER_ZERO_LATERAL_START_DERIVATIVES_FOR_VERTICAL_GOALS="${TRAJ_PLANNER_ZERO_LATERAL_START_DERIVATIVES_FOR_VERTICAL_GOALS}" \
  TRAJ_PLANNER_VERTICAL_GOAL_XY_THRESHOLD_M="${TRAJ_PLANNER_VERTICAL_GOAL_XY_THRESHOLD_M}" \
  TRAJ_PLANNER_VERTICAL_GOAL_Z_MIN_M="${TRAJ_PLANNER_VERTICAL_GOAL_Z_MIN_M}" \
  MAVLINK_STREAM_RATE_CONFIG="${MAVLINK_STREAM_RATE_CONFIG}" \
  setsid bash "${SCRIPT_DIR}/run_mocap_mavros.sh" >"${STACK_LOG_FILE}" 2>&1 &
  STACK_PID="$!"
  echo "[auto-record-hls-ude] stack process group: ${STACK_PID}"
  sleep "${STACK_STARTUP_WAIT_S}"
  start_cmd_trace_once
}

wait_for_recorder_ready() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1090
  source "${WORKSPACE_DIR}/install/setup.bash"
  set -u

  RECORD_PID="${RECORD_PID}" \
  RECORD_STATUS_TOPIC="${RECORD_STATUS_TOPIC}" \
  RECORD_READY_TIMEOUT_S="${RECORD_READY_TIMEOUT_S}" \
  RECORD_READY_LOG_PERIOD_S="${RECORD_READY_LOG_PERIOD_S}" \
  python3 - <<'PY'
import os
import signal
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


pid = int(os.environ["RECORD_PID"])
topic = os.environ["RECORD_STATUS_TOPIC"]
timeout_s = max(1.0, float(os.environ["RECORD_READY_TIMEOUT_S"]))
log_period_s = max(0.0, float(os.environ.get("RECORD_READY_LOG_PERIOD_S", "0.0")))
latest = None
deadline = time.monotonic() + timeout_s
next_log_s = time.monotonic() + log_period_s if log_period_s > 0.0 else float("inf")


def stop_recorder_process_group() -> None:
    try:
        os.killpg(pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    except PermissionError:
        pass
    except Exception:
        try:
            os.kill(pid, signal.SIGINT)
        except Exception:
            pass


def signal_handler(signum, _frame) -> None:
    print(
        f"[auto-record-hls-ude] interrupted while waiting recorder ready; "
        f"stopping recorder process group {pid}.",
        file=sys.stderr,
    )
    stop_recorder_process_group()
    sys.exit(128 + int(signum))


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def process_state(pid_value: int) -> str | None:
    try:
        with open(f"/proc/{pid_value}/stat", "r", encoding="utf-8") as f:
            fields = f.read().split()
    except FileNotFoundError:
        return None
    if len(fields) < 3:
        return None
    return fields[2]


def callback(msg: String) -> None:
    global latest
    latest = str(msg.data)


rclpy.init(args=None)
node = rclpy.create_node("auto_record_hls_ude_wait_record_ready")
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
node.create_subscription(String, topic, callback, qos)
executor = SingleThreadedExecutor()
executor.add_node(node)

try:
    while rclpy.ok() and time.monotonic() < deadline:
        state = process_state(pid)
        if state is None or state == "Z":
            print(
                f"[auto-record-hls-ude] recorder process exited before ready; "
                f"pid={pid} state={state!r} latest_status={latest!r}",
                file=sys.stderr,
            )
            sys.exit(2)

        executor.spin_once(timeout_sec=0.1)
        if latest == "WAITING_GATE":
            print(f"[auto-record-hls-ude] recorder ready: {topic}=WAITING_GATE")
            sys.exit(0)

        now = time.monotonic()
        if now >= next_log_s:
            print(
                f"[auto-record-hls-ude] waiting recorder ready: latest {topic}={latest!r}, "
                f"remaining={max(0.0, deadline - now):.1f}s"
            )
            next_log_s = now + log_period_s

    print(
        f"[auto-record-hls-ude] timed out waiting for {topic}=WAITING_GATE; "
        f"latest={latest!r}",
        file=sys.stderr,
    )
    sys.exit(3)
finally:
    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
}

wait_for_recorder_finish_after_auto() {
  if ! bool_is_true "${ENABLE_RECORD}" || [[ -z "${RECORD_PID}" ]]; then
    return 0
  fi
  if ! process_group_alive "${RECORD_PID}"; then
    RECORD_PID=""
    return 0
  fi
  if ! bool_is_true "${WAIT_RECORD_ON_AUTO_EXIT}"; then
    return 1
  fi

  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1090
  source "${WORKSPACE_DIR}/install/setup.bash"
  set -u

  RECORD_PID="${RECORD_PID}" \
  RECORD_STATUS_TOPIC="${RECORD_STATUS_TOPIC}" \
  RECORD_FINISH_TIMEOUT_S="${RECORD_FINISH_TIMEOUT_S}" \
  RECORD_GATE_AFTER_AUTO_GRACE_S="${RECORD_GATE_AFTER_AUTO_GRACE_S}" \
  EPISODE_TIME_S="${EPISODE_TIME_S}" \
  python3 - <<'PY'
import os
import signal
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


pid = int(os.environ["RECORD_PID"])
topic = os.environ["RECORD_STATUS_TOPIC"]
episode_time_s = max(1.0, float(os.environ["EPISODE_TIME_S"]))
finish_timeout_raw = os.environ.get("RECORD_FINISH_TIMEOUT_S", "").strip()
finish_timeout_s = (
    max(1.0, float(finish_timeout_raw))
    if finish_timeout_raw
    else episode_time_s + 240.0
)
gate_grace_s = max(0.0, float(os.environ.get("RECORD_GATE_AFTER_AUTO_GRACE_S", "3.0")))
latest = None
seen_recording = False
seen_saving = False
seen_episode_done = False
started_s = time.monotonic()
deadline_s = started_s + finish_timeout_s
next_prompt_s = started_s + 2.0


def stop_recorder_process_group() -> None:
    try:
        os.killpg(pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    except PermissionError:
        pass
    except Exception:
        try:
            os.kill(pid, signal.SIGINT)
        except Exception:
            pass


def signal_handler(signum, _frame) -> None:
    print(
        f"[auto-record-hls-ude] interrupted while waiting recorder finish; "
        f"stopping recorder process group {pid}.",
        file=sys.stderr,
    )
    stop_recorder_process_group()
    sys.exit(128 + int(signum))


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def process_state(pid_value: int) -> str | None:
    try:
        with open(f"/proc/{pid_value}/stat", "r", encoding="utf-8") as f:
            fields = f.read().split()
    except FileNotFoundError:
        return None
    if len(fields) < 3:
        return None
    return fields[2]


def callback(msg: String) -> None:
    global latest, seen_recording, seen_saving, seen_episode_done
    latest = str(msg.data)
    if latest == "RECORDING":
        seen_recording = True
    elif latest == "SAVING":
        seen_saving = True
    elif latest == "EPISODE_DONE":
        seen_episode_done = True


print(
    f"[auto-record-hls-ude] automatic process exited; waiting for recorder "
    f"pid={pid} to finish/save or reopen the gate (timeout={finish_timeout_s:.1f}s)."
)
print("[auto-record-hls-ude] Waiting for SAVING -> EPISODE_DONE or the next WAITING_GATE; do not press Ctrl+C while videos are saving.")

rclpy.init(args=None)
node = rclpy.create_node("auto_record_hls_ude_wait_record_finish")
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
node.create_subscription(String, topic, callback, qos)
executor = SingleThreadedExecutor()
executor.add_node(node)

try:
    while rclpy.ok() and time.monotonic() < deadline_s:
        state = process_state(pid)
        if state is None or state == "Z":
            print("[auto-record-hls-ude] recorder process exited; wrapper will collect its exit status.")
            sys.exit(0)

        executor.spin_once(timeout_sec=0.1)
        now_s = time.monotonic()

        if latest == "EPISODE_DONE":
            print("[auto-record-hls-ude] recorder saved the episode; waiting loop will continue to the next gate.")
            sys.exit(0)

        if latest == "SAVING" and now_s >= next_prompt_s:
            print("[auto-record-hls-ude] recorder is SAVING the episode; field reset can start after the vehicle is safe.")
            next_prompt_s = now_s + 5.0

        if latest == "WAITING_GATE":
            if seen_recording or seen_saving:
                print("[auto-record-hls-ude] recorder returned to WAITING_GATE; next episode can start.")
                sys.exit(0)
            if now_s - started_s >= gate_grace_s:
                print(
                    f"[auto-record-hls-ude] recorder is WAITING_GATE after automatic process exit. "
                    f"The previous episode may already be saved, or no episode was started."
                )
                sys.exit(0)

        if latest == "RECORDING" and now_s >= next_prompt_s:
            print("[auto-record-hls-ude] recorder still RECORDING/saving; waiting for EPISODE_DONE or next WAITING_GATE.")
            next_prompt_s = now_s + 5.0

    print(
        f"[auto-record-hls-ude] timed out waiting for recorder to finish; "
        f"latest {topic}={latest!r}.",
        file=sys.stderr,
    )
    sys.exit(3)
finally:
    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
}

trap cleanup_all EXIT
trap on_signal INT TERM HUP

echo "[auto-record-hls-ude] workspace: ${WORKSPACE_DIR}"
echo "[auto-record-hls-ude] record: ${ENABLE_RECORD}"
echo "[auto-record-hls-ude] run id: ${AUTO_RECORD_RUN_ID}"
echo "[auto-record-hls-ude] record gate: status=${RECORD_STATUS_TOPIC} start=${RECORD_GATE_TOPIC}:${RECORD_GATE_VALUE} stop=${RECORD_STOP_TOPIC}:${RECORD_STOP_VALUE} ready_timeout=${RECORD_READY_TIMEOUT_S}s ready_log_period=${RECORD_READY_LOG_PERIOD_S}s wait_on_auto_exit=${WAIT_RECORD_ON_AUTO_EXIT} finish_timeout=${RECORD_FINISH_TIMEOUT_S:-<auto>}s episodes=${NUM_EPISODES} episode_time=${EPISODE_TIME_S}s reset_time=${RESET_TIME_S}s prewarm_steps=${RECORD_PREWARM_STEPS} streaming=${STREAMING_ENCODING} vcodec=${AUTO_RECORD_DATASET_VCODEC} encoder_queue=${ENCODER_QUEUE_MAXSIZE}"
echo "[auto-record-hls-ude] automatic takeoff confirmation: ${AUTO_CONFIRM_BEFORE_TAKEOFF}"
echo "[auto-record-hls-ude] takeoff policy: base_mode=${TAKEOFF_MODE} policy=${AUTO_TAKEOFF_POLICY}"
echo "[auto-record-hls-ude] stack: start=${START_STACK} start_px4ctrl=${START_PX4CTRL} takeoff_height=${PX4CTRL_TAKEOFF_HEIGHT:-<yaml-default>} takeoff_speed=${PX4CTRL_TAKEOFF_LAND_SPEED} log=${STACK_LOG_FILE}"
echo "[auto-record-hls-ude] episode motion: transfer_z=${TRANSFER_DRONE_Z_M} release_retreat_forward=${RELEASE_RETREAT_FORWARD_M} frame=${RELEASE_RETREAT_FRAME} release_up=${RELEASE_RETREAT_UP_M} post_release_hold_stop=${POST_RELEASE_HOLD_STOP_S}s landing=${LANDING_MODE} auto_land_timeout=${AUTO_LAND_TIMEOUT_S}s"
echo "[auto-record-hls-ude] flight speed: max=${MAX_SPEED} approach=${APPROACH_SPEED} lift=${LIFT_SPEED} payload_lift=${PAYLOAD_LIFT_SPEED} payload_transfer=${PAYLOAD_TRANSFER_SPEED} pre_grasp_hold=${PRE_GRASP_HOLD_S}s post_grasp=${POST_GRASP_SETTLE_S}s post_lift=${POST_LIFT_SETTLE_S}s retreat=${RETREAT_SPEED}"
echo "[auto-record-hls-ude] release timing: release_at_box_hover=${RELEASE_AT_BOX_HOVER}"
echo "[auto-record-hls-ude] trajectory planner: enable=${TRAJ_PLANNER_ENABLE} raw=${TRAJ_PLANNER_INPUT_TOPIC} traj=${TRAJ_PLANNER_OUTPUT_TOPIC} td=${PX4CTRL_TD_ENABLE:-<yaml-default>} feedforward=${PX4CTRL_CMD_FEEDFORWARD_ENABLE:-<yaml-default>}"
echo "[auto-record-hls-ude] target bias: body_grasp_x=${TARGET_GRASP_X_BIAS_M} map_prehover_x=${TARGET_PREHOVER_MAP_X_BIAS_M} map_hover_x=${TARGET_HOVER_MAP_X_BIAS_M} map_grasp_x=${TARGET_GRASP_MAP_X_BIAS_M}"
echo "[auto-record-hls-ude] gripper body-frame offset: x=${GRIPPER_X_OFFSET_M}m forward y=${GRIPPER_Y_OFFSET_M}m left z=${GRIPPER_Z_OFFSET_M}m down"
echo "[auto-record-hls-ude] target yaw align: enable=${TARGET_YAW_ALIGN_ENABLE} offset=${TARGET_YAW_ALIGN_OFFSET_RAD}rad rate=${TARGET_YAW_ALIGN_RATE_DPS}deg/s tol=${TARGET_YAW_ALIGN_TOL_DEG}deg min_distance=${TARGET_YAW_ALIGN_MIN_DISTANCE_M}m max_duration=${TARGET_YAW_ALIGN_MAX_DURATION_S}s"
echo "[auto-record-hls-ude] top camera: enable=${TOP_CAMERA_ENABLE} type=${TOP_CAMERA_TYPE} server=${TOP_CAMERA_SERVER}:${TOP_CAMERA_PORT} name=${TOP_CAMERA_NAME} warmup=${TOP_CAMERA_WARMUP_S}s timeout=${TOP_CAMERA_TIMEOUT_MS}ms"
echo "[auto-record-hls-ude] side camera: enable=${SIDE_CAMERA_ENABLE} type=${SIDE_CAMERA_TYPE} server=${SIDE_CAMERA_SERVER}:${SIDE_CAMERA_PORT} name=${SIDE_CAMERA_NAME} warmup=${SIDE_CAMERA_WARMUP_S}s timeout=${SIDE_CAMERA_TIMEOUT_MS}ms"
echo "[auto-record-hls-ude] nx cameras: warmup=${CAMERA_WARMUP_S}s fourcc=${CAMERA_FOURCC}"
echo "[auto-record-hls-ude] mavlink stream rate config: ${MAVLINK_STREAM_RATE_CONFIG}"

if bool_is_true "${ENABLE_RECORD}"; then
  kill_stale_recorders
fi
kill_stale_stacks
start_stack_once

if bool_is_true "${ENABLE_RECORD}"; then
  START_GATE_TOPIC="${RECORD_GATE_TOPIC}" \
  START_GATE_VALUE="${RECORD_GATE_VALUE}" \
  STOP_GATE_TOPIC="${RECORD_STOP_TOPIC}" \
  STOP_GATE_VALUE="${RECORD_STOP_VALUE}" \
  START_GATE_STABLE_S=0.0 \
  RECORD_PREWARM_STEPS="${RECORD_PREWARM_STEPS}" \
  DATASET_VCODEC="${AUTO_RECORD_DATASET_VCODEC}" \
  STREAMING_ENCODING="${STREAMING_ENCODING}" \
  ENCODER_THREADS="${ENCODER_THREADS}" \
  ENCODER_QUEUE_MAXSIZE="${ENCODER_QUEUE_MAXSIZE}" \
  START_GATE_STATUS_LOG_S="${START_GATE_STATUS_LOG_S:-0.0}" \
  DATASET_STATUS_TOPIC="${RECORD_STATUS_TOPIC}" \
  CAMERA_FPS="${CAMERA_FPS}" \
  CAMERA_FOURCC="${CAMERA_FOURCC}" \
  CAMERA_WARMUP_S="${CAMERA_WARMUP_S}" \
  USE_ROS_GRIPPER=true \
  SAFE_OPEN_GRIPPER_AFTER_EPISODE="${SAFE_OPEN_GRIPPER_AFTER_EPISODE:-false}" \
  SAFE_OPEN_GRIPPER_ON_DISCONNECT="${SAFE_OPEN_GRIPPER_ON_DISCONNECT:-false}" \
  GRIPPER_TOPIC="${GRIPPER_TOPIC}" \
  GRIPPER_COMMAND_PAIR_TOPIC="${GRIPPER_COMMAND_PAIR_TOPIC}" \
  GRIPPER_FEEDBACK_TOPIC="${GRIPPER_FEEDBACK_TOPIC}" \
  GRIPPER_FEEDBACK_TIMEOUT_S="${GRIPPER_FEEDBACK_TIMEOUT_S}" \
  NUM_EPISODES="${NUM_EPISODES}" \
  EPISODE_TIME_S="${EPISODE_TIME_S}" \
  RESET_TIME_S="${RESET_TIME_S}" \
  TOP_CAMERA_ENABLE="${TOP_CAMERA_ENABLE}" \
  TOP_CAMERA_TYPE="${TOP_CAMERA_TYPE}" \
  TOP_CAMERA_SERVER="${TOP_CAMERA_SERVER}" \
  TOP_CAMERA_PORT="${TOP_CAMERA_PORT}" \
  TOP_CAMERA_NAME="${TOP_CAMERA_NAME}" \
  TOP_CAMERA_WIDTH="${TOP_CAMERA_WIDTH}" \
  TOP_CAMERA_HEIGHT="${TOP_CAMERA_HEIGHT}" \
  TOP_CAMERA_FPS="${TOP_CAMERA_FPS}" \
  TOP_CAMERA_WARMUP_S="${TOP_CAMERA_WARMUP_S}" \
  TOP_CAMERA_TIMEOUT_MS="${TOP_CAMERA_TIMEOUT_MS}" \
  SIDE_CAMERA_ENABLE="${SIDE_CAMERA_ENABLE}" \
  SIDE_CAMERA_TYPE="${SIDE_CAMERA_TYPE}" \
  SIDE_CAMERA_SERVER="${SIDE_CAMERA_SERVER}" \
  SIDE_CAMERA_PORT="${SIDE_CAMERA_PORT}" \
  SIDE_CAMERA_NAME="${SIDE_CAMERA_NAME}" \
  SIDE_CAMERA_WIDTH="${SIDE_CAMERA_WIDTH}" \
  SIDE_CAMERA_HEIGHT="${SIDE_CAMERA_HEIGHT}" \
  SIDE_CAMERA_FPS="${SIDE_CAMERA_FPS}" \
  SIDE_CAMERA_WARMUP_S="${SIDE_CAMERA_WARMUP_S}" \
  SIDE_CAMERA_TIMEOUT_MS="${SIDE_CAMERA_TIMEOUT_MS}" \
  TELEOP_STARTUP_TIMEOUT_S="${TELEOP_STARTUP_TIMEOUT_S}" \
  TASK="${TASK:-Pick up the yellow paper roll from the black platform and place it into the white box}" \
  setsid bash "${SCRIPT_DIR}/record_vla_dataset.sh" &
  RECORD_PID="$!"
  echo "[auto-record-hls-ude] record process group: ${RECORD_PID}"
  wait_for_recorder_ready
fi

AUTO_STATUS=0
attempt=1
while true; do
  if bool_is_true "${ENABLE_RECORD}"; then
    if [[ -z "${RECORD_PID}" ]] || ! process_group_alive "${RECORD_PID}"; then
      break
    fi
    wait_for_recorder_ready
  elif (( attempt > 1 )); then
    break
  fi

  if [[ "${MAX_RECORD_ATTEMPTS}" != "0" ]] && (( attempt > MAX_RECORD_ATTEMPTS )); then
    echo "[auto-record-hls-ude] ERROR: max attempts reached: ${MAX_RECORD_ATTEMPTS}" >&2
    AUTO_STATUS=3
    break
  fi

  ATTEMPT_TAKEOFF_MODE="$(takeoff_mode_for_attempt "${attempt}")"
  echo "[auto-record-hls-ude] attempt ${attempt} takeoff mode: ${ATTEMPT_TAKEOFF_MODE}"
  wait_for_takeoff_confirmation "${attempt}" "${ATTEMPT_TAKEOFF_MODE}"

  set +e
  ENABLE_RECORD="${ENABLE_RECORD}" \
  RECORD_STATUS_TOPIC="${RECORD_STATUS_TOPIC}" \
  RECORD_GATE_TOPIC="${RECORD_GATE_TOPIC}" \
  RECORD_GATE_VALUE="${RECORD_GATE_VALUE}" \
  RECORD_STOP_TOPIC="${RECORD_STOP_TOPIC}" \
  RECORD_STOP_VALUE="${RECORD_STOP_VALUE}" \
  RECORD_STOP_HOLD_S="${RECORD_STOP_HOLD_S}" \
  RECORD_READY_TIMEOUT_S="${RECORD_READY_TIMEOUT_S}" \
  RECORD_DURATION_S="${EPISODE_TIME_S}" \
  RECORD_START_HOLD_S="${RECORD_START_HOLD_S}" \
  START_STACK=false \
  START_PX4CTRL=false \
  CONFIRM_BEFORE_TAKEOFF=false \
  TAKEOFF_MODE="${ATTEMPT_TAKEOFF_MODE}" \
  CLEANUP_STACK_ON_EXIT=false \
  TRAJ_PLANNER_ENABLE="${TRAJ_PLANNER_ENABLE}" \
  TRAJ_PLANNER_INPUT_TOPIC="${TRAJ_PLANNER_INPUT_TOPIC}" \
  TRAJ_PLANNER_OUTPUT_TOPIC="${TRAJ_PLANNER_OUTPUT_TOPIC}" \
  TARGET_YAW_ALIGN_ENABLE="${TARGET_YAW_ALIGN_ENABLE}" \
  TARGET_YAW_ALIGN_OFFSET_RAD="${TARGET_YAW_ALIGN_OFFSET_RAD}" \
  TARGET_YAW_ALIGN_RATE_DPS="${TARGET_YAW_ALIGN_RATE_DPS}" \
  TARGET_YAW_ALIGN_TOL_DEG="${TARGET_YAW_ALIGN_TOL_DEG}" \
  TARGET_YAW_ALIGN_MIN_DISTANCE_M="${TARGET_YAW_ALIGN_MIN_DISTANCE_M}" \
  TARGET_YAW_ALIGN_MAX_DURATION_S="${TARGET_YAW_ALIGN_MAX_DURATION_S}" \
  MAX_SPEED="${MAX_SPEED}" \
  APPROACH_SPEED="${APPROACH_SPEED}" \
  LIFT_SPEED="${LIFT_SPEED}" \
  PAYLOAD_LIFT_SPEED="${PAYLOAD_LIFT_SPEED}" \
  PAYLOAD_TRANSFER_SPEED="${PAYLOAD_TRANSFER_SPEED}" \
  POST_GRASP_SETTLE_S="${POST_GRASP_SETTLE_S}" \
  POST_LIFT_SETTLE_S="${POST_LIFT_SETTLE_S}" \
  PRE_GRASP_HOLD_S="${PRE_GRASP_HOLD_S}" \
  RELEASE_AT_BOX_HOVER="${RELEASE_AT_BOX_HOVER}" \
  RETREAT_SPEED="${RETREAT_SPEED}" \
  GRIPPER_X_OFFSET_M="${GRIPPER_X_OFFSET_M}" \
  GRIPPER_Y_OFFSET_M="${GRIPPER_Y_OFFSET_M}" \
  GRIPPER_Z_OFFSET_M="${GRIPPER_Z_OFFSET_M}" \
  TRANSFER_DRONE_Z_M="${TRANSFER_DRONE_Z_M}" \
  TARGET_GRASP_X_BIAS_M="${TARGET_GRASP_X_BIAS_M}" \
  TARGET_PREHOVER_MAP_X_BIAS_M="${TARGET_PREHOVER_MAP_X_BIAS_M}" \
  TARGET_HOVER_MAP_X_BIAS_M="${TARGET_HOVER_MAP_X_BIAS_M}" \
  TARGET_GRASP_MAP_X_BIAS_M="${TARGET_GRASP_MAP_X_BIAS_M}" \
  RELEASE_RETREAT_UP_M="${RELEASE_RETREAT_UP_M}" \
  RELEASE_RETREAT_FORWARD_M="${RELEASE_RETREAT_FORWARD_M}" \
  RELEASE_RETREAT_FRAME="${RELEASE_RETREAT_FRAME}" \
  POST_RELEASE_HOLD_STOP_S="${POST_RELEASE_HOLD_STOP_S}" \
  LANDING_MODE="${LANDING_MODE}" \
  COMMAND_STOP_BEFORE_LAND_S="${COMMAND_STOP_BEFORE_LAND_S}" \
  AUTO_LAND_TIMEOUT_S="${AUTO_LAND_TIMEOUT_S}" \
  GRIPPER_TOPIC="${GRIPPER_TOPIC}" \
  GRIPPER_COMMAND_PAIR_TOPIC="${GRIPPER_COMMAND_PAIR_TOPIC}" \
  GRIPPER_FEEDBACK_TOPIC="${GRIPPER_FEEDBACK_TOPIC}" \
  MAVLINK_STREAM_RATE_CONFIG=false \
  setsid bash "${SCRIPT_DIR}/auto_hls_ude_grasp_place_test.sh" &
  AUTO_PID="$!"
  echo "[auto-record-hls-ude] automatic HLS-UDE attempt ${attempt} process group: ${AUTO_PID}"
  wait "${AUTO_PID}"
  episode_status="$?"
  AUTO_PID=""
  set -e

  if bool_is_true "${ENABLE_RECORD}" && [[ -n "${RECORD_PID}" ]]; then
    wait_for_recorder_finish_after_auto || true
  fi

  if [[ "${episode_status}" -ne 0 ]]; then
    echo "[auto-record-hls-ude] ERROR: automatic HLS-UDE attempt ${attempt} exited with status ${episode_status}." >&2
    AUTO_STATUS="${episode_status}"
    break
  fi

  if bool_is_true "${ENABLE_RECORD}" && (( attempt >= NUM_EPISODES )); then
    break
  fi

  attempt=$((attempt + 1))
done

if bool_is_true "${ENABLE_RECORD}" && [[ -n "${RECORD_PID}" ]]; then
  if wait "${RECORD_PID}" 2>/dev/null; then
    echo "[auto-record-hls-ude] recorder exited successfully."
  else
    RECORD_STATUS="$?"
    echo "[auto-record-hls-ude] ERROR: recorder exited with status ${RECORD_STATUS}." >&2
    if [[ "${AUTO_STATUS}" -eq 0 ]]; then
      AUTO_STATUS="${RECORD_STATUS}"
    fi
  fi
  RECORD_PID=""
fi

cleanup_all
exit "${AUTO_STATUS}"
