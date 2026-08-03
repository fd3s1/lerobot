#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${WORKSPACE_DIR}/log"
mkdir -p "${LOG_DIR}"

RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
STACK_LOG_FILE="${STACK_LOG_FILE:-${LOG_DIR}/pi05_inference_stack_${RUN_ID}.log}"
HLS_LOG_FILE="${HLS_LOG_FILE:-${LOG_DIR}/pi05_inference_hls_${RUN_ID}.log}"
CAMERA_LOG_FILE="${CAMERA_LOG_FILE:-${LOG_DIR}/pi05_inference_cameras_${RUN_ID}.log}"
RC_GATE_LOG_FILE="${RC_GATE_LOG_FILE:-${LOG_DIR}/pi05_inference_rc_gate_${RUN_ID}.log}"

START_STACK="${START_STACK:-true}"
START_HLS_MANAGER="${START_HLS_MANAGER:-true}"
START_NX_CAMERAS="${START_NX_CAMERAS:-true}"
START_RC_GRIPPER_GATE="${START_RC_GRIPPER_GATE:-true}"

PX4CTRL_TAKEOFF_HEIGHT="${PX4CTRL_TAKEOFF_HEIGHT:-0.70}"
PX4CTRL_TAKEOFF_LAND_SPEED="${PX4CTRL_TAKEOFF_LAND_SPEED:-0.35}"
PX4CTRL_GRIPPER_RC_CHANNEL="${PX4CTRL_GRIPPER_RC_CHANNEL:-0}"
MAVLINK_STREAM_RATE_CONFIG="${MAVLINK_STREAM_RATE_CONFIG:-false}"
NX_CAMERA_PORT="${NX_CAMERA_PORT:-5556}"

RC_TOPIC="${RC_TOPIC:-/mavros/rc/in}"
CH10_INDEX="${CH10_INDEX:-9}"
CH10_OPEN_PWM="${CH10_OPEN_PWM:-1300}"
RC_GRIPPER_GATE_TIMEOUT_S="${RC_GRIPPER_GATE_TIMEOUT_S:-0.5}"
RC_GRIPPER_GATE_STALE_ACTION="${RC_GRIPPER_GATE_STALE_ACTION:-hold}"
GRIPPER_TOPIC="${GRIPPER_TOPIC:-/gripper/command}"
GRIPPER_COMMAND_PAIR_TOPIC="${GRIPPER_COMMAND_PAIR_TOPIC:-/gripper/command_pair}"
GRIPPER_FEEDBACK_TOPIC="${GRIPPER_FEEDBACK_TOPIC:-/gripper/feedback}"
HLS_STATUS_TOPIC="${HLS_STATUS_TOPIC:-/hls_gripper/status}"
ATTITUDE_TOPIC="${ATTITUDE_TOPIC:-/mavros/imu/data}"
GRIPPER_MANAGER_PORT="${GRIPPER_MANAGER_PORT:-/dev/ttyACM1}"
HLS_GRAVITY_COMP_PATH="${HLS_GRAVITY_COMP_PATH:-${WORKSPACE_DIR}/install/hls_gripper/share/hls_gripper/config/gravity_compensation.json}"
HLS_SDK_ROOT="${HLS_SDK_ROOT:-}"
HLS_DRY_RUN="${HLS_DRY_RUN:-false}"
HLS_OPEN_SPEED="${HLS_OPEN_SPEED:-24}"
HLS_OPEN_ACC="${HLS_OPEN_ACC:-6}"
HLS_OPEN_TORQUE_LIMIT="${HLS_OPEN_TORQUE_LIMIT:-160}"
HLS_LOW_CURRENT="${HLS_LOW_CURRENT:-28}"
HLS_LIFT_CURRENT="${HLS_LIFT_CURRENT:-1000}"
HLS_MAX_CURRENT="${HLS_MAX_CURRENT:-1500}"
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
HLS_SEARCH_TIMEOUT_S="${HLS_SEARCH_TIMEOUT_S:-5.0}"
HLS_DRY_RUN_CONTACT_PATTERN="${HLS_DRY_RUN_CONTACT_PATTERN:-both}"
HLS_MANAGER_STARTUP_TIMEOUT_S="${HLS_MANAGER_STARTUP_TIMEOUT_S:-6.0}"
SUPPORT_PREFLIGHT_ENABLE="${SUPPORT_PREFLIGHT_ENABLE:-true}"
SUPPORT_PREFLIGHT_TIMEOUT_S="${SUPPORT_PREFLIGHT_TIMEOUT_S:-20.0}"
SUPPORT_PREFLIGHT_CHECKER="${SUPPORT_PREFLIGHT_CHECKER:-${WORKSPACE_DIR}/src/px4ctrl/scripts/topic_liveness_check.py}"
VISION_POSE_TOPIC="${VISION_POSE_TOPIC:-/mavros/vision_pose/pose}"
PX4CTRL_STATE_TOPIC="${PX4CTRL_STATE_TOPIC:-/px4ctrl/state}"
HLS_STATUS_SNAPSHOT_TOPIC="${HLS_STATUS_SNAPSHOT_TOPIC:-/hls_gripper/status_snapshot}"

PIDS=()
CLEANED_UP=false

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

start_process() {
  local label="$1"
  local log_file="$2"
  shift 2
  echo "[pi05-support-nx] starting ${label}; log=${log_file}"
  setsid "$@" >"${log_file}" 2>&1 &
  PIDS+=("$!")
}

cleanup() {
  if [[ "${CLEANED_UP}" == "true" ]]; then
    return
  fi
  CLEANED_UP=true
  echo "[pi05-support-nx] stopping child process groups"
  for pid in "${PIDS[@]:-}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  done
  for _ in {1..30}; do
    local any_alive=false
    for pid in "${PIDS[@]:-}"; do
      if kill -0 -- "-${pid}" 2>/dev/null || kill -0 "${pid}" 2>/dev/null; then
        any_alive=true
        break
      fi
    done
    [[ "${any_alive}" == "false" ]] && break
    sleep 0.1
  done
  for pid in "${PIDS[@]:-}"; do
    kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}

trap 'cleanup; exit 130' INT TERM HUP
trap cleanup EXIT

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

if bool_is_true "${START_STACK}"; then
  start_process "mocap/MAVROS/px4ctrl stack" "${STACK_LOG_FILE}" env \
    START_PX4CTRL=true \
    PX4CTRL_GRIPPER_RC_CHANNEL="${PX4CTRL_GRIPPER_RC_CHANNEL}" \
    PX4CTRL_TAKEOFF_HEIGHT="${PX4CTRL_TAKEOFF_HEIGHT}" \
    PX4CTRL_TAKEOFF_LAND_SPEED="${PX4CTRL_TAKEOFF_LAND_SPEED}" \
    MAVLINK_STREAM_RATE_CONFIG="${MAVLINK_STREAM_RATE_CONFIG}" \
    bash "${SCRIPT_DIR}/run_mocap_mavros.sh"
fi

if bool_is_true "${START_NX_CAMERAS}"; then
  start_process "front/down ZMQ cameras" "${CAMERA_LOG_FILE}" env \
    NX_CAMERA_PORT="${NX_CAMERA_PORT}" \
    bash "${SCRIPT_DIR}/start_nx_cameras_zmq.sh"
fi

if bool_is_true "${START_HLS_MANAGER}"; then
  hls_args=(
    ros2 run hls_gripper hls_gripper_node.py
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
    -p center_hold_current:="${HLS_CENTER_HOLD_CURRENT}"
    -p center_push_current:="${HLS_CENTER_PUSH_CURRENT}"
    -p grip_chase_min_current:="${HLS_GRIP_CHASE_MIN_CURRENT}"
    -p grip_chase_position_enable:="${HLS_GRIP_CHASE_POSITION_ENABLE}"
    -p grip_chase_position_speed:="${HLS_GRIP_CHASE_POSITION_SPEED}"
    -p grip_chase_position_acc:="${HLS_GRIP_CHASE_POSITION_ACC}"
    -p grip_chase_position_torque_limit:="${HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT}"
    -p grip_chase_slip_ratio:="$(ros_double "${HLS_GRIP_CHASE_SLIP_RATIO}")"
    -p grip_chase_position_period_s:="$(ros_double "${HLS_GRIP_CHASE_POSITION_PERIOD_S}")"
    -p grip_chase_position_pulse_s:="$(ros_double "${HLS_GRIP_CHASE_POSITION_PULSE_S}")"
    -p center_timeout_action:="${HLS_CENTER_TIMEOUT_ACTION}"
    -p center_timeout_s:="$(ros_double "${HLS_CENTER_TIMEOUT_S}")"
    -p center_stable_time_s:="$(ros_double "${HLS_CENTER_STABLE_TIME_S}")"
    -p final_grip_ramp_s:="$(ros_double "${HLS_FINAL_GRIP_RAMP_S}")"
    -p max_temp:="$(ros_double "${HLS_MAX_TEMP}")"
    -p temp_warn_threshold:="$(ros_double "${HLS_TEMP_WARN_THRESHOLD}")"
    -p left_current_inward_sign:="${HLS_LEFT_CURRENT_INWARD_SIGN}"
    -p right_current_inward_sign:="${HLS_RIGHT_CURRENT_INWARD_SIGN}"
    -p motion_profile:="${HLS_MOTION_PROFILE}"
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
    -p search_timeout_s:="$(ros_double "${HLS_SEARCH_TIMEOUT_S}")"
    -p dry_run_contact_pattern:="${HLS_DRY_RUN_CONTACT_PATTERN}"
  )
  [[ -n "${HLS_GRAVITY_COMP_PATH}" ]] && hls_args+=(-p gravity_comp_path:="${HLS_GRAVITY_COMP_PATH}")
  [[ -n "${HLS_SDK_ROOT}" ]] && hls_args+=(-p sdk_root:="${HLS_SDK_ROOT}")
  [[ -n "${HLS_MOTION_PROFILE_INDEX}" ]] && hls_args+=(-p motion_profile_index:="${HLS_MOTION_PROFILE_INDEX}")
  [[ -n "${HLS_SEARCH_SPEED}" ]] && hls_args+=(-p search_speed:="${HLS_SEARCH_SPEED}")
  [[ -n "${HLS_SEARCH_ACC}" ]] && hls_args+=(-p search_acc:="${HLS_SEARCH_ACC}")
  [[ -n "${HLS_SEARCH_TORQUE_LIMIT}" ]] && hls_args+=(-p search_torque_limit:="${HLS_SEARCH_TORQUE_LIMIT}")

  start_process "HLS gripper manager" "${HLS_LOG_FILE}" "${hls_args[@]}"
fi

if bool_is_true "${START_RC_GRIPPER_GATE}"; then
  start_process "CH10 force-open gripper gate" "${RC_GATE_LOG_FILE}" \
    python3 "${WORKSPACE_DIR}/src/px4ctrl/scripts/pi05_gripper_rc_gate.py" \
      --rc-topic "${RC_TOPIC}" \
      --command-topic "${GRIPPER_TOPIC}" \
      --ch10-index "${CH10_INDEX}" \
      --open-pwm "${CH10_OPEN_PWM}" \
      --open-command 100.0 \
      --rc-timeout-s "${RC_GRIPPER_GATE_TIMEOUT_S}" \
      --rc-stale-action "${RC_GRIPPER_GATE_STALE_ACTION}"
fi

if bool_is_true "${SUPPORT_PREFLIGHT_ENABLE}"; then
  echo "[pi05-support-nx] waiting for critical ROS2 topics before declaring ready"
  preflight_topics=(
    --topic "vision_pose:${VISION_POSE_TOPIC}:geometry_msgs/msg/PoseStamped"
    --topic "px4ctrl_state:${PX4CTRL_STATE_TOPIC}:std_msgs/msg/String"
  )
  if bool_is_true "${START_HLS_MANAGER}"; then
    preflight_topics+=(
      --topic "gripper_feedback:${GRIPPER_FEEDBACK_TOPIC}:quadrotor_msgs/msg/GripperFeedback"
      --topic "hls_snapshot:${HLS_STATUS_SNAPSHOT_TOPIC}:std_msgs/msg/String"
    )
  fi
  if ! python3 "${SUPPORT_PREFLIGHT_CHECKER}" \
    --timeout "${SUPPORT_PREFLIGHT_TIMEOUT_S}" \
    --status-log-period 2.0 \
    --log-prefix "[pi05-support-nx]" \
    "${preflight_topics[@]}"; then
    echo "[pi05-support-nx] ERROR: critical ROS2 preflight failed; refusing inference support startup." >&2
    echo "[pi05-support-nx] Verify VRPN ${VISION_POSE_TOPIC}, px4ctrl, and HLS before takeoff." >&2
    exit 1
  fi
fi

echo "[pi05-support-nx] support stack is ready."
echo "[pi05-support-nx] logs:"
echo "  stack:  ${STACK_LOG_FILE}"
echo "  hls:    ${HLS_LOG_FILE}"
echo "  camera: ${CAMERA_LOG_FILE}"
echo "  rcgate: ${RC_GATE_LOG_FILE}"
echo "[pi05-support-nx] gripper safety: px4ctrl gripper.rc_channel=${PX4CTRL_GRIPPER_RC_CHANNEL}; CH10 low<=${CH10_OPEN_PWM} forces open, high allows pi0.5/HLS control."
echo "[pi05-support-nx] local inference should use NX_CAMERA_SERVER=$(hostname -I | awk '{print $1}') NX_CAMERA_PORT=${NX_CAMERA_PORT}."
echo "[pi05-support-nx] In another terminal, use the existing takeoff flow; this script does not publish takeoff."
echo "[pi05-support-nx] Press Ctrl+C here only after the vehicle is safe."

wait -n "${PIDS[@]}" || true
echo "[pi05-support-nx] a child process exited; stopping support stack."
