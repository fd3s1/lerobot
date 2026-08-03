#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
STACK_LOCK_FILE="${STACK_LOCK_FILE:-/tmp/vla_run_mocap_mavros.lock}"

exec 9>"${STACK_LOCK_FILE}"
if ! flock -n 9; then
  echo "[run-mocap-mavros] another stack already holds ${STACK_LOCK_FILE}" >&2
  pgrep -af 'run_mocap_mavros|mavros_node|px4ctrl_node' >&2 || true
  exit 1
fi
printf '%s\n' "$$" 1>&9

VRPN_SERVER="${VRPN_SERVER:-10.1.1.198}"
VRPN_PORT="${VRPN_PORT:-3883}"
VRPN_SOURCE_TOPIC="${VRPN_SOURCE_TOPIC:-/vla_drone1/pose}"
MAVROS_VISION_TOPIC="${MAVROS_VISION_TOPIC:-/mavros/vision_pose/pose}"
FCU_URL="${FCU_URL:-}"
FCU_BAUD="${FCU_BAUD:-921600}"
GCS_URL="${GCS_URL:-udp://@10.1.1.35:14550}"
MAVROS_TGT_SYSTEM="${MAVROS_TGT_SYSTEM:-1}"
MAVROS_TGT_COMPONENT="${MAVROS_TGT_COMPONENT:-1}"
MAVROS_FCU_PROTOCOL="${MAVROS_FCU_PROTOCOL:-v2.0}"
BRIDGE_RESTAMP="${BRIDGE_RESTAMP:-false}"
BRIDGE_STATUS_PERIOD_S="${BRIDGE_STATUS_PERIOD_S:-10.0}"
PX4CTRL_PARAMS_FILE="${PX4CTRL_PARAMS_FILE:-${WORKSPACE_DIR}/src/px4ctrl/config/ctrl_param_fpv.yaml}"
MAVROS_CONFIG_FILE="${MAVROS_CONFIG_FILE:-/opt/ros/humble/share/mavros/launch/px4_config.yaml}"
MAVROS_LIGHT="${MAVROS_LIGHT:-true}"
MAVROS_LIGHT_PLUGINLISTS_FILE="${MAVROS_LIGHT_PLUGINLISTS_FILE:-${WORKSPACE_DIR}/config/mavros_vla_pluginlists.yaml}"
MAVROS_FULL_PLUGINLISTS_FILE="${MAVROS_FULL_PLUGINLISTS_FILE:-/opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml}"
START_PX4CTRL="${START_PX4CTRL:-true}"
PX4CTRL_GRIPPER_RC_CHANNEL="${PX4CTRL_GRIPPER_RC_CHANNEL:-}"
PX4CTRL_TAKEOFF_LAND_SPEED="${PX4CTRL_TAKEOFF_LAND_SPEED:-}"
PX4CTRL_TAKEOFF_HEIGHT="${PX4CTRL_TAKEOFF_HEIGHT:-}"
PX4CTRL_USE_BODYRATE_CTRL="${PX4CTRL_USE_BODYRATE_CTRL:-}"
PX4CTRL_UDE_ENABLE="${PX4CTRL_UDE_ENABLE:-}"
PX4CTRL_UDE_KD_DIAG="${PX4CTRL_UDE_KD_DIAG:-}"
PX4CTRL_UDE_MAX_F_HAT="${PX4CTRL_UDE_MAX_F_HAT:-}"
PX4CTRL_UDE_MAX_U_ACC="${PX4CTRL_UDE_MAX_U_ACC:-}"
PX4CTRL_TD_ENABLE="${PX4CTRL_TD_ENABLE:-}"
PX4CTRL_PHYSICAL_ENABLE="${PX4CTRL_PHYSICAL_ENABLE:-}"
PX4CTRL_PHYSICAL_MASS_KG="${PX4CTRL_PHYSICAL_MASS_KG:-}"
PX4CTRL_PHYSICAL_BODY_RATE_FF_SCALE="${PX4CTRL_PHYSICAL_BODY_RATE_FF_SCALE:-}"
PX4CTRL_PHYSICAL_ANGULAR_ACCEL_FF_SCALE="${PX4CTRL_PHYSICAL_ANGULAR_ACCEL_FF_SCALE:-}"
PX4CTRL_CMD_FEEDFORWARD_ENABLE="${PX4CTRL_CMD_FEEDFORWARD_ENABLE:-}"
PX4CTRL_CMD_FEEDFORWARD_MAX_VELOCITY="${PX4CTRL_CMD_FEEDFORWARD_MAX_VELOCITY:-}"
PX4CTRL_CMD_FEEDFORWARD_MAX_ACCELERATION="${PX4CTRL_CMD_FEEDFORWARD_MAX_ACCELERATION:-}"
PX4CTRL_CMD_FEEDFORWARD_MAX_JERK="${PX4CTRL_CMD_FEEDFORWARD_MAX_JERK:-}"
PX4CTRL_CMD_FEEDFORWARD_MAX_SNAP="${PX4CTRL_CMD_FEEDFORWARD_MAX_SNAP:-}"
PX4CTRL_CMD_FEEDFORWARD_MAX_YAW_ACCEL="${PX4CTRL_CMD_FEEDFORWARD_MAX_YAW_ACCEL:-}"
PX4CTRL_MOCAP_MAX_PREDICTION_DT_S="${PX4CTRL_MOCAP_MAX_PREDICTION_DT_S:-}"
PX4CTRL_ATTITUDE_FEEDBACK_MODE="${PX4CTRL_ATTITUDE_FEEDBACK_MODE:-}"
PX4CTRL_CONTROLLER_MAX_ANGLE_DEG="${PX4CTRL_CONTROLLER_MAX_ANGLE_DEG:-}"
PX4CTRL_CONTROLLER_MAX_BODYRATE_X="${PX4CTRL_CONTROLLER_MAX_BODYRATE_X:-}"
PX4CTRL_CONTROLLER_MAX_BODYRATE_Y="${PX4CTRL_CONTROLLER_MAX_BODYRATE_Y:-}"
PX4CTRL_CONTROLLER_MAX_BODYRATE_Z="${PX4CTRL_CONTROLLER_MAX_BODYRATE_Z:-}"
PX4CTRL_LIMIT_X_MIN="${PX4CTRL_LIMIT_X_MIN:-}"
PX4CTRL_LIMIT_X_MAX="${PX4CTRL_LIMIT_X_MAX:-}"
PX4CTRL_LIMIT_Y_MIN="${PX4CTRL_LIMIT_Y_MIN:-}"
PX4CTRL_LIMIT_Y_MAX="${PX4CTRL_LIMIT_Y_MAX:-}"
TRAJ_PLANNER_ENABLE="${TRAJ_PLANNER_ENABLE:-false}"
TRAJ_PLANNER_INPUT_TOPIC="${TRAJ_PLANNER_INPUT_TOPIC:-/position_cmd_raw}"
TRAJ_PLANNER_OUTPUT_TOPIC="${TRAJ_PLANNER_OUTPUT_TOPIC:-/position_cmd_traj}"
TRAJ_PLANNER_STATE_TOPIC="${TRAJ_PLANNER_STATE_TOPIC:-/px4ctrl/state}"
TRAJ_PLANNER_STATE_GATE_ENABLE="${TRAJ_PLANNER_STATE_GATE_ENABLE:-true}"
TRAJ_PLANNER_RATE_HZ="${TRAJ_PLANNER_RATE_HZ:-50.0}"
TRAJ_PLANNER_MAX_SPEED_XY_MPS="${TRAJ_PLANNER_MAX_SPEED_XY_MPS:-0.30}"
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
MAVLINK_STREAM_RATE_CONFIG="${MAVLINK_STREAM_RATE_CONFIG:-true}"
MAVLINK_STREAM_RATE_HZ="${MAVLINK_STREAM_RATE_HZ:-100}"
MAVLINK_STREAM_RATE_TIMEOUT_S="${MAVLINK_STREAM_RATE_TIMEOUT_S:-10}"
MAVLINK_STREAM_RATE_REQUIRED="${MAVLINK_STREAM_RATE_REQUIRED:-false}"

PIDS=()
CLEANED_UP=false
PROCESS_CLEANUP_TIMEOUT_S="${PROCESS_CLEANUP_TIMEOUT_S:-3}"

bool_is_true() {
  case "${1,,}" in
    1|true|yes|y|on) return 0 ;;
    *) return 1 ;;
  esac
}

is_px4_serial_device() {
  local device="$1"
  local properties
  properties="$(udevadm info --query=property --name="${device}" 2>/dev/null || true)"
  properties="${properties,,}"

  [[ "${properties}" == *"id_vendor_id=26ac"* ]] ||
    [[ "${properties}" == *"id_vendor=px4"* ]] ||
    [[ "${properties}" == *"id_vendor=holybro"* ]] ||
    [[ "${properties}" == *"id_model=px4"* ]] ||
    [[ "${properties}" == *"id_model=pixhawk"* ]] ||
    [[ "${properties}" == *"id_model=fmu"* ]] ||
    [[ "${properties}" == *"px4_fmu"* ]]
}

detect_px4_serial_link() {
  local link_dir
  local candidate
  local candidates=()

  # Prefer persistent udev links. by-id survives USB port changes; by-path is
  # the fallback when the device firmware does not expose a USB serial number.
  for link_dir in /dev/serial/by-id /dev/serial/by-path; do
    candidates=()
    if [[ -d "${link_dir}" ]]; then
      for candidate in "${link_dir}"/*; do
        [[ -e "${candidate}" ]] || continue
        if is_px4_serial_device "${candidate}"; then
          candidates+=("${candidate}")
        fi
      done
    fi

    if (( ${#candidates[@]} == 1 )); then
      printf '%s' "${candidates[0]}"
      return 0
    fi

    if (( ${#candidates[@]} > 1 )); then
      echo "[run-mocap-mavros] multiple PX4 serial links found in ${link_dir}: ${candidates[*]}" >&2
      echo "[run-mocap-mavros] set FCU_URL=<stable-link>:${FCU_BAUD} explicitly" >&2
      return 2
    fi
  done

  return 1
}

resolve_fcu_url() {
  if [[ -n "${FCU_URL}" ]]; then
    return
  fi

  local fcu_device
  if ! fcu_device="$(detect_px4_serial_link)"; then
    echo "[run-mocap-mavros] no unique PX4 /dev/serial/by-id or by-path link found" >&2
    echo "[run-mocap-mavros] connect the Pixhawk, or set FCU_URL explicitly; refusing to guess /dev/ttyACM0" >&2
    exit 1
  fi

  FCU_URL="${fcu_device}:${FCU_BAUD}"
}

start_process() {
  echo "[run-mocap-mavros] starting: $*"
  setsid "$@" &
  PIDS+=("$!")
}

wait_for_ros_service() {
  local service_name="$1"
  local timeout_s="$2"
  local deadline=$((SECONDS + ${timeout_s%.*}))
  while (( SECONDS <= deadline )); do
    if ros2 service list --no-daemon --spin-time 0.5 2>/dev/null |
      grep -qx "${service_name}"; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

set_mavlink_message_interval() {
  local message_id="$1"
  local label="$2"
  local interval_us="$3"
  local service_name="/mavros/cmd/command"

  echo "[run-mocap-mavros] requesting ${label} (${message_id}) at ${MAVLINK_STREAM_RATE_HZ}Hz"
  local response_log="/tmp/run_mocap_mavros_stream_${message_id}.log"
  if ! timeout 5 ros2 service call "${service_name}" mavros_msgs/srv/CommandLong \
    "{broadcast: false, command: 511, confirmation: 0, param1: ${message_id}.0, param2: ${interval_us}.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}" >"${response_log}" 2>&1; then
    echo "[run-mocap-mavros] warning: failed to request ${label}; see ${response_log}"
    return 1
  fi
  if ! grep -Eq 'success[=:][[:space:]]*(true|True)' "${response_log}"; then
    echo "[run-mocap-mavros] warning: PX4 rejected ${label} stream request; see ${response_log}"
    return 1
  fi
  echo "[run-mocap-mavros] confirmed ${label} (${message_id}) at ${MAVLINK_STREAM_RATE_HZ}Hz"
  return 0
}

configure_mavlink_stream_rates() {
  if [[ "${MAVLINK_STREAM_RATE_CONFIG}" != "true" ]]; then
    echo "[run-mocap-mavros] MAVLink stream rate config disabled"
    return
  fi

  local service_name="/mavros/cmd/command"
  if ! wait_for_ros_service "${service_name}" "${MAVLINK_STREAM_RATE_TIMEOUT_S}"; then
    echo "[run-mocap-mavros] warning: ${service_name} not available; stream rates not changed"
    return 1
  fi

  local interval_us
  interval_us="$(awk -v hz="${MAVLINK_STREAM_RATE_HZ}" 'BEGIN { if (hz <= 0) exit 1; printf "%.0f", 1000000.0 / hz }')" || {
    echo "[run-mocap-mavros] warning: invalid MAVLINK_STREAM_RATE_HZ=${MAVLINK_STREAM_RATE_HZ}; stream rates not changed"
    return 1
  }

  local failed=0
  set_mavlink_message_interval 105 HIGHRES_IMU "${interval_us}" || failed=1
  set_mavlink_message_interval 31 ATTITUDE_QUATERNION "${interval_us}" || failed=1
  set_mavlink_message_interval 32 LOCAL_POSITION_NED "${interval_us}" || failed=1
  set_mavlink_message_interval 331 ODOMETRY "${interval_us}" || failed=1
  return "${failed}"
}

cleanup() {
  if [[ "${CLEANED_UP}" == "true" ]]; then
    return
  fi
  CLEANED_UP=true
  echo "[run-mocap-mavros] stopping child process groups"
  for pid in "${PIDS[@]:-}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  done
  local deadline
  deadline="$(awk -v now="${SECONDS}" -v timeout="${PROCESS_CLEANUP_TIMEOUT_S}" 'BEGIN { printf "%.0f", now + timeout }')"
  while (( SECONDS < deadline )); do
    local any_alive=false
    for pid in "${PIDS[@]:-}"; do
      if kill -0 -- "-${pid}" 2>/dev/null || kill -0 "${pid}" 2>/dev/null; then
        any_alive=true
        break
      fi
    done
    if [[ "${any_alive}" != "true" ]]; then
      break
    fi
    sleep 0.1
  done
  for pid in "${PIDS[@]:-}"; do
    kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}

trap 'cleanup; exit 130' INT TERM
trap cleanup EXIT

set +u
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
set -u

resolve_fcu_url

if bool_is_true "${TRAJ_PLANNER_ENABLE}"; then
  PX4CTRL_TD_ENABLE="${PX4CTRL_TD_ENABLE:-false}"
  PX4CTRL_CMD_FEEDFORWARD_ENABLE="${PX4CTRL_CMD_FEEDFORWARD_ENABLE:-true}"
fi

echo "[run-mocap-mavros] workspace: ${WORKSPACE_DIR}"
echo "[run-mocap-mavros] vrpn server: ${VRPN_SERVER}:${VRPN_PORT}"
echo "[run-mocap-mavros] bridge: ${VRPN_SOURCE_TOPIC} -> ${MAVROS_VISION_TOPIC}"
echo "[run-mocap-mavros] fcu_url: ${FCU_URL}"
echo "[run-mocap-mavros] gcs_url: ${GCS_URL}"
echo "[run-mocap-mavros] target system/component: ${MAVROS_TGT_SYSTEM}/${MAVROS_TGT_COMPONENT}"
echo "[run-mocap-mavros] bridge restamp: ${BRIDGE_RESTAMP}"
echo "[run-mocap-mavros] bridge status period: ${BRIDGE_STATUS_PERIOD_S}s"
echo "[run-mocap-mavros] mavros light mode: ${MAVROS_LIGHT}"
echo "[run-mocap-mavros] start px4ctrl: ${START_PX4CTRL}"
echo "[run-mocap-mavros] px4ctrl params: ${PX4CTRL_PARAMS_FILE}"
echo "[run-mocap-mavros] MAVLink stream rate config: ${MAVLINK_STREAM_RATE_CONFIG} rate=${MAVLINK_STREAM_RATE_HZ}Hz"
if [[ -n "${PX4CTRL_GRIPPER_RC_CHANNEL}" ]]; then
  echo "[run-mocap-mavros] px4ctrl gripper.rc_channel override: ${PX4CTRL_GRIPPER_RC_CHANNEL}"
fi
if [[ -n "${PX4CTRL_TAKEOFF_LAND_SPEED}" ]]; then
  echo "[run-mocap-mavros] px4ctrl auto_takeoff_land.takeoff_land_speed override: ${PX4CTRL_TAKEOFF_LAND_SPEED}"
fi
if [[ -n "${PX4CTRL_TAKEOFF_HEIGHT}" ]]; then
  echo "[run-mocap-mavros] px4ctrl auto_takeoff_land.takeoff_height override: ${PX4CTRL_TAKEOFF_HEIGHT}"
fi
if [[ -n "${PX4CTRL_USE_BODYRATE_CTRL}" ]]; then
  echo "[run-mocap-mavros] px4ctrl use_bodyrate_ctrl override: ${PX4CTRL_USE_BODYRATE_CTRL}"
fi
if [[ -n "${PX4CTRL_UDE_ENABLE}" ]]; then
  echo "[run-mocap-mavros] px4ctrl ude.enable override: ${PX4CTRL_UDE_ENABLE}"
fi
if [[ -n "${PX4CTRL_UDE_KD_DIAG}" ]]; then
  echo "[run-mocap-mavros] px4ctrl ude.Kd_diag override: ${PX4CTRL_UDE_KD_DIAG}"
fi
if [[ -n "${PX4CTRL_UDE_MAX_F_HAT}" ]]; then
  echo "[run-mocap-mavros] px4ctrl ude.max_f_hat override: ${PX4CTRL_UDE_MAX_F_HAT}m/s2"
fi
if [[ -n "${PX4CTRL_UDE_MAX_U_ACC}" ]]; then
  echo "[run-mocap-mavros] px4ctrl ude.max_u_acc override: ${PX4CTRL_UDE_MAX_U_ACC}m/s2"
fi
if [[ -n "${PX4CTRL_TD_ENABLE}" ]]; then
  echo "[run-mocap-mavros] px4ctrl td.enable override: ${PX4CTRL_TD_ENABLE}"
fi
if [[ -n "${PX4CTRL_PHYSICAL_ENABLE}" ]]; then
  echo "[run-mocap-mavros] px4ctrl physical_control.enable override: ${PX4CTRL_PHYSICAL_ENABLE}"
fi
if [[ -n "${PX4CTRL_PHYSICAL_MASS_KG}" ]]; then
  echo "[run-mocap-mavros] px4ctrl physical_control.mass_kg override: ${PX4CTRL_PHYSICAL_MASS_KG}kg"
fi
if [[ -n "${PX4CTRL_PHYSICAL_BODY_RATE_FF_SCALE}" ]]; then
  echo "[run-mocap-mavros] px4ctrl physical_control.body_rate_feedforward_scale override: ${PX4CTRL_PHYSICAL_BODY_RATE_FF_SCALE}"
fi
if [[ -n "${PX4CTRL_PHYSICAL_ANGULAR_ACCEL_FF_SCALE}" ]]; then
  echo "[run-mocap-mavros] px4ctrl physical_control.angular_acceleration_feedforward_scale override: ${PX4CTRL_PHYSICAL_ANGULAR_ACCEL_FF_SCALE}"
fi
if [[ -n "${PX4CTRL_CMD_FEEDFORWARD_ENABLE}" ]]; then
  echo "[run-mocap-mavros] px4ctrl cmd_feedforward.enable override: ${PX4CTRL_CMD_FEEDFORWARD_ENABLE}"
fi
if [[ -n "${PX4CTRL_CMD_FEEDFORWARD_MAX_VELOCITY}" ]]; then
  echo "[run-mocap-mavros] px4ctrl cmd_feedforward.max_velocity override: ${PX4CTRL_CMD_FEEDFORWARD_MAX_VELOCITY}m/s"
fi
if [[ -n "${PX4CTRL_CMD_FEEDFORWARD_MAX_ACCELERATION}" ]]; then
  echo "[run-mocap-mavros] px4ctrl cmd_feedforward.max_acceleration override: ${PX4CTRL_CMD_FEEDFORWARD_MAX_ACCELERATION}m/s2"
fi
if [[ -n "${PX4CTRL_CMD_FEEDFORWARD_MAX_JERK}" ]]; then
  echo "[run-mocap-mavros] px4ctrl cmd_feedforward.max_jerk override: ${PX4CTRL_CMD_FEEDFORWARD_MAX_JERK}m/s3"
fi
if [[ -n "${PX4CTRL_CMD_FEEDFORWARD_MAX_SNAP}" ]]; then
  echo "[run-mocap-mavros] px4ctrl cmd_feedforward.max_snap override: ${PX4CTRL_CMD_FEEDFORWARD_MAX_SNAP}m/s4"
fi
if [[ -n "${PX4CTRL_MOCAP_MAX_PREDICTION_DT_S}" ]]; then
  echo "[run-mocap-mavros] px4ctrl mocap_state.max_prediction_dt_s override: ${PX4CTRL_MOCAP_MAX_PREDICTION_DT_S}s"
fi
if [[ -n "${PX4CTRL_CMD_FEEDFORWARD_MAX_YAW_ACCEL}" ]]; then
  echo "[run-mocap-mavros] px4ctrl cmd_feedforward.max_yaw_acceleration override: ${PX4CTRL_CMD_FEEDFORWARD_MAX_YAW_ACCEL}rad/s2"
fi
if [[ -n "${PX4CTRL_ATTITUDE_FEEDBACK_MODE}" ]]; then
  echo "[run-mocap-mavros] px4ctrl attitude.feedback_mode override: ${PX4CTRL_ATTITUDE_FEEDBACK_MODE}"
fi
if [[ -n "${PX4CTRL_CONTROLLER_MAX_ANGLE_DEG}" ]]; then
  echo "[run-mocap-mavros] px4ctrl controller.max_angle_deg override: ${PX4CTRL_CONTROLLER_MAX_ANGLE_DEG}deg"
fi
if [[ -n "${PX4CTRL_CONTROLLER_MAX_BODYRATE_X}${PX4CTRL_CONTROLLER_MAX_BODYRATE_Y}${PX4CTRL_CONTROLLER_MAX_BODYRATE_Z}" ]]; then
  echo "[run-mocap-mavros] px4ctrl controller bodyrate limits override: [${PX4CTRL_CONTROLLER_MAX_BODYRATE_X},${PX4CTRL_CONTROLLER_MAX_BODYRATE_Y},${PX4CTRL_CONTROLLER_MAX_BODYRATE_Z}]rad/s"
fi
if [[ -n "${PX4CTRL_LIMIT_X_MIN}${PX4CTRL_LIMIT_X_MAX}${PX4CTRL_LIMIT_Y_MIN}${PX4CTRL_LIMIT_Y_MAX}" ]]; then
  echo "[run-mocap-mavros] px4ctrl XY limits override: x=[${PX4CTRL_LIMIT_X_MIN},${PX4CTRL_LIMIT_X_MAX}] y=[${PX4CTRL_LIMIT_Y_MIN},${PX4CTRL_LIMIT_Y_MAX}]"
fi
echo "[run-mocap-mavros] trajectory planner: enable=${TRAJ_PLANNER_ENABLE} raw=${TRAJ_PLANNER_INPUT_TOPIC} traj=${TRAJ_PLANNER_OUTPUT_TOPIC} state_gate=${TRAJ_PLANNER_STATE_GATE_ENABLE} state_topic=${TRAJ_PLANNER_STATE_TOPIC} accel_xy=${TRAJ_PLANNER_MAX_ACCEL_XY_MPS2} accel_z=${TRAJ_PLANNER_MAX_ACCEL_Z_MPS2} centering_vxy=${TRAJ_PLANNER_CENTERING_MAX_SPEED_XY_MPS} duration_scale=${TRAJ_PLANNER_DURATION_SCALE} vertical_xy_reset=${TRAJ_PLANNER_ZERO_LATERAL_START_DERIVATIVES_FOR_VERTICAL_GOALS} vertical_xy_threshold=${TRAJ_PLANNER_VERTICAL_GOAL_XY_THRESHOLD_M} vertical_z_min=${TRAJ_PLANNER_VERTICAL_GOAL_Z_MIN_M}"

if [[ "${MAVROS_LIGHT}" == "true" ]]; then
  MAVROS_PLUGINLISTS_FILE="${MAVROS_LIGHT_PLUGINLISTS_FILE}"
else
  MAVROS_PLUGINLISTS_FILE="${MAVROS_FULL_PLUGINLISTS_FILE}"
fi
echo "[run-mocap-mavros] mavros plugin list: ${MAVROS_PLUGINLISTS_FILE}"

start_process ros2 run vrpn_mocap client_node --ros-args \
  -p server:="${VRPN_SERVER}" \
  -p port:="${VRPN_PORT}"

sleep 1

start_process ros2 launch mavros node.launch \
  fcu_url:="${FCU_URL}" \
  gcs_url:="${GCS_URL}" \
  tgt_system:="${MAVROS_TGT_SYSTEM}" \
  tgt_component:="${MAVROS_TGT_COMPONENT}" \
  fcu_protocol:="${MAVROS_FCU_PROTOCOL}" \
  pluginlists_yaml:="${MAVROS_PLUGINLISTS_FILE}" \
  config_yaml:="${MAVROS_CONFIG_FILE}" \
  namespace:=mavros

sleep 2

if ! configure_mavlink_stream_rates; then
  if bool_is_true "${MAVLINK_STREAM_RATE_REQUIRED}"; then
    echo "[run-mocap-mavros] required MAVLink stream-rate configuration failed; stopping stack" >&2
    exit 1
  fi
  echo "[run-mocap-mavros] continuing without confirmed stream-rate configuration"
fi

start_process python3 "${WORKSPACE_DIR}/src/px4ctrl/scripts/vrpn_to_mavros_vision_bridge.py" --ros-args \
  -p source_topic:="${VRPN_SOURCE_TOPIC}" \
  -p target_topic:="${MAVROS_VISION_TOPIC}" \
  -p restamp:="${BRIDGE_RESTAMP}" \
  -p status_period_s:="${BRIDGE_STATUS_PERIOD_S}"

sleep 1

if [[ "${START_PX4CTRL}" == "true" ]]; then
  px4ctrl_args=(ros2 run px4ctrl px4ctrl_node --ros-args --params-file "${PX4CTRL_PARAMS_FILE}")
  if [[ -n "${PX4CTRL_GRIPPER_RC_CHANNEL}" ]]; then
    px4ctrl_args+=(-p "gripper.rc_channel:=${PX4CTRL_GRIPPER_RC_CHANNEL}")
  fi
  if [[ -n "${PX4CTRL_TAKEOFF_LAND_SPEED}" ]]; then
    px4ctrl_args+=(-p "auto_takeoff_land.takeoff_land_speed:=${PX4CTRL_TAKEOFF_LAND_SPEED}")
  fi
  if [[ -n "${PX4CTRL_TAKEOFF_HEIGHT}" ]]; then
    px4ctrl_args+=(-p "auto_takeoff_land.takeoff_height:=${PX4CTRL_TAKEOFF_HEIGHT}")
  fi
  if [[ -n "${PX4CTRL_USE_BODYRATE_CTRL}" ]]; then
    px4ctrl_args+=(-p "use_bodyrate_ctrl:=${PX4CTRL_USE_BODYRATE_CTRL}")
  fi
  if [[ -n "${PX4CTRL_UDE_ENABLE}" ]]; then
    px4ctrl_args+=(-p "ude.enable:=${PX4CTRL_UDE_ENABLE}")
  fi
  if [[ -n "${PX4CTRL_UDE_KD_DIAG}" ]]; then
    px4ctrl_args+=(-p "ude.Kd_diag:=${PX4CTRL_UDE_KD_DIAG}")
  fi
  if [[ -n "${PX4CTRL_UDE_MAX_F_HAT}" ]]; then
    px4ctrl_args+=(-p "ude.max_f_hat:=${PX4CTRL_UDE_MAX_F_HAT}")
  fi
  if [[ -n "${PX4CTRL_UDE_MAX_U_ACC}" ]]; then
    px4ctrl_args+=(-p "ude.max_u_acc:=${PX4CTRL_UDE_MAX_U_ACC}")
  fi
  if [[ -n "${PX4CTRL_TD_ENABLE}" ]]; then
    px4ctrl_args+=(-p "td.enable:=${PX4CTRL_TD_ENABLE}")
  fi
  if [[ -n "${PX4CTRL_PHYSICAL_ENABLE}" ]]; then
    px4ctrl_args+=(-p "physical_control.enable:=${PX4CTRL_PHYSICAL_ENABLE}")
  fi
  if [[ -n "${PX4CTRL_PHYSICAL_MASS_KG}" ]]; then
    px4ctrl_args+=(-p "physical_control.mass_kg:=${PX4CTRL_PHYSICAL_MASS_KG}")
  fi
  if [[ -n "${PX4CTRL_PHYSICAL_BODY_RATE_FF_SCALE}" ]]; then
    px4ctrl_args+=(
      -p "physical_control.body_rate_feedforward_scale:=${PX4CTRL_PHYSICAL_BODY_RATE_FF_SCALE}"
    )
  fi
  if [[ -n "${PX4CTRL_PHYSICAL_ANGULAR_ACCEL_FF_SCALE}" ]]; then
    px4ctrl_args+=(
      -p "physical_control.angular_acceleration_feedforward_scale:=${PX4CTRL_PHYSICAL_ANGULAR_ACCEL_FF_SCALE}"
    )
  fi
  if [[ -n "${PX4CTRL_CMD_FEEDFORWARD_ENABLE}" ]]; then
    px4ctrl_args+=(-p "cmd_feedforward.enable:=${PX4CTRL_CMD_FEEDFORWARD_ENABLE}")
  fi
  if [[ -n "${PX4CTRL_CMD_FEEDFORWARD_MAX_VELOCITY}" ]]; then
    px4ctrl_args+=(-p "cmd_feedforward.max_velocity:=${PX4CTRL_CMD_FEEDFORWARD_MAX_VELOCITY}")
  fi
  if [[ -n "${PX4CTRL_CMD_FEEDFORWARD_MAX_ACCELERATION}" ]]; then
    px4ctrl_args+=(-p "cmd_feedforward.max_acceleration:=${PX4CTRL_CMD_FEEDFORWARD_MAX_ACCELERATION}")
  fi
  if [[ -n "${PX4CTRL_CMD_FEEDFORWARD_MAX_JERK}" ]]; then
    px4ctrl_args+=(-p "cmd_feedforward.max_jerk:=${PX4CTRL_CMD_FEEDFORWARD_MAX_JERK}")
  fi
  if [[ -n "${PX4CTRL_CMD_FEEDFORWARD_MAX_SNAP}" ]]; then
    px4ctrl_args+=(-p "cmd_feedforward.max_snap:=${PX4CTRL_CMD_FEEDFORWARD_MAX_SNAP}")
  fi
  if [[ -n "${PX4CTRL_MOCAP_MAX_PREDICTION_DT_S}" ]]; then
    px4ctrl_args+=(
      -p "mocap_state.max_prediction_dt_s:=${PX4CTRL_MOCAP_MAX_PREDICTION_DT_S}"
    )
  fi
  if [[ -n "${PX4CTRL_CMD_FEEDFORWARD_MAX_YAW_ACCEL}" ]]; then
    px4ctrl_args+=(-p "cmd_feedforward.max_yaw_acceleration:=${PX4CTRL_CMD_FEEDFORWARD_MAX_YAW_ACCEL}")
  fi
  if [[ -n "${PX4CTRL_ATTITUDE_FEEDBACK_MODE}" ]]; then
    px4ctrl_args+=(-p "attitude.feedback_mode:=${PX4CTRL_ATTITUDE_FEEDBACK_MODE}")
  fi
  if [[ -n "${PX4CTRL_CONTROLLER_MAX_ANGLE_DEG}" ]]; then
    px4ctrl_args+=(-p "controller.max_angle_deg:=${PX4CTRL_CONTROLLER_MAX_ANGLE_DEG}")
  fi
  if [[ -n "${PX4CTRL_CONTROLLER_MAX_BODYRATE_X}" ]]; then
    px4ctrl_args+=(-p "controller.max_bodyrate_x:=${PX4CTRL_CONTROLLER_MAX_BODYRATE_X}")
  fi
  if [[ -n "${PX4CTRL_CONTROLLER_MAX_BODYRATE_Y}" ]]; then
    px4ctrl_args+=(-p "controller.max_bodyrate_y:=${PX4CTRL_CONTROLLER_MAX_BODYRATE_Y}")
  fi
  if [[ -n "${PX4CTRL_CONTROLLER_MAX_BODYRATE_Z}" ]]; then
    px4ctrl_args+=(-p "controller.max_bodyrate_z:=${PX4CTRL_CONTROLLER_MAX_BODYRATE_Z}")
  fi
  if [[ -n "${PX4CTRL_LIMIT_X_MIN}" ]]; then
    px4ctrl_args+=(-p "limits.x_min:=${PX4CTRL_LIMIT_X_MIN}")
  fi
  if [[ -n "${PX4CTRL_LIMIT_X_MAX}" ]]; then
    px4ctrl_args+=(-p "limits.x_max:=${PX4CTRL_LIMIT_X_MAX}")
  fi
  if [[ -n "${PX4CTRL_LIMIT_Y_MIN}" ]]; then
    px4ctrl_args+=(-p "limits.y_min:=${PX4CTRL_LIMIT_Y_MIN}")
  fi
  if [[ -n "${PX4CTRL_LIMIT_Y_MAX}" ]]; then
    px4ctrl_args+=(-p "limits.y_max:=${PX4CTRL_LIMIT_Y_MAX}")
  fi
  start_process "${px4ctrl_args[@]}"
else
  echo "[run-mocap-mavros] skipping px4ctrl_node because START_PX4CTRL=${START_PX4CTRL}"
fi

if bool_is_true "${TRAJ_PLANNER_ENABLE}"; then
  start_process ros2 run px4ctrl trajectory_planner_node --ros-args \
    -p input_topic:="${TRAJ_PLANNER_INPUT_TOPIC}" \
    -p output_topic:="${TRAJ_PLANNER_OUTPUT_TOPIC}" \
    -p state_topic:="${TRAJ_PLANNER_STATE_TOPIC}" \
    -p state_gate_enable:="${TRAJ_PLANNER_STATE_GATE_ENABLE}" \
    -p rate_hz:="${TRAJ_PLANNER_RATE_HZ}" \
    -p max_speed_xy_mps:="${TRAJ_PLANNER_MAX_SPEED_XY_MPS}" \
    -p max_accel_xy_mps2:="${TRAJ_PLANNER_MAX_ACCEL_XY_MPS2}" \
    -p max_speed_z_mps:="${TRAJ_PLANNER_MAX_SPEED_Z_MPS}" \
    -p max_accel_z_mps2:="${TRAJ_PLANNER_MAX_ACCEL_Z_MPS2}" \
    -p centering_max_speed_xy_mps:="${TRAJ_PLANNER_CENTERING_MAX_SPEED_XY_MPS}" \
    -p centering_max_accel_xy_mps2:="${TRAJ_PLANNER_CENTERING_MAX_ACCEL_XY_MPS2}" \
    -p max_yaw_rate_radps:="${TRAJ_PLANNER_MAX_YAW_RATE_RADPS}" \
    -p max_yaw_accel_radps2:="${TRAJ_PLANNER_MAX_YAW_ACCEL_RADPS2}" \
    -p goal_replan_pos_threshold_m:="${TRAJ_PLANNER_GOAL_REPLAN_POS_THRESHOLD_M}" \
    -p goal_replan_yaw_threshold_rad:="${TRAJ_PLANNER_GOAL_REPLAN_YAW_THRESHOLD_RAD}" \
    -p goal_stale_timeout_s:="${TRAJ_PLANNER_GOAL_STALE_TIMEOUT_S}" \
    -p duration_scale:="${TRAJ_PLANNER_DURATION_SCALE}" \
    -p zero_lateral_start_derivatives_for_vertical_goals:="${TRAJ_PLANNER_ZERO_LATERAL_START_DERIVATIVES_FOR_VERTICAL_GOALS}" \
    -p vertical_goal_xy_threshold_m:="${TRAJ_PLANNER_VERTICAL_GOAL_XY_THRESHOLD_M}" \
    -p vertical_goal_z_min_m:="${TRAJ_PLANNER_VERTICAL_GOAL_Z_MIN_M}"
fi

echo "[run-mocap-mavros] all processes started. Press Ctrl+C to stop."
wait -n "${PIDS[@]}" || true
cleanup
