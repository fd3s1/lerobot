#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

VRPN_SERVER="${VRPN_SERVER:-10.1.1.198}"
VRPN_PORT="${VRPN_PORT:-3883}"
VRPN_SOURCE_TOPIC="${VRPN_SOURCE_TOPIC:-/vla_drone1/pose}"
MAVROS_VISION_TOPIC="${MAVROS_VISION_TOPIC:-/mavros/vision_pose/pose}"
FCU_URL="${FCU_URL:-/dev/ttyACM0:921600}"
GCS_URL="${GCS_URL:-udp://@10.1.1.198:14550}"
MAVROS_TGT_SYSTEM="${MAVROS_TGT_SYSTEM:-1}"
MAVROS_TGT_COMPONENT="${MAVROS_TGT_COMPONENT:-1}"
MAVROS_FCU_PROTOCOL="${MAVROS_FCU_PROTOCOL:-v2.0}"
BRIDGE_RESTAMP="${BRIDGE_RESTAMP:-false}"
BRIDGE_STATUS_PERIOD_S="${BRIDGE_STATUS_PERIOD_S:-10.0}"
PX4CTRL_PARAMS_FILE="${PX4CTRL_PARAMS_FILE:-${WORKSPACE_DIR}/install/px4ctrl/share/px4ctrl/config/ctrl_param_fpv.yaml}"
MAVROS_CONFIG_FILE="${MAVROS_CONFIG_FILE:-/opt/ros/humble/share/mavros/launch/px4_config.yaml}"
MAVROS_LIGHT="${MAVROS_LIGHT:-true}"
MAVROS_LIGHT_PLUGINLISTS_FILE="${MAVROS_LIGHT_PLUGINLISTS_FILE:-${WORKSPACE_DIR}/config/mavros_vla_pluginlists.yaml}"
MAVROS_FULL_PLUGINLISTS_FILE="${MAVROS_FULL_PLUGINLISTS_FILE:-/opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml}"
START_PX4CTRL="${START_PX4CTRL:-true}"
PX4CTRL_GRIPPER_RC_CHANNEL="${PX4CTRL_GRIPPER_RC_CHANNEL:-}"
PX4CTRL_TAKEOFF_LAND_SPEED="${PX4CTRL_TAKEOFF_LAND_SPEED:-}"

PIDS=()
CLEANED_UP=false

start_process() {
  echo "[run-mocap-mavros] starting: $*"
  setsid "$@" &
  PIDS+=("$!")
}

cleanup() {
  if [[ "${CLEANED_UP}" == "true" ]]; then
    return
  fi
  CLEANED_UP=true
  echo "[run-mocap-mavros] stopping child process groups"
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  sleep 1
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
    fi
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
if [[ -n "${PX4CTRL_GRIPPER_RC_CHANNEL}" ]]; then
  echo "[run-mocap-mavros] px4ctrl gripper.rc_channel override: ${PX4CTRL_GRIPPER_RC_CHANNEL}"
fi
if [[ -n "${PX4CTRL_TAKEOFF_LAND_SPEED}" ]]; then
  echo "[run-mocap-mavros] px4ctrl auto_takeoff_land.takeoff_land_speed override: ${PX4CTRL_TAKEOFF_LAND_SPEED}"
fi

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
  start_process "${px4ctrl_args[@]}"
else
  echo "[run-mocap-mavros] skipping px4ctrl_node because START_PX4CTRL=${START_PX4CTRL}"
fi

echo "[run-mocap-mavros] all processes started. Press Ctrl+C to stop."
wait -n "${PIDS[@]}" || true
cleanup
