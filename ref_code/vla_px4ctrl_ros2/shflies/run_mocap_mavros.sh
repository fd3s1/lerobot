#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

VRPN_SERVER="${VRPN_SERVER:-10.1.1.198}"
VRPN_PORT="${VRPN_PORT:-3883}"
VRPN_SOURCE_TOPIC="${VRPN_SOURCE_TOPIC:-/vla_drone1/pose}"
MAVROS_VISION_TOPIC="${MAVROS_VISION_TOPIC:-/mavros/vision_pose/pose}"
FCU_URL="${FCU_URL:-/dev/ttyACM1:921600}"
GCS_URL="${GCS_URL:-udp://@10.1.1.198:14550}"
BRIDGE_RESTAMP="${BRIDGE_RESTAMP:-false}"
PX4CTRL_PARAMS_FILE="${PX4CTRL_PARAMS_FILE:-${WORKSPACE_DIR}/install/px4ctrl/share/px4ctrl/config/ctrl_param_fpv.yaml}"
START_PX4CTRL="${START_PX4CTRL:-true}"

PIDS=()

cleanup() {
  echo "[run-mocap-mavros] stopping child processes"
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

echo "[run-mocap-mavros] workspace: ${WORKSPACE_DIR}"
echo "[run-mocap-mavros] vrpn server: ${VRPN_SERVER}:${VRPN_PORT}"
echo "[run-mocap-mavros] bridge: ${VRPN_SOURCE_TOPIC} -> ${MAVROS_VISION_TOPIC}"
echo "[run-mocap-mavros] fcu_url: ${FCU_URL}"
echo "[run-mocap-mavros] gcs_url: ${GCS_URL}"
echo "[run-mocap-mavros] bridge restamp: ${BRIDGE_RESTAMP}"
echo "[run-mocap-mavros] start px4ctrl: ${START_PX4CTRL}"
echo "[run-mocap-mavros] px4ctrl params: ${PX4CTRL_PARAMS_FILE}"

ros2 run vrpn_mocap client_node --ros-args \
  -p server:="${VRPN_SERVER}" \
  -p port:="${VRPN_PORT}" &
PIDS+=("$!")

sleep 1

ros2 launch mavros px4.launch fcu_url:="${FCU_URL}" gcs_url:="${GCS_URL}" &
PIDS+=("$!")

sleep 2

ros2 run px4ctrl vrpn_to_mavros_vision_bridge.py --ros-args \
  -p source_topic:="${VRPN_SOURCE_TOPIC}" \
  -p target_topic:="${MAVROS_VISION_TOPIC}" \
  -p restamp:="${BRIDGE_RESTAMP}" &
PIDS+=("$!")

sleep 1

if [[ "${START_PX4CTRL}" == "true" ]]; then
  ros2 run px4ctrl px4ctrl_node --ros-args --params-file "${PX4CTRL_PARAMS_FILE}" &
  PIDS+=("$!")
else
  echo "[run-mocap-mavros] skipping px4ctrl_node because START_PX4CTRL=${START_PX4CTRL}"
fi

echo "[run-mocap-mavros] all processes started. Press Ctrl+C to stop."
wait -n "${PIDS[@]}"
