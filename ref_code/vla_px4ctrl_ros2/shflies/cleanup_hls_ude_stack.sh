#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DRY_RUN="${DRY_RUN:-false}"
TERM_WAIT_S="${TERM_WAIT_S:-2}"

patterns=(
  "${WORKSPACE_DIR}/shflies/auto_record_hls_ude_grasp_place.sh"
  "${WORKSPACE_DIR}/shflies/auto_hls_ude_grasp_place_test.sh"
  "${WORKSPACE_DIR}/shflies/auto_hls_grasp_place.sh"
  "${WORKSPACE_DIR}/shflies/record_vla_dataset.sh"
  "${WORKSPACE_DIR}/shflies/run_mocap_mavros.sh"
  "${WORKSPACE_DIR}/shflies/start_pi05_inference_support_nx.sh"
  "${WORKSPACE_DIR}/shflies/start_nx_cameras_zmq.sh"
  "${WORKSPACE_DIR}/src/px4ctrl/scripts/pi05_gripper_rc_gate.py"
  "${WORKSPACE_DIR}/src/px4ctrl/scripts/auto_hls_grasp_place.py"
  "${WORKSPACE_DIR}/src/px4ctrl/scripts/vrpn_to_mavros_vision_bridge.py"
  "${WORKSPACE_DIR}/install/px4ctrl/lib/px4ctrl/vrpn_to_mavros_vision_bridge.py"
  "ros2 run px4ctrl px4ctrl_node"
  "/px4ctrl/px4ctrl_node"
  "ros2 run hls_gripper hls_gripper_node.py"
  "/hls_gripper/hls_gripper_node.py"
  "ros2 launch mavros node.launch"
  "/mavros/mavros_node"
  "ros2 run vrpn_mocap client_node"
  "/vrpn_mocap/client_node"
  "lerobot-record"
)

bool_is_true() {
  case "${1,,}" in
    1|true|yes|y|on) return 0 ;;
    *) return 1 ;;
  esac
}

kill_pattern() {
  local signal="$1"
  local pattern="$2"
  local pids
  pids="$(pgrep -f "${pattern}" || true)"
  if [[ -z "${pids}" ]]; then
    return
  fi
  echo "[cleanup-hls-ude] ${signal} pattern: ${pattern}"
  echo "${pids}" | sed 's/^/[cleanup-hls-ude]   pid /'
  if bool_is_true "${DRY_RUN}"; then
    return
  fi
  # shellcheck disable=SC2086
  kill "-${signal}" ${pids} 2>/dev/null || true
}

echo "[cleanup-hls-ude] workspace: ${WORKSPACE_DIR}"
echo "[cleanup-hls-ude] dry_run: ${DRY_RUN}"

for pattern in "${patterns[@]}"; do
  kill_pattern TERM "${pattern}"
done

sleep "${TERM_WAIT_S}"

for pattern in "${patterns[@]}"; do
  kill_pattern KILL "${pattern}"
done

if ! bool_is_true "${DRY_RUN}"; then
  set +u
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck source=/opt/ros/humble/setup.bash
    source /opt/ros/humble/setup.bash
    ros2 daemon stop >/dev/null 2>&1 || true
  fi
  set -u
fi

echo "[cleanup-hls-ude] done."
