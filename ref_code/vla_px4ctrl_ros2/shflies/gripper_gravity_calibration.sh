#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CALIBRATION_SCRIPT="${WORKSPACE_DIR}/src/hls_gripper/scripts/gripper_gravity_calibration.py"

GRIPPER_PORT="${GRIPPER_PORT:-/dev/ttyACM1}"
ATTITUDE_SOURCE="${ATTITUDE_SOURCE:-ros-imu}"
ATTITUDE_TOPIC="${ATTITUDE_TOPIC:-/mavros/imu/data}"

set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/opt/ros/humble/setup.bash
  source /opt/ros/humble/setup.bash
fi

if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WORKSPACE_DIR}/install/setup.bash"
else
  set -u
  echo "[gripper-gravity-calibration] warning: ${WORKSPACE_DIR}/install/setup.bash not found" >&2
  echo "[gripper-gravity-calibration] build first with: colcon build --packages-select hls_gripper" >&2
  set +u
fi
set -u

if [[ "$#" -eq 0 ]]; then
  python3 "${CALIBRATION_SCRIPT}" --help
  exit 0
fi

command="$1"
shift

case "${command}" in
  read-limits)
    echo "[gripper-gravity-calibration] port: ${GRIPPER_PORT}"
    python3 "${CALIBRATION_SCRIPT}" read-limits \
      --port "${GRIPPER_PORT}" \
      "$@"
    ;;
  collect)
    echo "[gripper-gravity-calibration] port: ${GRIPPER_PORT}"
    echo "[gripper-gravity-calibration] attitude: ${ATTITUDE_SOURCE} ${ATTITUDE_TOPIC}"
    python3 "${CALIBRATION_SCRIPT}" collect \
      --port "${GRIPPER_PORT}" \
      --attitude-source "${ATTITUDE_SOURCE}" \
      --attitude-topic "${ATTITUDE_TOPIC}" \
      "$@"
    ;;
  collect-full)
    echo "[gripper-gravity-calibration] port: ${GRIPPER_PORT}"
    echo "[gripper-gravity-calibration] attitude: ${ATTITUDE_SOURCE} ${ATTITUDE_TOPIC}"
    python3 "${CALIBRATION_SCRIPT}" collect-full \
      --port "${GRIPPER_PORT}" \
      --attitude-source "${ATTITUDE_SOURCE}" \
      --attitude-topic "${ATTITUDE_TOPIC}" \
      "$@"
    ;;
  fit)
    python3 "${CALIBRATION_SCRIPT}" fit "$@"
    ;;
  -h|--help|help)
    python3 "${CALIBRATION_SCRIPT}" --help
    ;;
  *)
    echo "[gripper-gravity-calibration] unknown command: ${command}" >&2
    echo "[gripper-gravity-calibration] expected: read-limits, collect, collect-full, or fit" >&2
    exit 2
    ;;
esac
