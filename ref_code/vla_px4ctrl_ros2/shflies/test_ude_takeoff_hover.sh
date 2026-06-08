#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

START_STACK="${START_STACK:-true}"
START_PX4CTRL="${START_PX4CTRL:-true}"
STACK_STARTUP_WAIT_S="${STACK_STARTUP_WAIT_S:-8}"
CLEANUP_STACK_ON_EXIT="${CLEANUP_STACK_ON_EXIT:-auto}"
KEEP_STACK_ON_INTERRUPT="${KEEP_STACK_ON_INTERRUPT:-false}"
TEST_ENTER_CMD="${TEST_ENTER_CMD:-false}"
TEST_PUBLISH_TAKEOFF="${TEST_PUBLISH_TAKEOFF:-false}"
TEST_AUTO_CONFIRM="${TEST_AUTO_CONFIRM:-false}"

STACK_PID=""
HELPER_STATUS=0

cleanup_stack() {
  if [[ -z "${STACK_PID}" ]]; then
    return
  fi

  if [[ "${CLEANUP_STACK_ON_EXIT}" == "true" ]] ||
     [[ "${CLEANUP_STACK_ON_EXIT}" == "auto" && "${HELPER_STATUS}" != "20" ]]; then
    echo "[ude-test] stopping mocap/MAVROS/bridge/px4ctrl stack"
    kill -TERM -- "-${STACK_PID}" 2>/dev/null || kill -TERM "${STACK_PID}" 2>/dev/null || true
    sleep 1
    kill -KILL -- "-${STACK_PID}" 2>/dev/null || kill -KILL "${STACK_PID}" 2>/dev/null || true
  else
    echo "[ude-test] leaving stack running because the vehicle may still be airborne"
    echo "[ude-test] after landing, stop it with: kill -TERM -- -${STACK_PID}"
  fi
}

on_interrupt() {
  echo
  if [[ "${KEEP_STACK_ON_INTERRUPT}" == "true" ]]; then
    echo "[ude-test] interrupted; leaving the stack running because KEEP_STACK_ON_INTERRUPT=true"
    CLEANUP_STACK_ON_EXIT="false"
  else
    echo "[ude-test] interrupted; stopping the stack"
    CLEANUP_STACK_ON_EXIT="true"
  fi
  HELPER_STATUS=130
  exit 130
}

trap on_interrupt INT TERM
trap cleanup_stack EXIT

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
  echo "[ude-test] warning: ${WORKSPACE_DIR}/install/setup.bash not found"
  echo "[ude-test] build first with: colcon build --packages-select px4ctrl"
fi
set -u

if [[ "${START_STACK}" == "true" ]]; then
  echo "[ude-test] starting mocap/MAVROS/bridge/px4ctrl stack"
  export START_PX4CTRL
  setsid bash "${SCRIPT_DIR}/run_mocap_mavros.sh" &
  STACK_PID="$!"
  echo "[ude-test] stack process group: ${STACK_PID}"
  sleep "${STACK_STARTUP_WAIT_S}"
else
  echo "[ude-test] START_STACK=false; using already-running ROS2 stack"
fi

HELPER_ARGS=("$@")
if [[ "${TEST_ENTER_CMD}" == "true" ]]; then
  HELPER_ARGS+=("--enter-cmd")
fi
if [[ "${TEST_PUBLISH_TAKEOFF}" == "true" ]]; then
  HELPER_ARGS+=("--publish-takeoff")
  echo "[ude-test] takeoff mode: helper publishes TAKEOFF"
else
  echo "[ude-test] takeoff mode: manual; helper waits for AUTO_HOVER"
fi
if [[ "${TEST_AUTO_CONFIRM}" == "true" ]]; then
  HELPER_ARGS+=("--auto-confirm")
fi

set +e
ros2 run px4ctrl ude_takeoff_hover_test.py "${HELPER_ARGS[@]}"
HELPER_STATUS="$?"
set -e

exit "${HELPER_STATUS}"
