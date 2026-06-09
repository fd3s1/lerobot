#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

START_STACK="${START_STACK:-true}"
START_PX4CTRL="${START_PX4CTRL:-true}"
STACK_STARTUP_WAIT_S="${STACK_STARTUP_WAIT_S:-8}"
QUIET_STACK_OUTPUT="${QUIET_STACK_OUTPUT:-true}"
STACK_LOG_DIR="${STACK_LOG_DIR:-${WORKSPACE_DIR}/log}"
CLEANUP_STACK_ON_EXIT="${CLEANUP_STACK_ON_EXIT:-auto}"
KEEP_STACK_ON_INTERRUPT="${KEEP_STACK_ON_INTERRUPT:-false}"
TEST_ENTER_CMD="${TEST_ENTER_CMD:-true}"
TEST_PUBLISH_TAKEOFF="${TEST_PUBLISH_TAKEOFF:-true}"
TEST_AUTO_CONFIRM="${TEST_AUTO_CONFIRM:-false}"
TEST_RUN_WAYPOINTS="${TEST_RUN_WAYPOINTS:-true}"
TEST_AXIS="${TEST_AXIS:-x}"
TEST_WP_STEP_MIN_M="${TEST_WP_STEP_MIN_M:-0.05}"
TEST_WP_STEP_MAX_M="${TEST_WP_STEP_MAX_M:-1.00}"
TEST_WP_AXIS_LIMIT_M="${TEST_WP_AXIS_LIMIT_M:-1.00}"
TEST_WP_HOLD_S="${TEST_WP_HOLD_S:-4.0}"
TEST_WP_RATE_HZ="${TEST_WP_RATE_HZ:-20.0}"
TEST_WP_RESAMPLE_ATTEMPTS="${TEST_WP_RESAMPLE_ATTEMPTS:-50}"
TEST_WP_RANDOM_SEED="${TEST_WP_RANDOM_SEED:-}"
TEST_WP_X_MIN="${TEST_WP_X_MIN:--7.0}"
TEST_WP_X_MAX="${TEST_WP_X_MAX:-14.0}"
TEST_WP_Y_MIN="${TEST_WP_Y_MIN:--2.5}"
TEST_WP_Y_MAX="${TEST_WP_Y_MAX:-2.5}"
TEST_WP_Z_MIN="${TEST_WP_Z_MIN:--0.3}"
TEST_WP_Z_MAX="${TEST_WP_Z_MAX:-2.5}"

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
  if [[ "${QUIET_STACK_OUTPUT}" == "true" ]]; then
    mkdir -p "${STACK_LOG_DIR}"
    STACK_LOG_FILE="${STACK_LOG_DIR}/ude_takeoff_hover_stack_$(date +%Y%m%d_%H%M%S).log"
    echo "[ude-test] stack output is redirected to: ${STACK_LOG_FILE}"
    setsid bash "${SCRIPT_DIR}/run_mocap_mavros.sh" >"${STACK_LOG_FILE}" 2>&1 &
  else
    setsid bash "${SCRIPT_DIR}/run_mocap_mavros.sh" &
  fi
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
if [[ "${TEST_RUN_WAYPOINTS}" == "true" ]]; then
  HELPER_ARGS+=(
    "--run-waypoints"
    "--test-axis" "${TEST_AXIS}"
    "--wp-step-min-m" "${TEST_WP_STEP_MIN_M}"
    "--wp-step-max-m" "${TEST_WP_STEP_MAX_M}"
    "--wp-axis-limit-m" "${TEST_WP_AXIS_LIMIT_M}"
    "--wp-hold-s" "${TEST_WP_HOLD_S}"
    "--wp-rate-hz" "${TEST_WP_RATE_HZ}"
    "--wp-resample-attempts" "${TEST_WP_RESAMPLE_ATTEMPTS}"
    "--limit-x-min" "${TEST_WP_X_MIN}"
    "--limit-x-max" "${TEST_WP_X_MAX}"
    "--limit-y-min" "${TEST_WP_Y_MIN}"
    "--limit-y-max" "${TEST_WP_Y_MAX}"
    "--limit-z-min" "${TEST_WP_Z_MIN}"
    "--limit-z-max" "${TEST_WP_Z_MAX}"
  )
  if [[ -n "${TEST_WP_RANDOM_SEED}" ]]; then
    HELPER_ARGS+=("--wp-random-seed" "${TEST_WP_RANDOM_SEED}")
  fi
  echo "[ude-test] waypoint mode: axis=${TEST_AXIS} step=[${TEST_WP_STEP_MIN_M},${TEST_WP_STEP_MAX_M}]m origin_limit=${TEST_WP_AXIS_LIMIT_M}m"
else
  echo "[ude-test] waypoint mode: disabled"
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
