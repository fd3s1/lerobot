#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

START_STACK="${START_STACK:-true}"
START_PX4CTRL="${START_PX4CTRL:-true}"
STACK_STARTUP_WAIT_S="${STACK_STARTUP_WAIT_S:-18}"
QUIET_STACK_OUTPUT="${QUIET_STACK_OUTPUT:-true}"
STACK_LOG_DIR="${STACK_LOG_DIR:-${WORKSPACE_DIR}/log}"
CLEANUP_STACK_ON_EXIT="${CLEANUP_STACK_ON_EXIT:-true}"
KEEP_STACK_ON_INTERRUPT="${KEEP_STACK_ON_INTERRUPT:-false}"
TEST_PUBLISH_TAKEOFF="${TEST_PUBLISH_TAKEOFF:-true}"
TEST_AUTO_CONFIRM="${TEST_AUTO_CONFIRM:-false}"
TEST_RUN_WAYPOINTS="${TEST_RUN_WAYPOINTS:-true}"
TEST_ENTER_CMD="${TEST_ENTER_CMD:-${TEST_RUN_WAYPOINTS}}"
TEST_RECORD_BAG="${TEST_RECORD_BAG:-false}"
TEST_ANALYZE_BAG="${TEST_ANALYZE_BAG:-true}"
TEST_REQUIRE_COMPOSED_STATE="${TEST_REQUIRE_COMPOSED_STATE:-true}"
TEST_BAG_ROOT="${TEST_BAG_ROOT:-${WORKSPACE_DIR}/bags}"
TEST_BAG_STORAGE="${TEST_BAG_STORAGE:-sqlite3}"
ROS_PYTHON="${ROS_PYTHON:-/usr/bin/python3}"
TEST_AXIS="${TEST_AXIS:-x}"
TEST_UDE_ENABLE="${TEST_UDE_ENABLE:-}"
TEST_UDE_KD_DIAG="${TEST_UDE_KD_DIAG:-}"
TEST_UDE_PARAM_TIMEOUT_S="${TEST_UDE_PARAM_TIMEOUT_S:-10}"
TEST_TD_ENABLE="${TEST_TD_ENABLE:-}"
TEST_TD_PARAM_TIMEOUT_S="${TEST_TD_PARAM_TIMEOUT_S:-10}"
TEST_PHYSICAL_ENABLE="${TEST_PHYSICAL_ENABLE:-false}"
TEST_PHYSICAL_MASS_KG="${TEST_PHYSICAL_MASS_KG:-1.75}"
TEST_USE_BODYRATE_CTRL="${TEST_USE_BODYRATE_CTRL:-}"
TEST_ATTITUDE_FEEDBACK_MODE="${TEST_ATTITUDE_FEEDBACK_MODE:-}"
TEST_ATTITUDE_PARAM_TIMEOUT_S="${TEST_ATTITUDE_PARAM_TIMEOUT_S:-10}"
TEST_WP_MODE="${TEST_WP_MODE:-toggle}"
TEST_WP_X_LOW="${TEST_WP_X_LOW:--1.0}"
TEST_WP_X_HIGH="${TEST_WP_X_HIGH:-1.0}"
TEST_WP_Y_LOW="${TEST_WP_Y_LOW:--1.0}"
TEST_WP_Y_HIGH="${TEST_WP_Y_HIGH:-1.0}"
TEST_WP_Z_LOW="${TEST_WP_Z_LOW:-0.6}"
TEST_WP_Z_HIGH="${TEST_WP_Z_HIGH:-1.2}"
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
UDE_ENABLE_NORMALIZED=""
UDE_KD_DIAG_NORMALIZED=""
TD_ENABLE_NORMALIZED=""
PHYSICAL_ENABLE_NORMALIZED=""
USE_BODYRATE_CTRL_NORMALIZED=""
ATTITUDE_FEEDBACK_MODE_NORMALIZED=""
RECORD_BAG_NORMALIZED=""
ANALYZE_BAG_NORMALIZED=""
REQUIRE_COMPOSED_STATE_NORMALIZED=""
PARAM_OVERRIDES_VIA_LAUNCH=false

normalize_bool() {
  case "$1" in
    true|TRUE|True|1|yes|YES|Yes|on|ON|On)
      printf "true"
      ;;
    false|FALSE|False|0|no|NO|No|off|OFF|Off)
      printf "false"
      ;;
    *)
      return 1
      ;;
  esac
}

normalize_diag3() {
  local value="${1#[}"
  value="${value%]}"
  awk -v value="${value}" '
    BEGIN {
      count = split(value, component, ",")
      if (count != 3) {
        exit 1
      }
      for (i = 1; i <= 3; ++i) {
        gsub(/^[[:space:]]+|[[:space:]]+$/, "", component[i])
        if (component[i] !~ /^[-+]?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$/ || component[i] <= 0) {
          exit 1
        }
      }
      # ROS 2 parameter arrays must not mix integer and floating-point scalars.
      printf "[%.9f,%.9f,%.9f]", component[1], component[2], component[3]
    }
  '
}

cleanup_stack() {
  if [[ -z "${STACK_PID}" ]]; then
    return
  fi

  local child_pids=()
  while IFS= read -r child_pid; do
    if [[ -n "${child_pid}" ]]; then
      child_pids+=("${child_pid}")
    fi
  done < <(pgrep -P "${STACK_PID}" 2>/dev/null || true)

  if [[ "${CLEANUP_STACK_ON_EXIT}" != "false" ]]; then
    echo "[ude-test] stopping mocap/MAVROS/bridge/px4ctrl stack"
    kill -TERM -- "-${STACK_PID}" 2>/dev/null || kill -TERM "${STACK_PID}" 2>/dev/null || true
    for child_pid in "${child_pids[@]:-}"; do
      kill -TERM -- "-${child_pid}" 2>/dev/null || kill -TERM "${child_pid}" 2>/dev/null || true
    done
    sleep 1
    for child_pid in "${child_pids[@]:-}"; do
      kill -KILL -- "-${child_pid}" 2>/dev/null || kill -KILL "${child_pid}" 2>/dev/null || true
    done
    kill -KILL -- "-${STACK_PID}" 2>/dev/null || kill -KILL "${STACK_PID}" 2>/dev/null || true
  else
    echo "[ude-test] CLEANUP_STACK_ON_EXIT=false; leaving stack running"
    echo "[ude-test] stop it with: kill -TERM -- -${STACK_PID}"
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

if [[ -n "${TEST_UDE_ENABLE}" ]]; then
  if ! UDE_ENABLE_NORMALIZED="$(normalize_bool "${TEST_UDE_ENABLE}")"; then
    echo "[ude-test] invalid TEST_UDE_ENABLE=${TEST_UDE_ENABLE}; use true or false" >&2
    exit 1
  fi
fi

if [[ -n "${TEST_UDE_KD_DIAG}" ]]; then
  if ! UDE_KD_DIAG_NORMALIZED="$(normalize_diag3 "${TEST_UDE_KD_DIAG}")"; then
    echo "[ude-test] invalid TEST_UDE_KD_DIAG=${TEST_UDE_KD_DIAG}; use [x,y,z] with positive values" >&2
    exit 1
  fi
fi

if [[ -n "${TEST_TD_ENABLE}" ]]; then
  if ! TD_ENABLE_NORMALIZED="$(normalize_bool "${TEST_TD_ENABLE}")"; then
    echo "[ude-test] invalid TEST_TD_ENABLE=${TEST_TD_ENABLE}; use true or false" >&2
    exit 1
  fi
fi

if ! PHYSICAL_ENABLE_NORMALIZED="$(normalize_bool "${TEST_PHYSICAL_ENABLE}")"; then
  echo "[ude-test] invalid TEST_PHYSICAL_ENABLE=${TEST_PHYSICAL_ENABLE}; use true or false" >&2
  exit 1
fi

if ! awk -v mass="${TEST_PHYSICAL_MASS_KG}" 'BEGIN {exit !(mass ~ /^[0-9]+([.][0-9]+)?$/ && mass > 0)}'; then
  echo "[ude-test] invalid TEST_PHYSICAL_MASS_KG=${TEST_PHYSICAL_MASS_KG}; use a positive number in kg" >&2
  exit 1
fi

if [[ -n "${TEST_USE_BODYRATE_CTRL}" ]]; then
  if ! USE_BODYRATE_CTRL_NORMALIZED="$(normalize_bool "${TEST_USE_BODYRATE_CTRL}")"; then
    echo "[ude-test] invalid TEST_USE_BODYRATE_CTRL=${TEST_USE_BODYRATE_CTRL}; use true or false" >&2
    exit 1
  fi
fi

if [[ -n "${TEST_ATTITUDE_FEEDBACK_MODE}" ]]; then
  case "${TEST_ATTITUDE_FEEDBACK_MODE}" in
    full_quaternion|reduced_attitude)
      ATTITUDE_FEEDBACK_MODE_NORMALIZED="${TEST_ATTITUDE_FEEDBACK_MODE}"
      ;;
    *)
      echo "[ude-test] invalid TEST_ATTITUDE_FEEDBACK_MODE=${TEST_ATTITUDE_FEEDBACK_MODE}; use full_quaternion or reduced_attitude" >&2
      exit 1
      ;;
  esac
fi

if ! RECORD_BAG_NORMALIZED="$(normalize_bool "${TEST_RECORD_BAG}")"; then
  echo "[ude-test] invalid TEST_RECORD_BAG=${TEST_RECORD_BAG}; use true or false" >&2
  exit 1
fi

if ! ANALYZE_BAG_NORMALIZED="$(normalize_bool "${TEST_ANALYZE_BAG}")"; then
  echo "[ude-test] invalid TEST_ANALYZE_BAG=${TEST_ANALYZE_BAG}; use true or false" >&2
  exit 1
fi

if ! REQUIRE_COMPOSED_STATE_NORMALIZED="$(normalize_bool "${TEST_REQUIRE_COMPOSED_STATE}")"; then
  echo "[ude-test] invalid TEST_REQUIRE_COMPOSED_STATE=${TEST_REQUIRE_COMPOSED_STATE}; use true or false" >&2
  exit 1
fi

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
  if [[ "${START_PX4CTRL}" == "true" ]]; then
    if [[ -n "${UDE_ENABLE_NORMALIZED}" ]]; then
      export PX4CTRL_UDE_ENABLE="${UDE_ENABLE_NORMALIZED}"
      PARAM_OVERRIDES_VIA_LAUNCH=true
      echo "[ude-test] UDE mode: px4ctrl launch override ude.enable=${UDE_ENABLE_NORMALIZED}"
    fi
    if [[ -n "${UDE_KD_DIAG_NORMALIZED}" ]]; then
      export PX4CTRL_UDE_KD_DIAG="${UDE_KD_DIAG_NORMALIZED}"
      PARAM_OVERRIDES_VIA_LAUNCH=true
      echo "[ude-test] UDE damping: px4ctrl launch override ude.Kd_diag=${UDE_KD_DIAG_NORMALIZED}"
    fi
    if [[ -n "${TD_ENABLE_NORMALIZED}" ]]; then
      export PX4CTRL_TD_ENABLE="${TD_ENABLE_NORMALIZED}"
      PARAM_OVERRIDES_VIA_LAUNCH=true
      echo "[ude-test] TD mode: px4ctrl launch override td.enable=${TD_ENABLE_NORMALIZED}"
    fi
    export PX4CTRL_PHYSICAL_ENABLE="${PHYSICAL_ENABLE_NORMALIZED}"
    export PX4CTRL_PHYSICAL_MASS_KG="${TEST_PHYSICAL_MASS_KG}"
    PARAM_OVERRIDES_VIA_LAUNCH=true
    echo "[ude-test] physical setpoint: px4ctrl launch override physical_control.enable=${PHYSICAL_ENABLE_NORMALIZED}"
    echo "[ude-test] physical mass: px4ctrl launch override physical_control.mass_kg=${TEST_PHYSICAL_MASS_KG}kg"
    if [[ -n "${USE_BODYRATE_CTRL_NORMALIZED}" ]]; then
      export PX4CTRL_USE_BODYRATE_CTRL="${USE_BODYRATE_CTRL_NORMALIZED}"
      PARAM_OVERRIDES_VIA_LAUNCH=true
      if [[ "${USE_BODYRATE_CTRL_NORMALIZED}" == "true" ]]; then
        echo "[ude-test] setpoint output: px4ctrl launch override use_bodyrate_ctrl=true (bodyrate+thrust)"
      else
        echo "[ude-test] setpoint output: px4ctrl launch override use_bodyrate_ctrl=false (attitude+thrust)"
      fi
    fi
    if [[ -n "${ATTITUDE_FEEDBACK_MODE_NORMALIZED}" ]]; then
      export PX4CTRL_ATTITUDE_FEEDBACK_MODE="${ATTITUDE_FEEDBACK_MODE_NORMALIZED}"
      PARAM_OVERRIDES_VIA_LAUNCH=true
      echo "[ude-test] attitude feedback: px4ctrl launch override attitude.feedback_mode=${ATTITUDE_FEEDBACK_MODE_NORMALIZED}"
    fi
  fi
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
if [[ -n "${UDE_ENABLE_NORMALIZED}" ]]; then
  if [[ "${PARAM_OVERRIDES_VIA_LAUNCH}" == "true" && "${START_PX4CTRL}" == "true" ]]; then
    echo "[ude-test] UDE mode: using px4ctrl launch override already passed"
  else
    HELPER_ARGS+=("--ude-enable" "${UDE_ENABLE_NORMALIZED}")
    HELPER_ARGS+=("--ude-param-timeout-s" "${TEST_UDE_PARAM_TIMEOUT_S}")
    echo "[ude-test] UDE mode: helper will set /px4ctrl ude.enable=${UDE_ENABLE_NORMALIZED} after startup inputs are live"
  fi
else
  echo "[ude-test] UDE mode: using px4ctrl YAML/runtime default"
fi
if [[ -n "${TD_ENABLE_NORMALIZED}" ]]; then
  if [[ "${PARAM_OVERRIDES_VIA_LAUNCH}" == "true" && "${START_PX4CTRL}" == "true" ]]; then
    echo "[ude-test] TD mode: using px4ctrl launch override already passed"
  else
    HELPER_ARGS+=("--td-enable" "${TD_ENABLE_NORMALIZED}")
    HELPER_ARGS+=("--td-param-timeout-s" "${TEST_TD_PARAM_TIMEOUT_S}")
    echo "[ude-test] TD mode: helper will set /px4ctrl td.enable=${TD_ENABLE_NORMALIZED} after startup inputs are live"
  fi
else
  echo "[ude-test] TD mode: using px4ctrl YAML/runtime default"
fi
if [[ -n "${ATTITUDE_FEEDBACK_MODE_NORMALIZED}" ]]; then
  if [[ "${PARAM_OVERRIDES_VIA_LAUNCH}" == "true" && "${START_PX4CTRL}" == "true" ]]; then
    echo "[ude-test] attitude feedback: using px4ctrl launch override already passed"
  else
    HELPER_ARGS+=("--attitude-feedback-mode" "${ATTITUDE_FEEDBACK_MODE_NORMALIZED}")
    HELPER_ARGS+=("--attitude-param-timeout-s" "${TEST_ATTITUDE_PARAM_TIMEOUT_S}")
    echo "[ude-test] attitude feedback: helper will set /px4ctrl attitude.feedback_mode=${ATTITUDE_FEEDBACK_MODE_NORMALIZED} after startup inputs are live"
  fi
else
  echo "[ude-test] attitude feedback: using px4ctrl YAML/runtime default"
fi
if [[ "${TEST_ENTER_CMD}" == "true" ]]; then
  HELPER_ARGS+=("--enter-cmd")
fi
if [[ "${TEST_RUN_WAYPOINTS}" == "true" ]]; then
  HELPER_ARGS+=(
    "--run-waypoints"
    "--test-axis" "${TEST_AXIS}"
    "--wp-mode" "${TEST_WP_MODE}"
    "--wp-x-low" "${TEST_WP_X_LOW}"
    "--wp-x-high" "${TEST_WP_X_HIGH}"
    "--wp-y-low" "${TEST_WP_Y_LOW}"
    "--wp-y-high" "${TEST_WP_Y_HIGH}"
    "--wp-z-low" "${TEST_WP_Z_LOW}"
    "--wp-z-high" "${TEST_WP_Z_HIGH}"
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
  if [[ "${TEST_WP_MODE}" == "random" ]]; then
    echo "[ude-test] waypoint mode: random axis=${TEST_AXIS} step=[${TEST_WP_STEP_MIN_M},${TEST_WP_STEP_MAX_M}]m origin_limit=${TEST_WP_AXIS_LIMIT_M}m"
  else
    echo "[ude-test] waypoint mode: toggle axis=${TEST_AXIS} x=[${TEST_WP_X_LOW},${TEST_WP_X_HIGH}] y=[${TEST_WP_Y_LOW},${TEST_WP_Y_HIGH}] z=[${TEST_WP_Z_LOW},${TEST_WP_Z_HIGH}]"
  fi
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
if [[ "${REQUIRE_COMPOSED_STATE_NORMALIZED}" == "true" ]]; then
  HELPER_ARGS+=("--require-composed-state")
  echo "[ude-test] control feedback gate: require synchronized mocap p/v + FCU odom q/w"
else
  echo "[ude-test] control feedback gate: composed-state check disabled"
fi
if [[ "${RECORD_BAG_NORMALIZED}" == "true" ]]; then
  HELPER_ARGS+=("--record-bag" "--bag-root" "${TEST_BAG_ROOT}" "--bag-storage" "${TEST_BAG_STORAGE}")
  if [[ "${ANALYZE_BAG_NORMALIZED}" == "true" ]]; then
    HELPER_ARGS+=("--analyze-bag")
  fi
  echo "[ude-test] rosbag: enabled after AUTO_HOVER; root=${TEST_BAG_ROOT}; storage=${TEST_BAG_STORAGE}; analyze=${ANALYZE_BAG_NORMALIZED}"
else
  echo "[ude-test] rosbag: disabled (set TEST_RECORD_BAG=true to enable)"
fi

set +e
"${ROS_PYTHON}" "${WORKSPACE_DIR}/src/px4ctrl/scripts/ude_takeoff_hover_test.py" "${HELPER_ARGS[@]}"
HELPER_STATUS="$?"
set -e

exit "${HELPER_STATUS}"
