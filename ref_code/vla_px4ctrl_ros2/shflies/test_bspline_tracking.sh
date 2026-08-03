#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

START_STACK="${START_STACK:-true}"
START_PX4CTRL="${START_PX4CTRL:-true}"
STACK_STARTUP_WAIT_S="${STACK_STARTUP_WAIT_S:-18}"
QUIET_STACK_OUTPUT="${QUIET_STACK_OUTPUT:-true}"
CLEANUP_STACK_ON_EXIT="${CLEANUP_STACK_ON_EXIT:-true}"
STACK_LOG_DIR="${STACK_LOG_DIR:-${WORKSPACE_DIR}/log}"
ROS_PYTHON="${ROS_PYTHON:-/usr/bin/python3}"

BSPLINE_X_SPAN_M="${BSPLINE_X_SPAN_M:-16.7}"
BSPLINE_Y_SPAN_M="${BSPLINE_Y_SPAN_M:-3.8}"
BSPLINE_CENTER_X_OFFSET_M="${BSPLINE_CENTER_X_OFFSET_M:-0.0}"
BSPLINE_CENTER_Y_OFFSET_M="${BSPLINE_CENTER_Y_OFFSET_M:-0.0}"
BSPLINE_HEADING_OFFSET_DEG="${BSPLINE_HEADING_OFFSET_DEG:-0.0}"
BSPLINE_Z_MIN_M="${BSPLINE_Z_MIN_M:-1.0}"
BSPLINE_Z_MAX_M="${BSPLINE_Z_MAX_M:-1.5}"
BSPLINE_LAP_SPEEDS_MPS="${BSPLINE_LAP_SPEEDS_MPS:-8.0,8.0,8.0}"
BSPLINE_RATE_HZ="${BSPLINE_RATE_HZ:-100.0}"
BSPLINE_TRANSFER_SPEED_MPS="${BSPLINE_TRANSFER_SPEED_MPS:-1.0}"
BSPLINE_TRANSFER_ACCEL_MPS2="${BSPLINE_TRANSFER_ACCEL_MPS2:-1.0}"
# Values used by the completed 2026-07-22 three-lap 8 m/s baseline.
BSPLINE_MAX_LONG_ACCEL_MPS2="${BSPLINE_MAX_LONG_ACCEL_MPS2:-14.0}"
BSPLINE_MAX_DECEL_MPS2="${BSPLINE_MAX_DECEL_MPS2:-16.0}"
BSPLINE_MAX_LATERAL_ACCEL_MPS2="${BSPLINE_MAX_LATERAL_ACCEL_MPS2:-15.0}"
BSPLINE_MAX_JERK_MPS3="${BSPLINE_MAX_JERK_MPS3:-160.0}"
BSPLINE_MAX_SNAP_MPS4="${BSPLINE_MAX_SNAP_MPS4:-5000.0}"
BSPLINE_MAX_YAW_RATE_RADPS="${BSPLINE_MAX_YAW_RATE_RADPS:-5.0}"
BSPLINE_MAX_BODYRATE_FF_RADPS="${BSPLINE_MAX_BODYRATE_FF_RADPS:-8.0}"
BSPLINE_MAX_BODYRATE_DOT_FF_RADPS2="${BSPLINE_MAX_BODYRATE_DOT_FF_RADPS2:-100.0,100.0,50.0}"
BSPLINE_JERK_SMOOTHING_S="${BSPLINE_JERK_SMOOTHING_S:-0.05}"
BSPLINE_CONTROLLER_MAX_ANGLE_DEG="${BSPLINE_CONTROLLER_MAX_ANGLE_DEG:-65.0}"
BSPLINE_CONTROLLER_MAX_BODYRATE_X="${BSPLINE_CONTROLLER_MAX_BODYRATE_X:-8.0}"
BSPLINE_CONTROLLER_MAX_BODYRATE_Y="${BSPLINE_CONTROLLER_MAX_BODYRATE_Y:-8.0}"
BSPLINE_CONTROLLER_MAX_BODYRATE_Z="${BSPLINE_CONTROLLER_MAX_BODYRATE_Z:-8.0}"
BSPLINE_UDE_MAX_F_HAT="${BSPLINE_UDE_MAX_F_HAT:-20.0,20.0,10.0}"
BSPLINE_UDE_MAX_U_ACC="${BSPLINE_UDE_MAX_U_ACC:-25.0,25.0,15.0}"
BSPLINE_SAFETY_MARGIN_M="${BSPLINE_SAFETY_MARGIN_M:-0.20}"
BSPLINE_LIMIT_X_MIN="${BSPLINE_LIMIT_X_MIN:--2.0}"
BSPLINE_LIMIT_X_MAX="${BSPLINE_LIMIT_X_MAX:-15.1}"
BSPLINE_LIMIT_Y_MIN="${BSPLINE_LIMIT_Y_MIN:--2.1}"
BSPLINE_LIMIT_Y_MAX="${BSPLINE_LIMIT_Y_MAX:-2.1}"
BSPLINE_FINAL_HOLD_S="${BSPLINE_FINAL_HOLD_S:-2.0}"
BSPLINE_MAX_XY_ERROR_M="${BSPLINE_MAX_XY_ERROR_M:-1.5}"
BSPLINE_MAX_Z_ERROR_M="${BSPLINE_MAX_Z_ERROR_M:-0.6}"
BSPLINE_AUTO_CONFIRM="${BSPLINE_AUTO_CONFIRM:-false}"
BSPLINE_RECORD_BAG="${BSPLINE_RECORD_BAG:-true}"
BSPLINE_ANALYZE_BAG="${BSPLINE_ANALYZE_BAG:-true}"
BSPLINE_BAG_ROOT="${BSPLINE_BAG_ROOT:-${WORKSPACE_DIR}/bags}"
BSPLINE_BAG_STORAGE="${BSPLINE_BAG_STORAGE:-sqlite3}"
BSPLINE_BATTERY_MIN_V="${BSPLINE_BATTERY_MIN_V:-18.0}"
BSPLINE_BATTERY_MAX_V="${BSPLINE_BATTERY_MAX_V:-26.0}"
BSPLINE_BATTERY_TIMEOUT_S="${BSPLINE_BATTERY_TIMEOUT_S:-5.0}"

BSPLINE_PHYSICAL_ENABLE="${BSPLINE_PHYSICAL_ENABLE:-true}"
BSPLINE_PHYSICAL_MASS_KG="${BSPLINE_PHYSICAL_MASS_KG:-1.75}"
BSPLINE_BODY_RATE_FF_SCALE="${BSPLINE_BODY_RATE_FF_SCALE:-1.0}"
BSPLINE_ANGULAR_ACCEL_FF_SCALE="${BSPLINE_ANGULAR_ACCEL_FF_SCALE:-0.0}"
BSPLINE_MOCAP_MAX_PREDICTION_DT_S="${BSPLINE_MOCAP_MAX_PREDICTION_DT_S:-0.0}"
BSPLINE_CMD_MAX_VELOCITY_MPS="${BSPLINE_CMD_MAX_VELOCITY_MPS:-9.0}"
BSPLINE_CMD_MAX_ACCELERATION_MPS2="${BSPLINE_CMD_MAX_ACCELERATION_MPS2:-25.0}"
BSPLINE_CMD_MAX_JERK_MPS3="${BSPLINE_CMD_MAX_JERK_MPS3:-160.0}"
BSPLINE_CMD_MAX_SNAP_MPS4="${BSPLINE_CMD_MAX_SNAP_MPS4:-5000.0}"
BSPLINE_CMD_MAX_YAW_ACCEL_RADPS2="${BSPLINE_CMD_MAX_YAW_ACCEL_RADPS2:-20.0}"

STACK_PID=""
HELPER_STATUS=0

bool_is_true() {
  case "${1,,}" in
    1|true|yes|y|on) return 0 ;;
    *) return 1 ;;
  esac
}

cleanup_stack() {
  if [[ -z "${STACK_PID}" ]]; then
    return
  fi
  if [[ "${CLEANUP_STACK_ON_EXIT}" == "false" ]]; then
    echo "[bspline-test] CLEANUP_STACK_ON_EXIT=false; leaving stack active"
    echo "[bspline-test] stop it with: kill -TERM -- -${STACK_PID}"
    return
  fi
  echo "[bspline-test] stopping mocap/MAVROS/bridge/px4ctrl stack"
  kill -TERM -- "-${STACK_PID}" 2>/dev/null || kill -TERM "${STACK_PID}" 2>/dev/null || true
  sleep 1
  kill -KILL -- "-${STACK_PID}" 2>/dev/null || kill -KILL "${STACK_PID}" 2>/dev/null || true
}

on_interrupt() {
  echo "[bspline-test] interrupted"
  HELPER_STATUS=20
  exit 20
}

trap on_interrupt INT TERM
trap cleanup_stack EXIT

set +u
source /opt/ros/humble/setup.bash
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  set -u
  echo "[bspline-test] build first: colcon build --packages-select px4ctrl" >&2
  exit 1
fi
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

if [[ "${BSPLINE_PHYSICAL_ENABLE,,}" != "true" && "${BSPLINE_PHYSICAL_ENABLE}" != "1" ]]; then
  echo "[bspline-test] this test requires BSPLINE_PHYSICAL_ENABLE=true" >&2
  exit 1
fi

if [[ "${START_STACK}" == "true" ]]; then
  export START_PX4CTRL
  export TRAJ_PLANNER_ENABLE=false
  export PX4CTRL_PHYSICAL_ENABLE=true
  export PX4CTRL_PHYSICAL_MASS_KG="${BSPLINE_PHYSICAL_MASS_KG}"
  export PX4CTRL_PHYSICAL_BODY_RATE_FF_SCALE="${BSPLINE_BODY_RATE_FF_SCALE}"
  export PX4CTRL_PHYSICAL_ANGULAR_ACCEL_FF_SCALE="${BSPLINE_ANGULAR_ACCEL_FF_SCALE}"
  export PX4CTRL_MOCAP_MAX_PREDICTION_DT_S="${BSPLINE_MOCAP_MAX_PREDICTION_DT_S}"
  export PX4CTRL_USE_BODYRATE_CTRL=false
  export PX4CTRL_ATTITUDE_FEEDBACK_MODE=full_quaternion
  export PX4CTRL_CMD_FEEDFORWARD_ENABLE=true
  export PX4CTRL_CMD_FEEDFORWARD_MAX_VELOCITY="${BSPLINE_CMD_MAX_VELOCITY_MPS}"
  export PX4CTRL_CMD_FEEDFORWARD_MAX_ACCELERATION="${BSPLINE_CMD_MAX_ACCELERATION_MPS2}"
  export PX4CTRL_CMD_FEEDFORWARD_MAX_JERK="${BSPLINE_CMD_MAX_JERK_MPS3}"
  export PX4CTRL_CMD_FEEDFORWARD_MAX_SNAP="${BSPLINE_CMD_MAX_SNAP_MPS4}"
  export PX4CTRL_CMD_FEEDFORWARD_MAX_YAW_ACCEL="${BSPLINE_CMD_MAX_YAW_ACCEL_RADPS2}"
  export PX4CTRL_CONTROLLER_MAX_ANGLE_DEG="${BSPLINE_CONTROLLER_MAX_ANGLE_DEG}"
  export PX4CTRL_CONTROLLER_MAX_BODYRATE_X="${BSPLINE_CONTROLLER_MAX_BODYRATE_X}"
  export PX4CTRL_CONTROLLER_MAX_BODYRATE_Y="${BSPLINE_CONTROLLER_MAX_BODYRATE_Y}"
  export PX4CTRL_CONTROLLER_MAX_BODYRATE_Z="${BSPLINE_CONTROLLER_MAX_BODYRATE_Z}"
  export PX4CTRL_UDE_MAX_F_HAT="[${BSPLINE_UDE_MAX_F_HAT}]"
  export PX4CTRL_UDE_MAX_U_ACC="[${BSPLINE_UDE_MAX_U_ACC}]"
  export PX4CTRL_TD_ENABLE=false
  export PX4CTRL_LIMIT_X_MIN="${BSPLINE_LIMIT_X_MIN}"
  export PX4CTRL_LIMIT_X_MAX="${BSPLINE_LIMIT_X_MAX}"
  export PX4CTRL_LIMIT_Y_MIN="${BSPLINE_LIMIT_Y_MIN}"
  export PX4CTRL_LIMIT_Y_MAX="${BSPLINE_LIMIT_Y_MAX}"
  export MAVLINK_STREAM_RATE_REQUIRED=true

  mkdir -p "${STACK_LOG_DIR}"
  STACK_LOG_FILE="${STACK_LOG_DIR}/bspline_tracking_stack_$(date +%Y%m%d_%H%M%S).log"
  echo "[bspline-test] starting flight stack; log=${STACK_LOG_FILE}"
  if [[ "${QUIET_STACK_OUTPUT}" == "true" ]]; then
    setsid bash "${SCRIPT_DIR}/run_mocap_mavros.sh" >"${STACK_LOG_FILE}" 2>&1 &
  else
    setsid bash "${SCRIPT_DIR}/run_mocap_mavros.sh" &
  fi
  STACK_PID="$!"
  echo "[bspline-test] stack process group=${STACK_PID}"
  sleep "${STACK_STARTUP_WAIT_S}"
else
  echo "[bspline-test] START_STACK=false; checking the already-running px4ctrl configuration"
fi

echo "[bspline-test] path: span=${BSPLINE_X_SPAN_M}x${BSPLINE_Y_SPAN_M}m z=[${BSPLINE_Z_MIN_M},${BSPLINE_Z_MAX_M}]m heading_offset=${BSPLINE_HEADING_OFFSET_DEG}deg"
echo "[bspline-test] full field limits: x=[${BSPLINE_LIMIT_X_MIN},${BSPLINE_LIMIT_X_MAX}] y=[${BSPLINE_LIMIT_Y_MIN},${BSPLINE_LIMIT_Y_MAX}]"
echo "[bspline-test] laps: continuous speeds=${BSPLINE_LAP_SPEEDS_MPS}m/s rate=${BSPLINE_RATE_HZ}Hz no_inter_lap_hold=true"
echo "[bspline-test] control: physical=true mass=${BSPLINE_PHYSICAL_MASS_KG}kg attitude/full_quaternion PVA_feedforward=true body_rate_ff_scale=${BSPLINE_BODY_RATE_FF_SCALE} angular_accel_ff_scale=${BSPLINE_ANGULAR_ACCEL_FF_SCALE} planned_bodyrate_ff_limit=${BSPLINE_MAX_BODYRATE_FF_RADPS}rad/s max_angle=${BSPLINE_CONTROLLER_MAX_ANGLE_DEG}deg bodyrate_limit=[${BSPLINE_CONTROLLER_MAX_BODYRATE_X},${BSPLINE_CONTROLLER_MAX_BODYRATE_Y},${BSPLINE_CONTROLLER_MAX_BODYRATE_Z}]rad/s TD=false"
echo "[bspline-test] trajectory baseline: stable P/V/A with finite-difference jerk; snap is observational only"
echo "[bspline-test] mocap position prediction: max_dt=${BSPLINE_MOCAP_MAX_PREDICTION_DT_S}s (0 restores the successful baseline)"
echo "[bspline-test] outer limits: ude_f_hat=[${BSPLINE_UDE_MAX_F_HAT}]m/s2 ude_u_acc=[${BSPLINE_UDE_MAX_U_ACC}]m/s2 cmd_v/a/j/snap=[${BSPLINE_CMD_MAX_VELOCITY_MPS},${BSPLINE_CMD_MAX_ACCELERATION_MPS2},${BSPLINE_CMD_MAX_JERK_MPS3},${BSPLINE_CMD_MAX_SNAP_MPS4}]"
echo "[bspline-test] analytic feedforward limits: bodyrate=${BSPLINE_MAX_BODYRATE_FF_RADPS}rad/s bodyrate_dot=[${BSPLINE_MAX_BODYRATE_DOT_FF_RADPS2}]rad/s2"
echo "[bspline-test] physical battery gate: present=true voltage=[${BSPLINE_BATTERY_MIN_V},${BSPLINE_BATTERY_MAX_V}]V timeout=${BSPLINE_BATTERY_TIMEOUT_S}s"
echo "[bspline-test] bag: record=${BSPLINE_RECORD_BAG} analyze=${BSPLINE_ANALYZE_BAG} root=${BSPLINE_BAG_ROOT}"

HELPER_ARGS=(
  --publish-takeoff
  --require-composed-state
  --x-span-m "${BSPLINE_X_SPAN_M}"
  --y-span-m "${BSPLINE_Y_SPAN_M}"
  --center-x-offset-m "${BSPLINE_CENTER_X_OFFSET_M}"
  --center-y-offset-m "${BSPLINE_CENTER_Y_OFFSET_M}"
  --heading-offset-deg "${BSPLINE_HEADING_OFFSET_DEG}"
  --z-min-m "${BSPLINE_Z_MIN_M}"
  --z-max-m "${BSPLINE_Z_MAX_M}"
  --lap-speeds "${BSPLINE_LAP_SPEEDS_MPS}"
  --rate-hz "${BSPLINE_RATE_HZ}"
  --transfer-speed-mps "${BSPLINE_TRANSFER_SPEED_MPS}"
  --transfer-accel-mps2 "${BSPLINE_TRANSFER_ACCEL_MPS2}"
  --max-long-accel-mps2 "${BSPLINE_MAX_LONG_ACCEL_MPS2}"
  --max-decel-mps2 "${BSPLINE_MAX_DECEL_MPS2}"
  --max-lateral-accel-mps2 "${BSPLINE_MAX_LATERAL_ACCEL_MPS2}"
  --max-jerk-mps3 "${BSPLINE_MAX_JERK_MPS3}"
  --max-snap-mps4 "${BSPLINE_MAX_SNAP_MPS4}"
  --max-yaw-rate-radps "${BSPLINE_MAX_YAW_RATE_RADPS}"
  --max-bodyrate-ff-radps "${BSPLINE_MAX_BODYRATE_FF_RADPS}"
  --max-bodyrate-dot-ff-radps2 "${BSPLINE_MAX_BODYRATE_DOT_FF_RADPS2}"
  --jerk-smoothing-s "${BSPLINE_JERK_SMOOTHING_S}"
  --controller-max-angle-deg "${BSPLINE_CONTROLLER_MAX_ANGLE_DEG}"
  --controller-max-bodyrates "${BSPLINE_CONTROLLER_MAX_BODYRATE_X},${BSPLINE_CONTROLLER_MAX_BODYRATE_Y},${BSPLINE_CONTROLLER_MAX_BODYRATE_Z}"
  --body-rate-ff-scale "${BSPLINE_BODY_RATE_FF_SCALE}"
  --angular-accel-ff-scale "${BSPLINE_ANGULAR_ACCEL_FF_SCALE}"
  --ude-max-f-hat "${BSPLINE_UDE_MAX_F_HAT}"
  --ude-max-u-acc "${BSPLINE_UDE_MAX_U_ACC}"
  --safety-margin-m "${BSPLINE_SAFETY_MARGIN_M}"
  --final-hold-s "${BSPLINE_FINAL_HOLD_S}"
  --max-xy-error-m "${BSPLINE_MAX_XY_ERROR_M}"
  --max-z-error-m "${BSPLINE_MAX_Z_ERROR_M}"
  --battery-min-voltage-v "${BSPLINE_BATTERY_MIN_V}"
  --battery-max-voltage-v "${BSPLINE_BATTERY_MAX_V}"
  --battery-timeout-s "${BSPLINE_BATTERY_TIMEOUT_S}"
  --limit-x-min "${BSPLINE_LIMIT_X_MIN}"
  --limit-x-max "${BSPLINE_LIMIT_X_MAX}"
  --limit-y-min "${BSPLINE_LIMIT_Y_MIN}"
  --limit-y-max "${BSPLINE_LIMIT_Y_MAX}"
  --cmd-return-timeout 5.0
  --bag-root "${BSPLINE_BAG_ROOT}"
  --bag-storage "${BSPLINE_BAG_STORAGE}"
  --bag-topic /position_cmd_traj
  --bag-topic /px4ctrl/bspline_test/state
)

if bool_is_true "${BSPLINE_AUTO_CONFIRM}"; then
  HELPER_ARGS+=(--auto-confirm)
fi
if bool_is_true "${BSPLINE_RECORD_BAG}"; then
  HELPER_ARGS+=(--record-bag)
fi
if bool_is_true "${BSPLINE_ANALYZE_BAG}"; then
  HELPER_ARGS+=(--analyze-bag)
fi
HELPER_ARGS+=("$@")

set +e
"${ROS_PYTHON}" "${WORKSPACE_DIR}/src/px4ctrl/scripts/bspline_tracking_test.py" "${HELPER_ARGS[@]}"
HELPER_STATUS="$?"
set -e
exit "${HELPER_STATUS}"
