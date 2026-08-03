#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_DIR="$(cd "${WORKSPACE_DIR}/../.." && pwd)"
CAMERA_PATHS_FILE="${CAMERA_PATHS_FILE:-${SCRIPT_DIR}/record_camera_paths.env}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="${LOG_DIR:-${WORKSPACE_DIR}/log}"
LOCAL_INFERENCE_LOG="${LOCAL_INFERENCE_LOG:-${LOG_DIR}/pi05_local_inference_${RUN_ID}.log}"
mkdir -p "${LOG_DIR}"
exec > >(tee -a "${LOCAL_INFERENCE_LOG}") 2>&1
echo "[pi05-vla-local] log: ${LOCAL_INFERENCE_LOG}"

TOP_CAMERA_DEVICE_OVERRIDE="${TOP_CAMERA_DEVICE:-}"
if [[ -f "${CAMERA_PATHS_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${CAMERA_PATHS_FILE}"
fi
if [[ -n "${TOP_CAMERA_DEVICE_OVERRIDE}" ]]; then
  TOP_CAMERA_DEVICE="${TOP_CAMERA_DEVICE_OVERRIDE}"
fi

CONDA_ENV="${CONDA_ENV:-vla-drone-pi05}"
if [[ -z "${CONDA_SH:-}" ]]; then
  if [[ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]]; then
    CONDA_SH="${HOME}/miniconda3/etc/profile.d/conda.sh"
  elif [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
    CONDA_SH="${HOME}/miniforge3/etc/profile.d/conda.sh"
  else
    CONDA_SH=""
  fi
fi

POLICY_PATH="${POLICY_PATH:-${REPO_DIR}/outputs/train/vla_yellow_roll_pi05_embedfix_20260701_215106/checkpoints/050000/pretrained_model}"
TASK="${TASK:-Pick up the yellow paper roll from the black platform and place it into the white box}"
POLICY_DEVICE="${POLICY_DEVICE:-cuda}"
POLICY_DTYPE="${POLICY_DTYPE:-bfloat16}"
POLICY_FPS="${POLICY_FPS:-5}"
DURATION_S="${DURATION_S:-120}"
DRY_RUN="${DRY_RUN:-false}"

NX_CAMERA_SERVER="${NX_CAMERA_SERVER:-10.1.1.23}"
NX_CAMERA_PORT="${NX_CAMERA_PORT:-5556}"
CAMERA_WIDTH="${CAMERA_WIDTH:-640}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-480}"
CAMERA_FPS="${CAMERA_FPS:-30}"
CAMERA_WARMUP_S="${CAMERA_WARMUP_S:-5}"
TOP_CAMERA_DEVICE="${TOP_CAMERA_DEVICE:-/dev/v4l/by-path/pci-0000:80:14.0-usb-0:9.1:1.0-video-index0}"
TOP_CAMERA_FOURCC="${TOP_CAMERA_FOURCC:-MJPG}"
TOP_CAMERA_WARMUP_S="${TOP_CAMERA_WARMUP_S:-5}"
ZMQ_TIMEOUT_MS="${ZMQ_TIMEOUT_MS:-8000}"

NOKOV_POSE_TOPIC="${NOKOV_POSE_TOPIC:-/mavros/vision_pose/pose}"
MAVROS_SETPOINT_TOPIC="${MAVROS_SETPOINT_TOPIC:-/position_cmd}"
PX4CTRL_STATE_TOPIC="${PX4CTRL_STATE_TOPIC:-/px4ctrl/state}"
GRIPPER_TOPIC="${GRIPPER_TOPIC:-/gripper/command}"
GRIPPER_FEEDBACK_TOPIC="${GRIPPER_FEEDBACK_TOPIC:-/gripper/feedback}"
HLS_STATUS_SNAPSHOT_TOPIC="${HLS_STATUS_SNAPSHOT_TOPIC:-/hls_gripper/status_snapshot}"

WAIT_FOR_CMD_CTRL="${WAIT_FOR_CMD_CTRL:-true}"
WAIT_CMD_CTRL_TIMEOUT_S="${WAIT_CMD_CTRL_TIMEOUT_S:-90}"
HOLD_BEFORE_CMD_CTRL="${HOLD_BEFORE_CMD_CTRL:-true}"
HOLD_RATE_HZ="${HOLD_RATE_HZ:-20}"
STATE_TIMEOUT_S="${STATE_TIMEOUT_S:-0.5}"

RTC_ENABLED="${RTC_ENABLED:-true}"
RTC_EXECUTION_HORIZON="${RTC_EXECUTION_HORIZON:-5}"
ACTION_QUEUE_REFRESH_THRESHOLD="${ACTION_QUEUE_REFRESH_THRESHOLD:-10}"
HLS_ASSIST_ENABLE="${HLS_ASSIST_ENABLE:-true}"
HLS_STATUS_TIMEOUT_S="${HLS_STATUS_TIMEOUT_S:-0.3}"
HLS_ASSIST_CENTER_DEADBAND_M="${HLS_ASSIST_CENTER_DEADBAND_M:-0.05}"
HLS_ASSIST_CENTER_VMAX_MPS="${HLS_ASSIST_CENTER_VMAX_MPS:-0.05}"
HLS_ASSIST_CENTER_OFFSET_MAX_M="${HLS_ASSIST_CENTER_OFFSET_MAX_M:-0.30}"
HLS_ASSIST_SINGLE_CONTACT_VMAX_MPS="${HLS_ASSIST_SINGLE_CONTACT_VMAX_MPS:-0.04}"
HLS_ASSIST_SINGLE_CONTACT_OFFSET_MAX_M="${HLS_ASSIST_SINGLE_CONTACT_OFFSET_MAX_M:-0.12}"
ACTION_LOG_PERIOD_S="${ACTION_LOG_PERIOD_S:-1.0}"
ACTION_FILTER_ENABLE="${ACTION_FILTER_ENABLE:-true}"
ACTION_FILTER_XY_RATE_LIMIT_MPS="${ACTION_FILTER_XY_RATE_LIMIT_MPS:-0.15}"
ACTION_FILTER_Z_RATE_LIMIT_MPS="${ACTION_FILTER_Z_RATE_LIMIT_MPS:-0.10}"
ACTION_FILTER_YAW_RATE_LIMIT_RADPS="${ACTION_FILTER_YAW_RATE_LIMIT_RADPS:-0.15}"
ACTION_FILTER_LPF_TAU_S="${ACTION_FILTER_LPF_TAU_S:-0.25}"

if [[ ! -d "${POLICY_PATH}" ]]; then
  echo "[pi05-vla-local] policy path does not exist: ${POLICY_PATH}" >&2
  exit 1
fi
if [[ ! -e "${TOP_CAMERA_DEVICE}" ]]; then
  echo "[pi05-vla-local] top camera path does not exist: ${TOP_CAMERA_DEVICE}" >&2
  echo "[pi05-vla-local] Check: ls -l /dev/v4l/by-path/ /dev/video*" >&2
  exit 1
fi

if [[ -n "${CONDA_SH}" && -f "${CONDA_SH}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${CONDA_SH}"
  conda activate "${CONDA_ENV}"
  set -u
fi

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi
set -u

cd "${REPO_DIR}"
export PYTHONPATH="${REPO_DIR}/src:${PYTHONPATH:-}"
export HF_HOME="${HF_HOME:-${HOME}/.cache/huggingface}"

CAMERAS_CONFIG=$(
  cat <<EOF
{
  front: {type: zmq, server_address: "${NX_CAMERA_SERVER}", port: ${NX_CAMERA_PORT}, camera_name: "front", width: ${CAMERA_WIDTH}, height: ${CAMERA_HEIGHT}, fps: ${CAMERA_FPS}, timeout_ms: ${ZMQ_TIMEOUT_MS}, warmup_s: ${CAMERA_WARMUP_S}},
  down: {type: zmq, server_address: "${NX_CAMERA_SERVER}", port: ${NX_CAMERA_PORT}, camera_name: "down", width: ${CAMERA_WIDTH}, height: ${CAMERA_HEIGHT}, fps: ${CAMERA_FPS}, timeout_ms: ${ZMQ_TIMEOUT_MS}, warmup_s: ${CAMERA_WARMUP_S}},
  top: {type: opencv, index_or_path: "${TOP_CAMERA_DEVICE}", width: ${CAMERA_WIDTH}, height: ${CAMERA_HEIGHT}, fps: ${CAMERA_FPS}, fourcc: "${TOP_CAMERA_FOURCC}", warmup_s: ${TOP_CAMERA_WARMUP_S}}
}
EOF
)

echo "[pi05-vla-local] policy: ${POLICY_PATH}"
echo "[pi05-vla-local] task: ${TASK}"
echo "[pi05-vla-local] nx cameras: ${NX_CAMERA_SERVER}:${NX_CAMERA_PORT} front/down"
echo "[pi05-vla-local] top camera: ${TOP_CAMERA_DEVICE}"
echo "[pi05-vla-local] dry_run=${DRY_RUN} wait_for_cmd_ctrl=${WAIT_FOR_CMD_CTRL} hls_assist=${HLS_ASSIST_ENABLE}"
echo "[pi05-vla-local] action: fps=${POLICY_FPS} filter=${ACTION_FILTER_ENABLE} xy_rate=${ACTION_FILTER_XY_RATE_LIMIT_MPS} z_rate=${ACTION_FILTER_Z_RATE_LIMIT_MPS} yaw_rate=${ACTION_FILTER_YAW_RATE_LIMIT_RADPS} lpf_tau=${ACTION_FILTER_LPF_TAU_S}"
echo "[pi05-vla-local] rtc: enabled=${RTC_ENABLED} execution_horizon=${RTC_EXECUTION_HORIZON} refresh_threshold=${ACTION_QUEUE_REFRESH_THRESHOLD}"

python3 examples/rtc/eval_vla_drone_pi05.py \
  --policy.path="${POLICY_PATH}" \
  --policy.device="${POLICY_DEVICE}" \
  --policy.dtype="${POLICY_DTYPE}" \
  --device="${POLICY_DEVICE}" \
  --rtc.enabled="${RTC_ENABLED}" \
  --rtc.execution_horizon="${RTC_EXECUTION_HORIZON}" \
  --robot.type=vla_drone \
  --robot.nokov_pose_topic="${NOKOV_POSE_TOPIC}" \
  --robot.mavros_setpoint_topic="${MAVROS_SETPOINT_TOPIC}" \
  --robot.max_pose_age_s=0.5 \
  --robot.camera_max_age_ms=1500 \
  --robot.send_pose_actions=true \
  --robot.use_ros_gripper=true \
  --robot.gripper_command_topic="${GRIPPER_TOPIC}" \
  --robot.gripper_feedback_topic="${GRIPPER_FEEDBACK_TOPIC}" \
  --robot.gripper_feedback_timeout_s=0.5 \
  --robot.safe_open_gripper_on_disconnect=false \
  --robot.safe_open_gripper_after_episode=false \
  --robot.hls_assist_enable="${HLS_ASSIST_ENABLE}" \
  --robot.hls_status_snapshot_topic="${HLS_STATUS_SNAPSHOT_TOPIC}" \
  --robot.hls_status_timeout_s="${HLS_STATUS_TIMEOUT_S}" \
  --robot.hls_assist_center_deadband_m="${HLS_ASSIST_CENTER_DEADBAND_M}" \
  --robot.hls_assist_center_vmax_mps="${HLS_ASSIST_CENTER_VMAX_MPS}" \
  --robot.hls_assist_center_offset_max_m="${HLS_ASSIST_CENTER_OFFSET_MAX_M}" \
  --robot.hls_assist_single_contact_vmax_mps="${HLS_ASSIST_SINGLE_CONTACT_VMAX_MPS}" \
  --robot.hls_assist_single_contact_offset_max_m="${HLS_ASSIST_SINGLE_CONTACT_OFFSET_MAX_M}" \
  --robot.cameras="${CAMERAS_CONFIG}" \
  --px4ctrl_state_topic="${PX4CTRL_STATE_TOPIC}" \
  --state_timeout_s="${STATE_TIMEOUT_S}" \
  --wait_for_cmd_ctrl="${WAIT_FOR_CMD_CTRL}" \
  --wait_cmd_ctrl_timeout_s="${WAIT_CMD_CTRL_TIMEOUT_S}" \
  --hold_before_cmd_ctrl="${HOLD_BEFORE_CMD_CTRL}" \
  --hold_rate_hz="${HOLD_RATE_HZ}" \
  --action_log_period_s="${ACTION_LOG_PERIOD_S}" \
  --action_filter_enable="${ACTION_FILTER_ENABLE}" \
  --action_filter_xy_rate_limit_mps="${ACTION_FILTER_XY_RATE_LIMIT_MPS}" \
  --action_filter_z_rate_limit_mps="${ACTION_FILTER_Z_RATE_LIMIT_MPS}" \
  --action_filter_yaw_rate_limit_radps="${ACTION_FILTER_YAW_RATE_LIMIT_RADPS}" \
  --action_filter_lpf_tau_s="${ACTION_FILTER_LPF_TAU_S}" \
  --dry_run="${DRY_RUN}" \
  --fps="${POLICY_FPS}" \
  --action_queue_size_to_get_new_actions="${ACTION_QUEUE_REFRESH_THRESHOLD}" \
  --duration="${DURATION_S}" \
  --task="${TASK}"
