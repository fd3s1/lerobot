#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_DIR="$(cd "${WORKSPACE_DIR}/../.." && pwd)"
CAMERA_PATHS_FILE="${CAMERA_PATHS_FILE:-${SCRIPT_DIR}/record_camera_paths.env}"

CAMERA_ENV_OVERRIDE_NAMES=(
  FRONT_CAMERA
  DOWN_CAMERA
  CAMERA_WIDTH
  CAMERA_HEIGHT
  CAMERA_FPS
  CAMERA_FOURCC
  CAMERA_WARMUP_S
  TOP_CAMERA_ENABLE
  TOP_CAMERA_TYPE
  TOP_CAMERA
  TOP_CAMERA_SERVER
  TOP_CAMERA_PORT
  TOP_CAMERA_NAME
  TOP_CAMERA_WIDTH
  TOP_CAMERA_HEIGHT
  TOP_CAMERA_FPS
  TOP_CAMERA_FOURCC
  TOP_CAMERA_WARMUP_S
  TOP_CAMERA_TIMEOUT_MS
  SIDE_CAMERA_ENABLE
  SIDE_CAMERA_TYPE
  SIDE_CAMERA
  SIDE_CAMERA_SERVER
  SIDE_CAMERA_PORT
  SIDE_CAMERA_NAME
  SIDE_CAMERA_WIDTH
  SIDE_CAMERA_HEIGHT
  SIDE_CAMERA_FPS
  SIDE_CAMERA_FOURCC
  SIDE_CAMERA_WARMUP_S
  SIDE_CAMERA_TIMEOUT_MS
)
declare -A CAMERA_ENV_OVERRIDES=()
for env_name in "${CAMERA_ENV_OVERRIDE_NAMES[@]}"; do
  if [[ -v "${env_name}" ]]; then
    CAMERA_ENV_OVERRIDES["${env_name}"]="${!env_name}"
  fi
done

if [[ -f "${CAMERA_PATHS_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${CAMERA_PATHS_FILE}"
fi

for env_name in "${!CAMERA_ENV_OVERRIDES[@]}"; do
  printf -v "${env_name}" '%s' "${CAMERA_ENV_OVERRIDES[${env_name}]}"
done

CONDA_ENV="${CONDA_ENV:-vla-drone-v044}"
CONDA_SH="${CONDA_SH:-${HOME}/miniforge3/etc/profile.d/conda.sh}"

RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
DATASET_PREFIX="${DATASET_PREFIX:-vla_drone_grasp}"
DATASET_BASE_DIR="${DATASET_BASE_DIR:-${HOME}/vla_drone/data}"
REPO_OWNER="${REPO_OWNER:-fd3s1}"
RESUME_DATASET="${RESUME_DATASET:-false}"
RESUME_LATEST="${RESUME_LATEST:-false}"

NOKOV_POSE_TOPIC="${NOKOV_POSE_TOPIC:-/mavros/vision_pose/pose}"
MAVROS_SETPOINT_TOPIC="${MAVROS_SETPOINT_TOPIC:-/position_cmd}"
EXPERT_POSE_TOPIC="${EXPERT_POSE_TOPIC:-/px4ctrl/expert_pose}"
GRIPPER_TOPIC="${GRIPPER_TOPIC:-/gripper/command}"
GRIPPER_COMMAND_PAIR_TOPIC="${GRIPPER_COMMAND_PAIR_TOPIC:-/gripper/command_pair}"
GRIPPER_FEEDBACK_TOPIC="${GRIPPER_FEEDBACK_TOPIC:-/gripper/feedback}"
USE_ROS_GRIPPER="${USE_ROS_GRIPPER:-false}"
GRIPPER_FEEDBACK_TIMEOUT_S="${GRIPPER_FEEDBACK_TIMEOUT_S:-0.5}"
GRIPPER_PORT="${GRIPPER_PORT:-/dev/ttyACM1}"
GRIPPER_LEFT_INVERTED="${GRIPPER_LEFT_INVERTED:-true}"
GRIPPER_RIGHT_INVERTED="${GRIPPER_RIGHT_INVERTED:-true}"
SAFE_OPEN_GRIPPER_ON_DISCONNECT="${SAFE_OPEN_GRIPPER_ON_DISCONNECT:-true}"
SAFE_OPEN_GRIPPER_AFTER_EPISODE="${SAFE_OPEN_GRIPPER_AFTER_EPISODE:-true}"
DISCONNECT_GRIPPER_OPEN_POSITION="${DISCONNECT_GRIPPER_OPEN_POSITION:-100.0}"
DISCONNECT_GRIPPER_REPEATS="${DISCONNECT_GRIPPER_REPEATS:-3}"
DISCONNECT_GRIPPER_SETTLE_S="${DISCONNECT_GRIPPER_SETTLE_S:-0.5}"

FRONT_CAMERA="${FRONT_CAMERA:-/dev/video2}"
DOWN_CAMERA="${DOWN_CAMERA:-/dev/video0}"
CAMERA_WIDTH="${CAMERA_WIDTH:-640}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-480}"
CAMERA_FPS="${CAMERA_FPS:-20}"
CAMERA_FOURCC="${CAMERA_FOURCC:-MJPG}"
CAMERA_WARMUP_S="${CAMERA_WARMUP_S:-3}"
LEROBOT_OPENCV_MAX_CONSECUTIVE_READ_FAILURES="${LEROBOT_OPENCV_MAX_CONSECUTIVE_READ_FAILURES:-80}"
TOP_CAMERA_ENABLE="${TOP_CAMERA_ENABLE:-false}"
TOP_CAMERA_TYPE="${TOP_CAMERA_TYPE:-zmq}"
TOP_CAMERA="${TOP_CAMERA:-}"
TOP_CAMERA_SERVER="${TOP_CAMERA_SERVER:-}"
TOP_CAMERA_PORT="${TOP_CAMERA_PORT:-5555}"
TOP_CAMERA_NAME="${TOP_CAMERA_NAME:-top}"
TOP_CAMERA_WIDTH="${TOP_CAMERA_WIDTH:-${CAMERA_WIDTH}}"
TOP_CAMERA_HEIGHT="${TOP_CAMERA_HEIGHT:-${CAMERA_HEIGHT}}"
TOP_CAMERA_FPS="${TOP_CAMERA_FPS:-${CAMERA_FPS}}"
TOP_CAMERA_FOURCC="${TOP_CAMERA_FOURCC:-${CAMERA_FOURCC}}"
TOP_CAMERA_WARMUP_S="${TOP_CAMERA_WARMUP_S:-${CAMERA_WARMUP_S}}"
TOP_CAMERA_TIMEOUT_MS="${TOP_CAMERA_TIMEOUT_MS:-3000}"
SIDE_CAMERA_ENABLE="${SIDE_CAMERA_ENABLE:-false}"
SIDE_CAMERA_TYPE="${SIDE_CAMERA_TYPE:-zmq}"
SIDE_CAMERA="${SIDE_CAMERA:-}"
SIDE_CAMERA_SERVER="${SIDE_CAMERA_SERVER:-${TOP_CAMERA_SERVER}}"
SIDE_CAMERA_PORT="${SIDE_CAMERA_PORT:-5555}"
SIDE_CAMERA_NAME="${SIDE_CAMERA_NAME:-side}"
SIDE_CAMERA_WIDTH="${SIDE_CAMERA_WIDTH:-${CAMERA_WIDTH}}"
SIDE_CAMERA_HEIGHT="${SIDE_CAMERA_HEIGHT:-${CAMERA_HEIGHT}}"
SIDE_CAMERA_FPS="${SIDE_CAMERA_FPS:-${CAMERA_FPS}}"
SIDE_CAMERA_FOURCC="${SIDE_CAMERA_FOURCC:-${CAMERA_FOURCC}}"
SIDE_CAMERA_WARMUP_S="${SIDE_CAMERA_WARMUP_S:-${CAMERA_WARMUP_S}}"
SIDE_CAMERA_TIMEOUT_MS="${SIDE_CAMERA_TIMEOUT_MS:-3000}"

DATASET_FPS="${DATASET_FPS:-20}"
NUM_EPISODES="${NUM_EPISODES:-1}"
EPISODE_TIME_S="${EPISODE_TIME_S:-30}"
RECORD_PREWARM_STEPS="${RECORD_PREWARM_STEPS:-2}"
START_GATE_TOPIC="${START_GATE_TOPIC-/px4ctrl/state}"
START_GATE_VALUE="${START_GATE_VALUE:-AUTO_HOVER}"
START_GATE_STABLE_S="${START_GATE_STABLE_S:-3.0}"
START_GATE_TIMEOUT_S="${START_GATE_TIMEOUT_S:-0.0}"
START_GATE_STATUS_LOG_S="${START_GATE_STATUS_LOG_S:-0.0}"
STOP_GATE_TOPIC="${STOP_GATE_TOPIC:-}"
STOP_GATE_VALUE="${STOP_GATE_VALUE:-STOP}"
DATASET_STATUS_TOPIC="${DATASET_STATUS_TOPIC:-}"
RESET_TIME_S="${RESET_TIME_S:-10}"
TASK="${TASK:-Fly to the target and operate the gripper}"
PUSH_TO_HUB="${PUSH_TO_HUB:-false}"
DATASET_VIDEO="${DATASET_VIDEO:-true}"
DATASET_VCODEC="${DATASET_VCODEC:-h264}"
STREAMING_ENCODING="${STREAMING_ENCODING:-false}"
ENCODER_THREADS="${ENCODER_THREADS:-2}"
ENCODER_QUEUE_MAXSIZE="${ENCODER_QUEUE_MAXSIZE:-240}"
IMAGE_WRITER_PROCESSES="${IMAGE_WRITER_PROCESSES:-0}"
IMAGE_WRITER_THREADS_PER_CAMERA="${IMAGE_WRITER_THREADS_PER_CAMERA:-2}"
PLAY_SOUNDS="${PLAY_SOUNDS:-false}"

ROBOT_MAX_POSE_AGE_S="${ROBOT_MAX_POSE_AGE_S:-2.0}"
ROBOT_CAMERA_MAX_AGE_MS="${ROBOT_CAMERA_MAX_AGE_MS:-1500}"
TELEOP_STARTUP_TIMEOUT_S="${TELEOP_STARTUP_TIMEOUT_S:-2.0}"
TELEOP_MAX_POSE_AGE_S="${TELEOP_MAX_POSE_AGE_S:-0.5}"

bool_is_true() {
  case "${1,,}" in
    1|true|yes|y|on) return 0 ;;
    *) return 1 ;;
  esac
}

require_camera_path() {
  local label="$1"
  local path="$2"
  if [[ ! -e "${path}" ]]; then
    echo "[record-vla-dataset] ${label} camera path does not exist: ${path}" >&2
    echo "[record-vla-dataset] Check: ls -l /dev/v4l/by-path/ /dev/video*" >&2
    exit 1
  fi
}

select_dataset_for_resume() {
  if [[ -d "${DATASET_BASE_DIR}" ]]; then
    mapfile -t dataset_dirs < <(
      find "${DATASET_BASE_DIR}" -maxdepth 1 -type d -name "${DATASET_PREFIX}_*" -printf "%T@ %p\n" 2>/dev/null |
        sort -nr |
        awk '{sub($1 FS, ""); print}'
    )
  else
    dataset_dirs=()
  fi

  if [[ "${#dataset_dirs[@]}" -eq 0 ]]; then
    echo "[record-vla-dataset] no datasets found under ${DATASET_BASE_DIR}/${DATASET_PREFIX}_*" >&2
    exit 1
  fi

  if ! [[ -t 0 ]]; then
    echo "[record-vla-dataset] resume selection needs an interactive terminal." >&2
    echo "[record-vla-dataset] Set DATASET_NAME, DATASET_ROOT, or RESUME_LATEST=true." >&2
    exit 1
  fi

  echo "[record-vla-dataset] select dataset to resume:"
  for idx in "${!dataset_dirs[@]}"; do
    printf "  [%d] %s\n" "$((idx + 1))" "$(basename "${dataset_dirs[$idx]}")"
  done

  local selection
  read -r -p "[record-vla-dataset] dataset number: " selection
  if ! [[ "${selection}" =~ ^[0-9]+$ ]] ||
     ((selection < 1 || selection > ${#dataset_dirs[@]})); then
    echo "[record-vla-dataset] invalid selection: ${selection}" >&2
    exit 1
  fi

  DATASET_ROOT="${dataset_dirs[$((selection - 1))]}"
  DATASET_NAME="$(basename "${DATASET_ROOT}")"
}

if bool_is_true "${RESUME_LATEST}"; then
  RESUME_DATASET=true
  if [[ -d "${DATASET_BASE_DIR}" ]]; then
    DATASET_ROOT="$(
      find "${DATASET_BASE_DIR}" -maxdepth 1 -type d -name "${DATASET_PREFIX}_*" -printf "%T@ %p\n" 2>/dev/null |
        sort -nr |
        awk 'NR == 1 {sub($1 FS, ""); print}'
    )"
  else
    DATASET_ROOT=""
  fi
  if [[ -z "${DATASET_ROOT}" ]]; then
    echo "[record-vla-dataset] no datasets found under ${DATASET_BASE_DIR}/${DATASET_PREFIX}_*" >&2
    exit 1
  fi
  DATASET_NAME="$(basename "${DATASET_ROOT}")"
elif bool_is_true "${RESUME_DATASET}"; then
  if [[ -n "${DATASET_ROOT:-}" ]]; then
    DATASET_NAME="${DATASET_NAME:-$(basename "${DATASET_ROOT}")}"
  elif [[ -n "${DATASET_NAME:-}" ]]; then
    DATASET_ROOT="${DATASET_BASE_DIR}/${DATASET_NAME}"
  else
    select_dataset_for_resume
  fi
else
  DATASET_NAME="${DATASET_NAME:-${DATASET_PREFIX}_${RUN_ID}}"
  DATASET_ROOT="${DATASET_ROOT:-${DATASET_BASE_DIR}/${DATASET_NAME}}"
fi

REPO_ID="${REPO_ID:-${REPO_OWNER}/${DATASET_NAME}}"

if bool_is_true "${RESUME_DATASET}" && [[ ! -d "${DATASET_ROOT}" ]]; then
  echo "[record-vla-dataset] resume dataset root does not exist: ${DATASET_ROOT}" >&2
  exit 1
fi

if bool_is_true "${RESUME_DATASET}"; then
  RESUME_DATASET=true
else
  RESUME_DATASET=false
fi

if bool_is_true "${RESUME_LATEST}"; then
  RESUME_LATEST=true
else
  RESUME_LATEST=false
fi

if [[ -f "${CONDA_SH}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${CONDA_SH}"
  conda activate "${CONDA_ENV}"
  set -u
elif ! command -v lerobot-record >/dev/null 2>&1; then
  echo "[record-vla-dataset] conda setup not found: ${CONDA_SH}" >&2
  echo "[record-vla-dataset] Either run from an activated ${CONDA_ENV} env or set CONDA_SH." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

cd "${REPO_DIR}"

require_camera_path "front" "${FRONT_CAMERA}"
require_camera_path "down" "${DOWN_CAMERA}"
if bool_is_true "${TOP_CAMERA_ENABLE}" && [[ "${TOP_CAMERA_TYPE}" == "opencv" ]]; then
  require_camera_path "top" "${TOP_CAMERA}"
fi
if bool_is_true "${SIDE_CAMERA_ENABLE}" && [[ "${SIDE_CAMERA_TYPE}" == "opencv" ]]; then
  require_camera_path "side" "${SIDE_CAMERA}"
fi

CAMERAS_CONFIG=$(
  cat <<EOF
{
  front: {type: opencv, index_or_path: "${FRONT_CAMERA}", width: ${CAMERA_WIDTH}, height: ${CAMERA_HEIGHT}, fps: ${CAMERA_FPS}, fourcc: "${CAMERA_FOURCC}", warmup_s: ${CAMERA_WARMUP_S}},
  down: {type: opencv, index_or_path: "${DOWN_CAMERA}", width: ${CAMERA_WIDTH}, height: ${CAMERA_HEIGHT}, fps: ${CAMERA_FPS}, fourcc: "${CAMERA_FOURCC}", warmup_s: ${CAMERA_WARMUP_S}}
EOF
)

if bool_is_true "${TOP_CAMERA_ENABLE}"; then
  case "${TOP_CAMERA_TYPE}" in
    zmq)
      if [[ -z "${TOP_CAMERA_SERVER}" ]]; then
        echo "[record-vla-dataset] TOP_CAMERA_ENABLE=true but TOP_CAMERA_SERVER is empty." >&2
        echo "[record-vla-dataset] Set TOP_CAMERA_SERVER to the IP of the computer running start_top_camera_zmq.sh." >&2
        exit 1
      fi
      CAMERAS_CONFIG+=$(
        cat <<EOF
,
  top: {type: zmq, server_address: "${TOP_CAMERA_SERVER}", port: ${TOP_CAMERA_PORT}, camera_name: "${TOP_CAMERA_NAME}", width: ${TOP_CAMERA_WIDTH}, height: ${TOP_CAMERA_HEIGHT}, fps: ${TOP_CAMERA_FPS}, timeout_ms: ${TOP_CAMERA_TIMEOUT_MS}, warmup_s: ${TOP_CAMERA_WARMUP_S}}
EOF
      )
      ;;
    opencv)
      if [[ -z "${TOP_CAMERA}" ]]; then
        echo "[record-vla-dataset] TOP_CAMERA_ENABLE=true TOP_CAMERA_TYPE=opencv but TOP_CAMERA is empty." >&2
        exit 1
      fi
      CAMERAS_CONFIG+=$(
        cat <<EOF
,
  top: {type: opencv, index_or_path: "${TOP_CAMERA}", width: ${TOP_CAMERA_WIDTH}, height: ${TOP_CAMERA_HEIGHT}, fps: ${TOP_CAMERA_FPS}, fourcc: "${TOP_CAMERA_FOURCC}", warmup_s: ${TOP_CAMERA_WARMUP_S}}
EOF
      )
      ;;
    *)
      echo "[record-vla-dataset] Unsupported TOP_CAMERA_TYPE=${TOP_CAMERA_TYPE}; use zmq or opencv." >&2
      exit 1
      ;;
  esac
fi

if bool_is_true "${SIDE_CAMERA_ENABLE}"; then
  case "${SIDE_CAMERA_TYPE}" in
    zmq)
      if [[ -z "${SIDE_CAMERA_SERVER}" ]]; then
        echo "[record-vla-dataset] SIDE_CAMERA_ENABLE=true but SIDE_CAMERA_SERVER is empty." >&2
        echo "[record-vla-dataset] Set SIDE_CAMERA_SERVER to the IP of the computer running the side/top ZMQ server." >&2
        exit 1
      fi
      CAMERAS_CONFIG+=$(
        cat <<EOF
,
  side: {type: zmq, server_address: "${SIDE_CAMERA_SERVER}", port: ${SIDE_CAMERA_PORT}, camera_name: "${SIDE_CAMERA_NAME}", width: ${SIDE_CAMERA_WIDTH}, height: ${SIDE_CAMERA_HEIGHT}, fps: ${SIDE_CAMERA_FPS}, timeout_ms: ${SIDE_CAMERA_TIMEOUT_MS}, warmup_s: ${SIDE_CAMERA_WARMUP_S}}
EOF
      )
      ;;
    opencv)
      if [[ -z "${SIDE_CAMERA}" ]]; then
        echo "[record-vla-dataset] SIDE_CAMERA_ENABLE=true SIDE_CAMERA_TYPE=opencv but SIDE_CAMERA is empty." >&2
        exit 1
      fi
      CAMERAS_CONFIG+=$(
        cat <<EOF
,
  side: {type: opencv, index_or_path: "${SIDE_CAMERA}", width: ${SIDE_CAMERA_WIDTH}, height: ${SIDE_CAMERA_HEIGHT}, fps: ${SIDE_CAMERA_FPS}, fourcc: "${SIDE_CAMERA_FOURCC}", warmup_s: ${SIDE_CAMERA_WARMUP_S}}
EOF
      )
      ;;
    *)
      echo "[record-vla-dataset] Unsupported SIDE_CAMERA_TYPE=${SIDE_CAMERA_TYPE}; use zmq or opencv." >&2
      exit 1
      ;;
  esac
fi

CAMERAS_CONFIG+=$'\n}'

echo "[record-vla-dataset] repo: ${REPO_DIR}"
echo "[record-vla-dataset] conda env: ${CONDA_ENV}"
echo "[record-vla-dataset] dataset name: ${DATASET_NAME}"
echo "[record-vla-dataset] dataset root: ${DATASET_ROOT}"
echo "[record-vla-dataset] repo id: ${REPO_ID}"
echo "[record-vla-dataset] resume dataset: ${RESUME_DATASET}"
echo "[record-vla-dataset] resume latest: ${RESUME_LATEST}"
echo "[record-vla-dataset] dataset base dir: ${DATASET_BASE_DIR}"
echo "[record-vla-dataset] episodes: ${NUM_EPISODES}"
echo "[record-vla-dataset] episode time: ${EPISODE_TIME_S}s"
echo "[record-vla-dataset] record prewarm steps: ${RECORD_PREWARM_STEPS}"
echo "[record-vla-dataset] start gate topic: ${START_GATE_TOPIC:-disabled}"
echo "[record-vla-dataset] start gate value: ${START_GATE_VALUE}"
echo "[record-vla-dataset] start gate stable time: ${START_GATE_STABLE_S}s"
echo "[record-vla-dataset] start gate timeout: ${START_GATE_TIMEOUT_S}s"
echo "[record-vla-dataset] start gate status log: ${START_GATE_STATUS_LOG_S}s"
echo "[record-vla-dataset] stop gate topic: ${STOP_GATE_TOPIC:-disabled}"
echo "[record-vla-dataset] stop gate value: ${STOP_GATE_VALUE}"
echo "[record-vla-dataset] dataset status topic: ${DATASET_STATUS_TOPIC:-<disabled>}"
echo "[record-vla-dataset] reset time: ${RESET_TIME_S}s"
echo "[record-vla-dataset] video: ${DATASET_VIDEO}"
echo "[record-vla-dataset] dataset fps: ${DATASET_FPS}"
echo "[record-vla-dataset] camera fps: ${CAMERA_FPS}"
echo "[record-vla-dataset] camera fourcc: ${CAMERA_FOURCC}"
echo "[record-vla-dataset] camera warmup: ${CAMERA_WARMUP_S}s"
echo "[record-vla-dataset] opencv max consecutive read failures: ${LEROBOT_OPENCV_MAX_CONSECUTIVE_READ_FAILURES}"
echo "[record-vla-dataset] video codec: ${DATASET_VCODEC}"
echo "[record-vla-dataset] streaming encoding: ${STREAMING_ENCODING}"
echo "[record-vla-dataset] encoder queue maxsize: ${ENCODER_QUEUE_MAXSIZE}"
echo "[record-vla-dataset] image writer processes: ${IMAGE_WRITER_PROCESSES}"
echo "[record-vla-dataset] image writer threads/camera: ${IMAGE_WRITER_THREADS_PER_CAMERA}"
echo "[record-vla-dataset] front camera: ${FRONT_CAMERA}"
echo "[record-vla-dataset] down camera: ${DOWN_CAMERA}"
echo "[record-vla-dataset] top camera enable: ${TOP_CAMERA_ENABLE}"
echo "[record-vla-dataset] top camera type: ${TOP_CAMERA_TYPE}"
if bool_is_true "${TOP_CAMERA_ENABLE}"; then
  if [[ "${TOP_CAMERA_TYPE}" == "zmq" ]]; then
    echo "[record-vla-dataset] top camera zmq: ${TOP_CAMERA_NAME}@${TOP_CAMERA_SERVER}:${TOP_CAMERA_PORT} ${TOP_CAMERA_WIDTH}x${TOP_CAMERA_HEIGHT}@${TOP_CAMERA_FPS}fps timeout=${TOP_CAMERA_TIMEOUT_MS}ms"
  else
    echo "[record-vla-dataset] top camera opencv: ${TOP_CAMERA} ${TOP_CAMERA_WIDTH}x${TOP_CAMERA_HEIGHT}@${TOP_CAMERA_FPS}fps"
  fi
fi
echo "[record-vla-dataset] side camera enable: ${SIDE_CAMERA_ENABLE}"
echo "[record-vla-dataset] side camera type: ${SIDE_CAMERA_TYPE}"
if bool_is_true "${SIDE_CAMERA_ENABLE}"; then
  if [[ "${SIDE_CAMERA_TYPE}" == "zmq" ]]; then
    echo "[record-vla-dataset] side camera zmq: ${SIDE_CAMERA_NAME}@${SIDE_CAMERA_SERVER}:${SIDE_CAMERA_PORT} ${SIDE_CAMERA_WIDTH}x${SIDE_CAMERA_HEIGHT}@${SIDE_CAMERA_FPS}fps timeout=${SIDE_CAMERA_TIMEOUT_MS}ms"
  else
    echo "[record-vla-dataset] side camera opencv: ${SIDE_CAMERA} ${SIDE_CAMERA_WIDTH}x${SIDE_CAMERA_HEIGHT}@${SIDE_CAMERA_FPS}fps"
  fi
fi
echo "[record-vla-dataset] gripper port: ${GRIPPER_PORT}"
echo "[record-vla-dataset] use ros gripper: ${USE_ROS_GRIPPER}"
echo "[record-vla-dataset] gripper scalar command topic: ${GRIPPER_TOPIC}"
echo "[record-vla-dataset] gripper command pair topic: ${GRIPPER_COMMAND_PAIR_TOPIC}"
echo "[record-vla-dataset] gripper feedback topic: ${GRIPPER_FEEDBACK_TOPIC}"
echo "[record-vla-dataset] gripper feedback timeout: ${GRIPPER_FEEDBACK_TIMEOUT_S}s"
echo "[record-vla-dataset] camera max frame age: ${ROBOT_CAMERA_MAX_AGE_MS}ms"
echo "[record-vla-dataset] gripper left inverted: ${GRIPPER_LEFT_INVERTED}"
echo "[record-vla-dataset] gripper right inverted: ${GRIPPER_RIGHT_INVERTED}"
echo "[record-vla-dataset] safe open gripper on disconnect: ${SAFE_OPEN_GRIPPER_ON_DISCONNECT}"
echo "[record-vla-dataset] safe open gripper after episode: ${SAFE_OPEN_GRIPPER_AFTER_EPISODE}"
echo "[record-vla-dataset] disconnect gripper open position: ${DISCONNECT_GRIPPER_OPEN_POSITION}"
echo "[record-vla-dataset] disconnect gripper repeats: ${DISCONNECT_GRIPPER_REPEATS}"
echo "[record-vla-dataset] disconnect gripper settle: ${DISCONNECT_GRIPPER_SETTLE_S}s"
echo "[record-vla-dataset] pose topic: ${NOKOV_POSE_TOPIC}"
echo "[record-vla-dataset] expert topic: ${EXPERT_POSE_TOPIC}"
echo "[record-vla-dataset] gripper topic: ${GRIPPER_TOPIC}"
echo "[record-vla-dataset] start gate: ${START_GATE_TOPIC} == ${START_GATE_VALUE}"
echo "[record-vla-dataset] stop gate: ${STOP_GATE_TOPIC:-<disabled>} == ${STOP_GATE_VALUE}"

export LEROBOT_OPENCV_MAX_CONSECUTIVE_READ_FAILURES
PYTHONUNBUFFERED=1 lerobot-record \
  --resume="${RESUME_DATASET}" \
  --robot.type=vla_drone \
  --robot.nokov_pose_topic="${NOKOV_POSE_TOPIC}" \
  --robot.max_pose_age_s="${ROBOT_MAX_POSE_AGE_S}" \
  --robot.camera_max_age_ms="${ROBOT_CAMERA_MAX_AGE_MS}" \
  --robot.mavros_setpoint_topic="${MAVROS_SETPOINT_TOPIC}" \
  --robot.send_pose_actions=false \
  --robot.gripper_port="${GRIPPER_PORT}" \
  --robot.use_ros_gripper="${USE_ROS_GRIPPER}" \
  --robot.gripper_command_topic="${GRIPPER_TOPIC}" \
  --robot.gripper_feedback_topic="${GRIPPER_FEEDBACK_TOPIC}" \
  --robot.gripper_feedback_timeout_s="${GRIPPER_FEEDBACK_TIMEOUT_S}" \
  --robot.gripper_left_inverted="${GRIPPER_LEFT_INVERTED}" \
  --robot.gripper_right_inverted="${GRIPPER_RIGHT_INVERTED}" \
  --robot.safe_open_gripper_on_disconnect="${SAFE_OPEN_GRIPPER_ON_DISCONNECT}" \
  --robot.safe_open_gripper_after_episode="${SAFE_OPEN_GRIPPER_AFTER_EPISODE}" \
  --robot.disconnect_gripper_open_position="${DISCONNECT_GRIPPER_OPEN_POSITION}" \
  --robot.disconnect_gripper_repeats="${DISCONNECT_GRIPPER_REPEATS}" \
  --robot.disconnect_gripper_settle_s="${DISCONNECT_GRIPPER_SETTLE_S}" \
  --robot.cameras="${CAMERAS_CONFIG}" \
  --teleop.type=ros_expert_pose \
  --teleop.expert_pose_topic="${EXPERT_POSE_TOPIC}" \
  --teleop.gripper_topic="${GRIPPER_TOPIC}" \
  --teleop.gripper_pair_topic="${GRIPPER_COMMAND_PAIR_TOPIC}" \
  --teleop.startup_timeout_s="${TELEOP_STARTUP_TIMEOUT_S}" \
  --teleop.max_pose_age_s="${TELEOP_MAX_POSE_AGE_S}" \
  --dataset.repo_id="${REPO_ID}" \
  --dataset.root="${DATASET_ROOT}" \
  --dataset.fps="${DATASET_FPS}" \
  --dataset.num_episodes="${NUM_EPISODES}" \
  --dataset.episode_time_s="${EPISODE_TIME_S}" \
  --dataset.record_prewarm_steps="${RECORD_PREWARM_STEPS}" \
  --dataset.start_gate_topic="${START_GATE_TOPIC}" \
  --dataset.start_gate_value="${START_GATE_VALUE}" \
  --dataset.start_gate_stable_s="${START_GATE_STABLE_S}" \
  --dataset.start_gate_timeout_s="${START_GATE_TIMEOUT_S}" \
  --dataset.start_gate_status_log_s="${START_GATE_STATUS_LOG_S}" \
  --dataset.stop_gate_topic="${STOP_GATE_TOPIC}" \
  --dataset.stop_gate_value="${STOP_GATE_VALUE}" \
  --dataset.status_topic="${DATASET_STATUS_TOPIC}" \
  --dataset.reset_time_s="${RESET_TIME_S}" \
  --dataset.single_task="${TASK}" \
  --dataset.push_to_hub="${PUSH_TO_HUB}" \
  --dataset.video="${DATASET_VIDEO}" \
  --dataset.vcodec="${DATASET_VCODEC}" \
  --dataset.streaming_encoding="${STREAMING_ENCODING}" \
  --dataset.encoder_threads="${ENCODER_THREADS}" \
  --dataset.encoder_queue_maxsize="${ENCODER_QUEUE_MAXSIZE}" \
  --dataset.num_image_writer_processes="${IMAGE_WRITER_PROCESSES}" \
  --dataset.num_image_writer_threads_per_camera="${IMAGE_WRITER_THREADS_PER_CAMERA}" \
  --play_sounds="${PLAY_SOUNDS}"
