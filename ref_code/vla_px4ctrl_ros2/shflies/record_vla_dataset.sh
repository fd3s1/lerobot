#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_DIR="$(cd "${WORKSPACE_DIR}/../.." && pwd)"
CAMERA_PATHS_FILE="${CAMERA_PATHS_FILE:-${SCRIPT_DIR}/record_camera_paths.env}"

if [[ -f "${CAMERA_PATHS_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${CAMERA_PATHS_FILE}"
fi

CONDA_ENV="${CONDA_ENV:-vla-drone-v044}"
CONDA_SH="${CONDA_SH:-${HOME}/miniforge3/etc/profile.d/conda.sh}"

RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
DATASET_PREFIX="${DATASET_PREFIX:-vla_drone_grasp}"
DATASET_NAME="${DATASET_NAME:-${DATASET_PREFIX}_${RUN_ID}}"
DATASET_ROOT="${DATASET_ROOT:-${HOME}/vla_drone/data/${DATASET_NAME}}"
REPO_OWNER="${REPO_OWNER:-fd3s1}"
REPO_ID="${REPO_ID:-${REPO_OWNER}/${DATASET_NAME}}"

NOKOV_POSE_TOPIC="${NOKOV_POSE_TOPIC:-/mavros/vision_pose/pose}"
MAVROS_SETPOINT_TOPIC="${MAVROS_SETPOINT_TOPIC:-/position_cmd}"
EXPERT_POSE_TOPIC="${EXPERT_POSE_TOPIC:-/px4ctrl/expert_pose}"
GRIPPER_TOPIC="${GRIPPER_TOPIC:-/gripper/command}"
GRIPPER_PORT="${GRIPPER_PORT:-/dev/ttyACM1}"
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
CAMERA_WARMUP_S="${CAMERA_WARMUP_S:-3}"

DATASET_FPS="${DATASET_FPS:-20}"
NUM_EPISODES="${NUM_EPISODES:-1}"
EPISODE_TIME_S="${EPISODE_TIME_S:-30}"
RECORD_PREWARM_STEPS="${RECORD_PREWARM_STEPS:-2}"
START_GATE_TOPIC="${START_GATE_TOPIC-/px4ctrl/state}"
START_GATE_VALUE="${START_GATE_VALUE:-AUTO_HOVER}"
START_GATE_STABLE_S="${START_GATE_STABLE_S:-3.0}"
START_GATE_TIMEOUT_S="${START_GATE_TIMEOUT_S:-0.0}"
RESET_TIME_S="${RESET_TIME_S:-10}"
TASK="${TASK:-Fly to the target and operate the gripper}"
PUSH_TO_HUB="${PUSH_TO_HUB:-false}"
DATASET_VIDEO="${DATASET_VIDEO:-true}"
DATASET_VCODEC="${DATASET_VCODEC:-h264}"
STREAMING_ENCODING="${STREAMING_ENCODING:-false}"
ENCODER_THREADS="${ENCODER_THREADS:-2}"
IMAGE_WRITER_PROCESSES="${IMAGE_WRITER_PROCESSES:-0}"
IMAGE_WRITER_THREADS_PER_CAMERA="${IMAGE_WRITER_THREADS_PER_CAMERA:-2}"
PLAY_SOUNDS="${PLAY_SOUNDS:-false}"

ROBOT_MAX_POSE_AGE_S="${ROBOT_MAX_POSE_AGE_S:-2.0}"
TELEOP_STARTUP_TIMEOUT_S="${TELEOP_STARTUP_TIMEOUT_S:-2.0}"
TELEOP_MAX_POSE_AGE_S="${TELEOP_MAX_POSE_AGE_S:-0.5}"

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

CAMERAS_CONFIG=$(cat <<EOF
{
  front: {type: opencv, index_or_path: "${FRONT_CAMERA}", width: ${CAMERA_WIDTH}, height: ${CAMERA_HEIGHT}, fps: ${CAMERA_FPS}, warmup_s: ${CAMERA_WARMUP_S}},
  down: {type: opencv, index_or_path: "${DOWN_CAMERA}", width: ${CAMERA_WIDTH}, height: ${CAMERA_HEIGHT}, fps: ${CAMERA_FPS}, warmup_s: ${CAMERA_WARMUP_S}}
}
EOF
)

echo "[record-vla-dataset] repo: ${REPO_DIR}"
echo "[record-vla-dataset] conda env: ${CONDA_ENV}"
echo "[record-vla-dataset] dataset name: ${DATASET_NAME}"
echo "[record-vla-dataset] dataset root: ${DATASET_ROOT}"
echo "[record-vla-dataset] repo id: ${REPO_ID}"
echo "[record-vla-dataset] episodes: ${NUM_EPISODES}"
echo "[record-vla-dataset] episode time: ${EPISODE_TIME_S}s"
echo "[record-vla-dataset] record prewarm steps: ${RECORD_PREWARM_STEPS}"
echo "[record-vla-dataset] start gate topic: ${START_GATE_TOPIC:-disabled}"
echo "[record-vla-dataset] start gate value: ${START_GATE_VALUE}"
echo "[record-vla-dataset] start gate stable time: ${START_GATE_STABLE_S}s"
echo "[record-vla-dataset] start gate timeout: ${START_GATE_TIMEOUT_S}s"
echo "[record-vla-dataset] reset time: ${RESET_TIME_S}s"
echo "[record-vla-dataset] video: ${DATASET_VIDEO}"
echo "[record-vla-dataset] dataset fps: ${DATASET_FPS}"
echo "[record-vla-dataset] camera fps: ${CAMERA_FPS}"
echo "[record-vla-dataset] camera warmup: ${CAMERA_WARMUP_S}s"
echo "[record-vla-dataset] video codec: ${DATASET_VCODEC}"
echo "[record-vla-dataset] streaming encoding: ${STREAMING_ENCODING}"
echo "[record-vla-dataset] image writer processes: ${IMAGE_WRITER_PROCESSES}"
echo "[record-vla-dataset] image writer threads/camera: ${IMAGE_WRITER_THREADS_PER_CAMERA}"
echo "[record-vla-dataset] front camera: ${FRONT_CAMERA}"
echo "[record-vla-dataset] down camera: ${DOWN_CAMERA}"
echo "[record-vla-dataset] gripper port: ${GRIPPER_PORT}"
echo "[record-vla-dataset] safe open gripper on disconnect: ${SAFE_OPEN_GRIPPER_ON_DISCONNECT}"
echo "[record-vla-dataset] safe open gripper after episode: ${SAFE_OPEN_GRIPPER_AFTER_EPISODE}"
echo "[record-vla-dataset] disconnect gripper open position: ${DISCONNECT_GRIPPER_OPEN_POSITION}"
echo "[record-vla-dataset] disconnect gripper repeats: ${DISCONNECT_GRIPPER_REPEATS}"
echo "[record-vla-dataset] disconnect gripper settle: ${DISCONNECT_GRIPPER_SETTLE_S}s"
echo "[record-vla-dataset] pose topic: ${NOKOV_POSE_TOPIC}"
echo "[record-vla-dataset] expert topic: ${EXPERT_POSE_TOPIC}"
echo "[record-vla-dataset] gripper topic: ${GRIPPER_TOPIC}"

PYTHONUNBUFFERED=1 lerobot-record \
  --robot.type=vla_drone \
  --robot.nokov_pose_topic="${NOKOV_POSE_TOPIC}" \
  --robot.max_pose_age_s="${ROBOT_MAX_POSE_AGE_S}" \
  --robot.mavros_setpoint_topic="${MAVROS_SETPOINT_TOPIC}" \
  --robot.send_pose_actions=false \
  --robot.gripper_port="${GRIPPER_PORT}" \
  --robot.safe_open_gripper_on_disconnect="${SAFE_OPEN_GRIPPER_ON_DISCONNECT}" \
  --robot.safe_open_gripper_after_episode="${SAFE_OPEN_GRIPPER_AFTER_EPISODE}" \
  --robot.disconnect_gripper_open_position="${DISCONNECT_GRIPPER_OPEN_POSITION}" \
  --robot.disconnect_gripper_repeats="${DISCONNECT_GRIPPER_REPEATS}" \
  --robot.disconnect_gripper_settle_s="${DISCONNECT_GRIPPER_SETTLE_S}" \
  --robot.cameras="${CAMERAS_CONFIG}" \
  --teleop.type=ros_expert_pose \
  --teleop.expert_pose_topic="${EXPERT_POSE_TOPIC}" \
  --teleop.gripper_topic="${GRIPPER_TOPIC}" \
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
  --dataset.reset_time_s="${RESET_TIME_S}" \
  --dataset.single_task="${TASK}" \
  --dataset.push_to_hub="${PUSH_TO_HUB}" \
  --dataset.video="${DATASET_VIDEO}" \
  --dataset.vcodec="${DATASET_VCODEC}" \
  --dataset.streaming_encoding="${STREAMING_ENCODING}" \
  --dataset.encoder_threads="${ENCODER_THREADS}" \
  --dataset.num_image_writer_processes="${IMAGE_WRITER_PROCESSES}" \
  --dataset.num_image_writer_threads_per_camera="${IMAGE_WRITER_THREADS_PER_CAMERA}" \
  --play_sounds="${PLAY_SOUNDS}"
