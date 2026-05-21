#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_DIR="$(cd "${WORKSPACE_DIR}/../.." && pwd)"

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

FRONT_CAMERA="${FRONT_CAMERA:-/dev/video0}"
DOWN_CAMERA="${DOWN_CAMERA:-/dev/video2}"
CAMERA_WIDTH="${CAMERA_WIDTH:-640}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-480}"
CAMERA_FPS="${CAMERA_FPS:-20}"

DATASET_FPS="${DATASET_FPS:-20}"
NUM_EPISODES="${NUM_EPISODES:-1}"
EPISODE_TIME_S="${EPISODE_TIME_S:-30}"
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
  front: {type: opencv, index_or_path: "${FRONT_CAMERA}", width: ${CAMERA_WIDTH}, height: ${CAMERA_HEIGHT}, fps: ${CAMERA_FPS}},
  down: {type: opencv, index_or_path: "${DOWN_CAMERA}", width: ${CAMERA_WIDTH}, height: ${CAMERA_HEIGHT}, fps: ${CAMERA_FPS}}
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
echo "[record-vla-dataset] reset time: ${RESET_TIME_S}s"
echo "[record-vla-dataset] video: ${DATASET_VIDEO}"
echo "[record-vla-dataset] dataset fps: ${DATASET_FPS}"
echo "[record-vla-dataset] camera fps: ${CAMERA_FPS}"
echo "[record-vla-dataset] video codec: ${DATASET_VCODEC}"
echo "[record-vla-dataset] streaming encoding: ${STREAMING_ENCODING}"
echo "[record-vla-dataset] image writer processes: ${IMAGE_WRITER_PROCESSES}"
echo "[record-vla-dataset] image writer threads/camera: ${IMAGE_WRITER_THREADS_PER_CAMERA}"
echo "[record-vla-dataset] front camera: ${FRONT_CAMERA}"
echo "[record-vla-dataset] down camera: ${DOWN_CAMERA}"
echo "[record-vla-dataset] gripper port: ${GRIPPER_PORT}"
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
