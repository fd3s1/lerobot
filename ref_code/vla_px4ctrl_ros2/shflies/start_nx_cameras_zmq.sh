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
NX_CAMERA_PORT="${NX_CAMERA_PORT:-5556}"
FRONT_CAMERA="${FRONT_CAMERA:-/dev/video2}"
DOWN_CAMERA="${DOWN_CAMERA:-/dev/video0}"
CAMERA_WIDTH="${CAMERA_WIDTH:-640}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-480}"
CAMERA_FPS="${CAMERA_FPS:-30}"
CAMERA_FOURCC="${CAMERA_FOURCC:-MJPG}"

require_camera_path() {
  local label="$1"
  local path="$2"
  if [[ ! -e "${path}" ]]; then
    echo "[nx-cameras-zmq] ${label} camera path does not exist: ${path}" >&2
    echo "[nx-cameras-zmq] Check: ls -l /dev/v4l/by-path/ /dev/video*" >&2
    exit 1
  fi
}

require_camera_path "front" "${FRONT_CAMERA}"
require_camera_path "down" "${DOWN_CAMERA}"

if [[ -f "${CONDA_SH}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${CONDA_SH}"
  conda activate "${CONDA_ENV}"
  set -u
fi

cd "${REPO_DIR}"
export PYTHONPATH="${REPO_DIR}/src:${PYTHONPATH:-}"

echo "[nx-cameras-zmq] front: ${FRONT_CAMERA}"
echo "[nx-cameras-zmq] down: ${DOWN_CAMERA}"
echo "[nx-cameras-zmq] bind: tcp://*:$(printf '%s' "${NX_CAMERA_PORT}")"
echo "[nx-cameras-zmq] shape: ${CAMERA_WIDTH}x${CAMERA_HEIGHT}@${CAMERA_FPS}fps fourcc=${CAMERA_FOURCC}"

FRONT_CAMERA="${FRONT_CAMERA}" \
DOWN_CAMERA="${DOWN_CAMERA}" \
NX_CAMERA_PORT="${NX_CAMERA_PORT}" \
CAMERA_WIDTH="${CAMERA_WIDTH}" \
CAMERA_HEIGHT="${CAMERA_HEIGHT}" \
CAMERA_FPS="${CAMERA_FPS}" \
CAMERA_FOURCC="${CAMERA_FOURCC}" \
python3 - <<'PY'
import os

from lerobot.cameras.zmq.image_server import ImageServer

shape = [int(float(os.environ["CAMERA_HEIGHT"])), int(float(os.environ["CAMERA_WIDTH"]))]
config = {
    "fps": int(float(os.environ["CAMERA_FPS"])),
    "cameras": {
        "front": {
            "device_id": os.environ["FRONT_CAMERA"],
            "shape": shape,
            "fourcc": os.environ.get("CAMERA_FOURCC") or None,
        },
        "down": {
            "device_id": os.environ["DOWN_CAMERA"],
            "shape": shape,
            "fourcc": os.environ.get("CAMERA_FOURCC") or None,
        },
    },
}

ImageServer(config, port=int(os.environ["NX_CAMERA_PORT"])).run()
PY
