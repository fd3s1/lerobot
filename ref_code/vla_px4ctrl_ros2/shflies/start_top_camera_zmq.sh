#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_DIR="$(cd "${WORKSPACE_DIR}/../.." && pwd)"
CAMERA_PATHS_FILE="${CAMERA_PATHS_FILE:-${SCRIPT_DIR}/record_camera_paths.env}"

TOP_CAMERA_DEVICE_OVERRIDE="${TOP_CAMERA_DEVICE:-}"

if [[ -f "${CAMERA_PATHS_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${CAMERA_PATHS_FILE}"
fi

if [[ -n "${TOP_CAMERA_DEVICE_OVERRIDE}" ]]; then
  TOP_CAMERA_DEVICE="${TOP_CAMERA_DEVICE_OVERRIDE}"
fi

CONDA_ENV="${CONDA_ENV:-vla-drone}"
if [[ -z "${CONDA_SH:-}" ]]; then
  if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
    CONDA_SH="${HOME}/miniforge3/etc/profile.d/conda.sh"
  elif [[ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]]; then
    CONDA_SH="${HOME}/miniconda3/etc/profile.d/conda.sh"
  else
    CONDA_SH=""
  fi
fi

TOP_CAMERA_DEVICE="${TOP_CAMERA_DEVICE:-}"
TOP_CAMERA_NAME="${TOP_CAMERA_NAME:-top}"
TOP_CAMERA_PORT="${TOP_CAMERA_PORT:-5555}"
TOP_CAMERA_WIDTH="${TOP_CAMERA_WIDTH:-640}"
TOP_CAMERA_HEIGHT="${TOP_CAMERA_HEIGHT:-480}"
TOP_CAMERA_FPS="${TOP_CAMERA_FPS:-30}"
TOP_CAMERA_FOURCC="${TOP_CAMERA_FOURCC:-MJPG}"

discover_top_camera_device() {
  local candidate
  local candidates=()

  while IFS= read -r candidate; do
    candidates+=("${candidate}")
  done < <(
    {
      find /dev/v4l/by-id -maxdepth 1 -type l -name '*video-index0' -print 2>/dev/null
      find /dev/v4l/by-path -maxdepth 1 -type l -name '*video-index0' -print 2>/dev/null
      find /dev -maxdepth 1 -type c -name 'video*' -print 2>/dev/null
    } | sort -u
  )

  for candidate in "${candidates[@]}"; do
    if [[ -e "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  return 1
}

if [[ -z "${TOP_CAMERA_DEVICE}" ]]; then
  if ! TOP_CAMERA_DEVICE="$(discover_top_camera_device)"; then
    echo "[top-camera-zmq] ERROR: no video camera found." >&2
    echo "[top-camera-zmq] Set TOP_CAMERA_DEVICE=/dev/videoX or a /dev/v4l/by-id path." >&2
    exit 1
  fi
fi

if [[ -n "${CONDA_SH}" && -f "${CONDA_SH}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${CONDA_SH}"
  conda activate "${CONDA_ENV}"
  set -u
fi

cd "${REPO_DIR}"
export PYTHONPATH="${REPO_DIR}/src:${PYTHONPATH:-}"

echo "[top-camera-zmq] device: ${TOP_CAMERA_DEVICE}"
echo "[top-camera-zmq] name: ${TOP_CAMERA_NAME}"
echo "[top-camera-zmq] bind: tcp://*:$(printf '%s' "${TOP_CAMERA_PORT}")"
echo "[top-camera-zmq] shape: ${TOP_CAMERA_WIDTH}x${TOP_CAMERA_HEIGHT}@${TOP_CAMERA_FPS}fps fourcc=${TOP_CAMERA_FOURCC}"
echo "[top-camera-zmq] use the host IP as TOP_CAMERA_SERVER on the NX."

TOP_CAMERA_DEVICE="${TOP_CAMERA_DEVICE}" \
TOP_CAMERA_NAME="${TOP_CAMERA_NAME}" \
TOP_CAMERA_PORT="${TOP_CAMERA_PORT}" \
TOP_CAMERA_WIDTH="${TOP_CAMERA_WIDTH}" \
TOP_CAMERA_HEIGHT="${TOP_CAMERA_HEIGHT}" \
TOP_CAMERA_FPS="${TOP_CAMERA_FPS}" \
TOP_CAMERA_FOURCC="${TOP_CAMERA_FOURCC}" \
python3 - <<'PY'
import os

from lerobot.cameras.zmq.image_server import ImageServer

config = {
    "fps": int(float(os.environ["TOP_CAMERA_FPS"])),
    "cameras": {
        os.environ["TOP_CAMERA_NAME"]: {
            "device_id": os.environ["TOP_CAMERA_DEVICE"],
            "shape": [
                int(float(os.environ["TOP_CAMERA_HEIGHT"])),
                int(float(os.environ["TOP_CAMERA_WIDTH"])),
            ],
            "fourcc": os.environ.get("TOP_CAMERA_FOURCC") or None,
        }
    },
}

ImageServer(config, port=int(os.environ["TOP_CAMERA_PORT"])).run()
PY
