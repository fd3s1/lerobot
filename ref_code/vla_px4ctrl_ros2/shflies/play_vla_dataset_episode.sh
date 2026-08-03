#!/usr/bin/env bash
set -euo pipefail

DATASET_BASE_DIR="${DATASET_BASE_DIR:-${HOME}/vla_drone/data}"
DATASET_PREFIX="${DATASET_PREFIX:-vla_drone_grasp}"
DATASET_ROOT="${DATASET_ROOT:-}"
EPISODE_FILE="${EPISODE_FILE:-file-000.mp4}"
CHUNK_DIR="${CHUNK_DIR:-chunk-000}"
CAMERAS="${CAMERAS:-front down top}"
PLAY="${PLAY:-true}"
REBUILD_PREVIEW="${REBUILD_PREVIEW:-false}"
PREVIEW_PATH="${PREVIEW_PATH:-}"
FFMPEG="${FFMPEG:-ffmpeg}"
FFPLAY="${FFPLAY:-ffplay}"

bool_is_true() {
  case "${1,,}" in
    1|true|yes|y|on) return 0 ;;
    *) return 1 ;;
  esac
}

latest_dataset_root() {
  find "${DATASET_BASE_DIR}" -maxdepth 1 -type d -name "${DATASET_PREFIX}_*" -printf "%T@ %p\n" 2>/dev/null |
    sort -nr |
    awk 'NR == 1 {sub($1 FS, ""); print}'
}

if [[ -z "${DATASET_ROOT}" ]]; then
  DATASET_ROOT="$(latest_dataset_root)"
fi

if [[ -z "${DATASET_ROOT}" || ! -d "${DATASET_ROOT}" ]]; then
  echo "[play-vla-dataset] dataset root not found: ${DATASET_ROOT:-<latest ${DATASET_PREFIX}_*>}" >&2
  echo "[play-vla-dataset] Set DATASET_ROOT=/path/to/dataset." >&2
  exit 1
fi

if [[ -z "${PREVIEW_PATH}" ]]; then
  PREVIEW_PATH="${DATASET_ROOT}/preview_${CHUNK_DIR}_${EPISODE_FILE%.mp4}.mp4"
fi

inputs=()
filter_parts=()
stack_inputs=()
idx=0
for camera in ${CAMERAS}; do
  video="${DATASET_ROOT}/videos/observation.images.${camera}/${CHUNK_DIR}/${EPISODE_FILE}"
  if [[ ! -f "${video}" ]]; then
    echo "[play-vla-dataset] missing ${camera} video: ${video}" >&2
    exit 1
  fi
  inputs+=(-i "${video}")
  filter_parts+=("[${idx}:v]drawtext=text='${camera}':x=12:y=12:fontsize=24:fontcolor=white:box=1:boxcolor=black@0.55[v${idx}]")
  stack_inputs+=("[v${idx}]")
  idx=$((idx + 1))
done

if (( idx < 1 )); then
  echo "[play-vla-dataset] no cameras selected." >&2
  exit 1
fi

echo "[play-vla-dataset] dataset: ${DATASET_ROOT}"
echo "[play-vla-dataset] cameras: ${CAMERAS}"
echo "[play-vla-dataset] preview: ${PREVIEW_PATH}"

if [[ ! -f "${PREVIEW_PATH}" ]] || bool_is_true "${REBUILD_PREVIEW}"; then
  filter="$(IFS=';'; echo "${filter_parts[*]}");$(printf '%s' "${stack_inputs[@]}")hstack=inputs=${idx}[v]"
  if ! "${FFMPEG}" -hide_banner -y "${inputs[@]}" \
      -filter_complex "${filter}" \
      -map "[v]" -an -c:v libx264 -preset veryfast -crf 23 "${PREVIEW_PATH}"; then
    echo "[play-vla-dataset] labeled preview failed; retrying without drawtext labels." >&2
    raw_inputs=()
    for ((i = 0; i < idx; i++)); do
      raw_inputs+=("[${i}:v]")
    done
    "${FFMPEG}" -hide_banner -y "${inputs[@]}" \
      -filter_complex "$(printf '%s' "${raw_inputs[@]}")hstack=inputs=${idx}[v]" \
      -map "[v]" -an -c:v libx264 -preset veryfast -crf 23 "${PREVIEW_PATH}"
  fi
fi

if bool_is_true "${PLAY}"; then
  echo "[play-vla-dataset] playing with ${FFPLAY}. Close the window or press q to exit."
  "${FFPLAY}" -hide_banner -autoexit "${PREVIEW_PATH}"
else
  echo "[play-vla-dataset] PLAY=false; preview generated only."
fi
