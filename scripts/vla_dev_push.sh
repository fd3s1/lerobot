#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/vla_dev_push.sh [--include-lesson-ws] [--include-august-ws] ["commit message"]

Commit local development changes and push the current branch to origin.
Default VLADrone branch: drone-smolvla-v044. The script uses your current
branch unless VLA_SYNC_BRANCH is set.

Options:
  --include-lesson-ws
      Also commit changes under ref_code/lesson_ws_od. By default this old
      reference workspace is excluded to avoid accidental large syncs.
  --include-august-ws
      Also commit changes under ref_code/august_ws_od. By default this old
      reference workspace is excluded to avoid accidental large syncs.
  --include-old-ref-workspaces
      Include both ref_code/lesson_ws_od and ref_code/august_ws_od.

Environment variables:
  VLA_SYNC_REMOTE   Git remote to push to. Default: origin
  VLA_SYNC_BRANCH   Branch to push. Default: current branch
  VLA_SYNC_LESSON_WS
                    Set to 1 or true to include ref_code/lesson_ws_od.
  VLA_SYNC_AUGUST_WS
                    Set to 1 or true to include ref_code/august_ws_od.
  VLA_SYNC_OLD_REF_WORKSPACES
                    Set to 1 or true to include both old reference workspaces.

Notes:
  - This script stages the repository, but excludes ref_code/lesson_ws_od and
    ref_code/august_ws_od by default because they are old reference workspaces
    and can contain large local-only changes.
  - Build artifacts under ref_code/vla_px4ctrl_ros2/build, install, and log
    are ignored/excluded.
USAGE
}

include_lesson_ws="${VLA_SYNC_LESSON_WS:-false}"
include_august_ws="${VLA_SYNC_AUGUST_WS:-false}"
include_old_ref_workspaces="${VLA_SYNC_OLD_REF_WORKSPACES:-false}"
commit_msg_parts=()

for arg in "$@"; do
  case "$arg" in
    --include-lesson-ws)
      include_lesson_ws=true
      ;;
    --include-august-ws)
      include_august_ws=true
      ;;
    --include-old-ref-workspaces)
      include_old_ref_workspaces=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      commit_msg_parts+=("$arg")
      ;;
  esac
done

repo_root="$(git rev-parse --show-toplevel)"
cd "$repo_root"

remote="${VLA_SYNC_REMOTE:-origin}"
branch="${VLA_SYNC_BRANCH:-$(git branch --show-current)}"

if [[ -z "$branch" ]]; then
  echo "ERROR: Could not determine current git branch. Set VLA_SYNC_BRANCH." >&2
  exit 1
fi

if [[ "${include_lesson_ws,,}" == "1" ]]; then
  include_lesson_ws=true
fi
if [[ "${include_august_ws,,}" == "1" ]]; then
  include_august_ws=true
fi
if [[ "${include_old_ref_workspaces,,}" == "1" ]]; then
  include_old_ref_workspaces=true
fi
if [[ "${include_old_ref_workspaces,,}" == "true" ]]; then
  include_lesson_ws=true
  include_august_ws=true
fi

if [[ "${#commit_msg_parts[@]}" -gt 0 ]]; then
  commit_msg="${commit_msg_parts[*]}"
else
  commit_msg="Update VLADrone workspace $(date '+%Y-%m-%d %H:%M:%S')"
fi

echo "[vla-dev-push] repo: $repo_root"
echo "[vla-dev-push] branch: $branch"
echo "[vla-dev-push] remote: $remote"
echo "[vla-dev-push] include lesson_ws_od: $include_lesson_ws"
echo "[vla-dev-push] include august_ws_od: $include_august_ws"

echo "[vla-dev-push] working tree changes before staging:"
if git --no-pager status --short | grep -q .; then
  git --no-pager status --short
else
  echo "  (none)"
fi

git add -A -- .

# Keep known local-only reference/build paths out of this sync commit.
git reset -q -- \
  ref_code/vla_px4ctrl_ros2/build \
  ref_code/vla_px4ctrl_ros2/install \
  ref_code/vla_px4ctrl_ros2/log || true

if [[ "$include_lesson_ws" != true ]]; then
  git reset -q -- ref_code/lesson_ws_od || true
fi
if [[ "$include_august_ws" != true ]]; then
  git reset -q -- ref_code/august_ws_od || true
fi

if git diff --cached --quiet; then
  echo "[vla-dev-push] no staged changes; pushing any existing local commits."
else
  echo "[vla-dev-push] files to commit:"
  git --no-pager diff --cached --name-status
  git commit -m "$commit_msg"
fi

echo "[vla-dev-push] local commits not yet on $remote/$branch:"
if git rev-parse --verify --quiet "$remote/$branch" >/dev/null; then
  if git --no-pager log --oneline "$remote/$branch..HEAD" | grep -q .; then
    git --no-pager log --oneline "$remote/$branch..HEAD"
  else
    echo "  (none)"
  fi
else
  echo "  remote tracking ref $remote/$branch is not available locally yet"
fi

git push "$remote" "$branch"

echo "[vla-dev-push] done."
