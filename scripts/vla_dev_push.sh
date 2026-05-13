#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/vla_dev_push.sh ["commit message"]

Commit local development changes and push the current branch to origin.

Environment variables:
  VLA_SYNC_REMOTE   Git remote to push to. Default: origin
  VLA_SYNC_BRANCH   Branch to push. Default: current branch

Notes:
  - This script stages the repository, but excludes known old YOLO nested-git
    reference folders that are unrelated to the VLADrone work.
  - Build artifacts under ref_code/vla_px4ctrl_ros2/build, install, and log
    are ignored/excluded.
USAGE
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

repo_root="$(git rev-parse --show-toplevel)"
cd "$repo_root"

remote="${VLA_SYNC_REMOTE:-origin}"
branch="${VLA_SYNC_BRANCH:-$(git branch --show-current)}"

if [[ -z "$branch" ]]; then
  echo "ERROR: Could not determine current git branch. Set VLA_SYNC_BRANCH." >&2
  exit 1
fi

commit_msg="${1:-Update VLADrone workspace $(date '+%Y-%m-%d %H:%M:%S')}"

echo "[vla-dev-push] repo: $repo_root"
echo "[vla-dev-push] branch: $branch"
echo "[vla-dev-push] remote: $remote"

echo "[vla-dev-push] working tree changes before staging:"
if git status --short | grep -q .; then
  git status --short
else
  echo "  (none)"
fi

git add -A -- .

# Keep known local-only reference/build paths out of this sync commit.
git reset -q -- \
  ref_code/lesson_ws_od \
  ref_code/vla_px4ctrl_ros2/build \
  ref_code/vla_px4ctrl_ros2/install \
  ref_code/vla_px4ctrl_ros2/log || true

if git diff --cached --quiet; then
  echo "[vla-dev-push] no staged changes; pushing any existing local commits."
else
  echo "[vla-dev-push] files to commit:"
  git diff --cached --name-status
  git commit -m "$commit_msg"
fi

echo "[vla-dev-push] local commits not yet on $remote/$branch:"
if git rev-parse --verify --quiet "$remote/$branch" >/dev/null; then
  if git log --oneline "$remote/$branch..HEAD" | grep -q .; then
    git log --oneline "$remote/$branch..HEAD"
  else
    echo "  (none)"
  fi
else
  echo "  remote tracking ref $remote/$branch is not available locally yet"
fi

git push "$remote" "$branch"

echo "[vla-dev-push] done."
