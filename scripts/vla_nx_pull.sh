#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/vla_nx_pull.sh [--build]

Pull the current VLADrone branch on the Orin NX.
Default VLADrone branch: drone-smolvla-v044. The script uses your current
branch unless VLA_SYNC_BRANCH is set.

Options:
  --build   Run colcon build for ref_code/vla_px4ctrl_ros2 after pulling.

Environment variables:
  VLA_SYNC_REMOTE   Git remote to pull from. Default: origin
  VLA_SYNC_BRANCH   Branch to pull. Default: current branch

Behavior:
  - Stashes local uncommitted changes, including untracked files.
  - Fetches and fast-forwards from the remote branch.
  - Re-applies the stash if one was created.
  - With --build, builds the ROS2 workspace using /usr/bin/python3.
  - Build artifacts under ref_code/vla_px4ctrl_ros2/build, install, and log
    should remain local and ignored.
USAGE
}

build_after_pull=false
for arg in "$@"; do
  case "$arg" in
    --build)
      build_after_pull=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: Unknown argument: $arg" >&2
      usage >&2
      exit 1
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

echo "[vla-nx-pull] repo: $repo_root"
echo "[vla-nx-pull] branch: $branch"
echo "[vla-nx-pull] remote: $remote"

echo "[vla-nx-pull] local working tree changes before pull:"
if git --no-pager status --short | grep -q .; then
  git --no-pager status --short
else
  echo "  (none)"
fi

stash_created=false
if ! git diff --quiet || ! git diff --cached --quiet || [[ -n "$(git ls-files --others --exclude-standard)" ]]; then
  stash_name="vla-nx-auto-stash $(date '+%Y-%m-%d %H:%M:%S')"
  echo "[vla-nx-pull] local changes detected; stashing as: $stash_name"
  git stash push -u -m "$stash_name"
  stash_created=true
fi

git fetch "$remote" "$branch"
local_head="$(git rev-parse HEAD)"
remote_head="$(git rev-parse FETCH_HEAD)"

if [[ "$local_head" == "$remote_head" ]]; then
  echo "[vla-nx-pull] remote branch has no new commits."
else
  echo "[vla-nx-pull] commits to pull:"
  git --no-pager log --oneline "$local_head..FETCH_HEAD"
  echo "[vla-nx-pull] files changed by incoming commits:"
  git --no-pager diff --name-status "$local_head..FETCH_HEAD"
fi

git checkout "$branch"
git pull --ff-only "$remote" "$branch"

if [[ "$stash_created" == true ]]; then
  echo "[vla-nx-pull] re-applying local stash."
  if ! git stash pop; then
    echo "ERROR: stash pop had conflicts. Resolve them on the NX, then run git status." >&2
    exit 1
  fi
fi

echo "[vla-nx-pull] local working tree changes after pull:"
if git --no-pager status --short | grep -q .; then
  git --no-pager status --short
else
  echo "  (none)"
fi

if [[ "$build_after_pull" == true ]]; then
  ros_setup="/opt/ros/humble/setup.bash"
  if [[ ! -f "$ros_setup" ]]; then
    echo "ERROR: $ros_setup not found. Install/source the correct ROS2 distribution first." >&2
    exit 1
  fi

  echo "[vla-nx-pull] building ROS2 workspace."
  # shellcheck disable=SC1091
  had_nounset=false
  case "$-" in
    *u*)
      had_nounset=true
      set +u
      ;;
  esac
  source "$ros_setup"
  if [[ "$had_nounset" == true ]]; then
    set -u
  fi
  cd "$repo_root/ref_code/vla_px4ctrl_ros2"
  colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
fi

echo "[vla-nx-pull] done."
if [[ "$build_after_pull" == true ]]; then
  echo "[vla-nx-pull] To run ROS2 nodes in a new terminal:"
  echo "  cd $repo_root/ref_code/vla_px4ctrl_ros2"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  source install/setup.bash"
fi
