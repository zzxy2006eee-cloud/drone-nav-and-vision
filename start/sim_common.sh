#!/usr/bin/env bash
# Shared environment and ownership helpers for the two-terminal simulator.
set -eo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$project_root/devel/setup.bash"
set -u
unset ROS_IP
export ROS_MASTER_URI="${DRONE_ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${DRONE_ROS_HOSTNAME:-127.0.0.1}"
export ROS_HOME="${DRONE_ROS_HOME:-/tmp/drone_ros_home}"
export OPENBLAS_NUM_THREADS=1
export DRONE_SHOW_GAZEBO="${DRONE_SHOW_GAZEBO:-1}"
export DRONE_PROCESS_ROOT="$project_root"
python3 "$project_root/start/manage_processes.py" register "$$"
if command -v xrandr >/dev/null && ! xrandr --query 2>/dev/null | rg -q ' connected'; then
  display_compat="$project_root/devel/lib/libdrone_headless_xrandr_compat.so"
  if [[ -f "$display_compat" ]]; then
    export DRONE_HEADLESS_RANDR_WORKAROUND=1
    export LD_PRELOAD="$display_compat${LD_PRELOAD:+:$LD_PRELOAD}"
  fi
fi
process_groups=()
cleanup() {
  trap - EXIT INT TERM
  for group in "${process_groups[@]}"; do kill -INT -- "-$group" 2>/dev/null || true; done
  sleep 2
  for group in "${process_groups[@]}"; do kill -TERM -- "-$group" 2>/dev/null || true; done
  python3 "$project_root/start/manage_processes.py" unregister "$$" || true
  wait || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
cd "$project_root"
