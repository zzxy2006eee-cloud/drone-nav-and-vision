#!/usr/bin/env bash
# Own the complete simulation process groups so Ctrl-C leaves no gzserver.
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
export DRONE_PROCESS_ROOT="$project_root"
export DRONE_SHOW_GAZEBO="${DRONE_SHOW_GAZEBO:-1}"
python3 "$project_root/start/manage_processes.py" register "$$"
source /opt/ros/noetic/setup.bash
source "$project_root/devel/setup.bash"
unset ROS_IP
export ROS_MASTER_URI="${DRONE_ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${DRONE_ROS_HOSTNAME:-127.0.0.1}"
export ROS_HOME="${DRONE_ROS_HOME:-/tmp/drone_ros_home}"
export HEADLESS=1
# Tiny rigid transforms should not wake an entire BLAS worker pool. Leave
# Faster-LIO's C++ worker settings intact while reducing Python CPU jitter.
export OPENBLAS_NUM_THREADS=1
# Ogre 1.9 crashes on an empty RandR mode list when all monitors are unplugged.
# Keep GPU rendering; fall back to virtual X screen dimensions in our children.
if command -v xrandr >/dev/null && ! xrandr --query 2>/dev/null | rg -q ' connected'; then
  display_compat="$project_root/devel/lib/libdrone_headless_xrandr_compat.so"
  if [[ -f "$display_compat" ]]; then
    export DRONE_HEADLESS_RANDR_WORKAROUND=1
    export LD_PRELOAD="$display_compat${LD_PRELOAD:+:$LD_PRELOAD}"
  fi
fi
log_dir="${DRONE_LOG_DIR:-$project_root/start/logs/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$log_dir"
mkdir -p "$log_dir/source_snapshot"
cp -a "$project_root/src/drone_stack/config" "$project_root/src/drone_stack/launch" \
      "$project_root/src/drone_stack/scripts" "$project_root/src/drone_stack/patches" \
      "$log_dir/source_snapshot/"
mkdir -p "$log_dir/source_snapshot/ego_plan_env"
cp -a "$project_root/src/ego_planner/src/planner/plan_env/src/grid_map.cpp" \
      "$project_root/src/ego_planner/src/planner/plan_env/include" \
      "$project_root/src/ego_planner/src/planner/plan_env/CMakeLists.txt" \
      "$log_dir/source_snapshot/ego_plan_env/"
sha256sum "$project_root/devel/lib/libplan_env.so" \
          "$project_root/devel/lib/libbspline_opt.so" \
          "$project_root/devel/lib/libpath_searching.so" \
          "$project_root/devel/lib/libtraj_utils.so" \
          "$project_root/devel/lib/ego_planner/traj_server" \
          "$project_root/devel/lib/ego_planner/ego_planner_node" \
          > "$log_dir/source_snapshot/ego_binaries.sha256"
sha256sum "$project_root/external/PX4-Autopilot/build/px4_sitl_inspection/build_gazebo-classic/libgazebo_imu_plugin.so" \
          > "$log_dir/source_snapshot/gazebo_imu_binary.sha256"
mkdir -p "$log_dir/source_snapshot/faster_lio"
cp -a "$project_root/src/faster_lio_main/src" "$project_root/src/faster_lio_main/include" \
      "$project_root/src/faster_lio_main/CMakeLists.txt" "$project_root/src/faster_lio_main/package.xml" \
      "$log_dir/source_snapshot/faster_lio/"
sha256sum "$project_root/devel/lib/faster_lio/run_mapping_online" \
          "$project_root/devel/lib/libfaster_lio.so" \
          "$project_root/external/PX4-Autopilot/build/px4_sitl_inspection/bin/px4" \
          > "$log_dir/source_snapshot/faster_lio_binary.sha256"
ln -sfn "$log_dir" "$project_root/start/logs/latest"
if pgrep -f "^gzserver $project_root/src/drone_stack/worlds/inspection_demo.world" >/dev/null; then
  echo 'The inspection simulator is already running. Stop its launcher before starting another.' >&2
  exit 1
fi
python3 - <<'PY'
import rosgraph
if rosgraph.is_master_online():
    raise SystemExit('A ROS master is already running. Stop the previous simulation before restarting this launcher.')
PY
process_groups=()
cleanup() {
  trap - EXIT INT TERM
  for process_group in "${process_groups[@]}"; do
    kill -INT -- "-$process_group" 2>/dev/null || true
  done
  sleep 2
  for process_group in "${process_groups[@]}"; do
    kill -TERM -- "-$process_group" 2>/dev/null || true
  done
  python3 "$project_root/start/manage_processes.py" unregister "$$" || true
  wait || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
cd "$project_root"
online_mode=false
if [[ "${DRONE_PREBUILT_MAP+x}" == x && -z "${DRONE_PREBUILT_MAP:-}" ]]; then online_mode=true; fi
setsid roslaunch drone_stack stack.launch gui:=false \
  heartbeat_diagnostics:="${DRONE_RECORD_HEARTBEAT:-false}" \
  online_mode:="$online_mode" \
  lio_latency_diagnostics:="${DRONE_RECORD_LIO_TIMING:-false}" \
  prebuilt_map:="${DRONE_PREBUILT_MAP-$project_root/start/maps/inspection_demo_scene.dmap}" \
  fcu_url:="${DRONE_FCU_URL:-udp://:14540@127.0.0.1:14557}" > "$log_dir/stack.log" 2>&1 &
process_groups+=("$!")
python3 - <<'PY'
import time, rosgraph
deadline = time.monotonic() + 20
while time.monotonic() < deadline:
    try:
        rosgraph.Master('/simulation_launcher').getPid()
        break
    except Exception:
        time.sleep(.2)
else:
    raise SystemExit('ROS master did not become ready; check stack.log')
PY
if [[ "${1:-}" != "--no-gui" ]]; then
  setsid rosrun drone_stack drone_operator_gui > "$log_dir/gui.log" 2>&1 &
  process_groups+=("$!")
fi
export DRONE_SPAWN_REQUEST="$log_dir/spawn_request.json"
# Wall time: Gazebo and /clock deliberately do not exist until confirmation.
setsid python3 - <<'PYWAIT' &
import json, os, time, rosgraph
master=rosgraph.Master('/simulation_spawn_wait')
print('Waiting for Qt initial pose confirmation before starting Gazebo.',flush=True)
while True:
    try:
        request=master.getParam('/drone/simulation_spawn_request')
        with open(os.environ['DRONE_SPAWN_REQUEST'],'w') as f:json.dump(request,f)
        break
    except rosgraph.masterapi.MasterError:
        time.sleep(.2)
PYWAIT
spawn_wait_pid=$!
process_groups+=("$spawn_wait_pid")
wait "$spawn_wait_pid"
setsid "$project_root/src/drone_stack/scripts/start_px4_sim.sh" > "$log_dir/px4_gazebo.log" 2>&1 &
sim_pid=$!
process_groups+=("$sim_pid")
if [[ "${1:-}" != "--no-gui" && "${DRONE_SHOW_GAZEBO:-1}" == 1 ]]; then
  (
    # Gazebo setup expands optional unset environment variables.
    set +u
    source /usr/share/gazebo/setup.sh
    set -u
    export GAZEBO_MODEL_DATABASE_URI=""
    export GAZEBO_MODEL_PATH="$project_root/src/drone_stack/models:$project_root/external/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:/usr/share/gazebo-11/models"
    export GAZEBO_PLUGIN_PATH="$project_root/external/PX4-Autopilot/build/px4_sitl_inspection/build_gazebo-classic:${GAZEBO_PLUGIN_PATH:-}"
    exec setsid gzclient --verbose
  ) > "$log_dir/gazebo_client.log" 2>&1 &
  process_groups+=("$!")
fi
echo "Simulation started. Logs: $log_dir. Ctrl-C stops ROS, PX4 and Gazebo together."
wait "$sim_pid"
