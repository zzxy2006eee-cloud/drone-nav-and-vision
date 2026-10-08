#!/usr/bin/env bash
# Terminal A: owns ROS master, PX4, Gazebo and the sensor-equipped vehicle.
source "$(dirname "${BASH_SOURCE[0]}")/sim_common.sh"
if [[ $# -gt 0 ]]; then echo 'Usage: bash start/start_gazebo.sh (DRONE_SHOW_GAZEBO=0 disables the viewer)'; exit 2; fi
lock_key="$(printf '%s' "$ROS_MASTER_URI" | sha256sum | cut -c1-16)"
exec 9>"/tmp/drone_world_${UID}_${lock_key}.lock"
flock -n 9 || { echo '该 ROS 地址的仿真启动器已运行'; exit 1; }
log_dir="${DRONE_LOG_DIR:-$project_root/start/logs/world_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$log_dir"
python3 - <<'PY'
import rosgraph
if rosgraph.is_master_online():
    raise SystemExit('已有 ROS master；请停止旧启动器后再启动，禁止混用旧的一体启动器。')
PY
setsid roscore > "$log_dir/roscore.log" 2>&1 &
master_pid=$!; process_groups+=("$master_pid")
python3 - <<'PY'
import time, rosgraph
master=rosgraph.Master('/split_world_launcher')
for _ in range(100):
    try:
        master.getPid()
        master.setParam('/use_sim_time',True)
        master.setParam('/drone/split_world_launcher',True)
        break
    except Exception: time.sleep(.2)
else: raise SystemExit('ROS master 启动失败，请查看 roscore.log')
PY
echo "仿真入口已就绪；请在另一终端运行 start_navigation.sh。巡检模式等待 Qt 确认出生位姿。日志：$log_dir"
export DRONE_SPAWN_REQUEST="$log_dir/spawn_request.json"
setsid python3 - <<'PY' &
import json, os, time, rosgraph
master=rosgraph.Master('/split_spawn_wait')
while True:
    try:
        request=master.getParam('/drone/simulation_spawn_request')
    except rosgraph.masterapi.MasterError:
        time.sleep(.2)
        continue
    with open(os.environ['DRONE_SPAWN_REQUEST'],'w') as f: json.dump(request,f)
    break
PY
spawn_pid=$!; process_groups+=("$spawn_pid"); wait "$spawn_pid"
export HEADLESS=1
setsid "$project_root/src/drone_stack/scripts/start_px4_sim.sh" > "$log_dir/px4_gazebo.log" 2>&1 &
sim_pid=$!; process_groups+=("$sim_pid")
if [[ "${DRONE_SHOW_GAZEBO:-1}" == 1 ]]; then
  (
    set +u
    source /usr/share/gazebo/setup.sh
    export GAZEBO_MODEL_DATABASE_URI=""
    export GAZEBO_MODEL_PATH="$project_root/src/drone_stack/models:$project_root/external/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:/usr/share/gazebo-11/models"
    export GAZEBO_PLUGIN_PATH="$project_root/external/PX4-Autopilot/build/px4_sitl_inspection/build_gazebo-classic:${GAZEBO_PLUGIN_PATH:-}"
    exec setsid gzclient --verbose
  ) > "$log_dir/gazebo_client.log" 2>&1 &
  process_groups+=("$!")
  echo "Gazebo 界面已随仿真启动；窗口日志：$log_dir/gazebo_client.log"
fi
echo '已按请求位姿启动 PX4/Gazebo，机体含雷达 IMU、倾斜 MID360、前视及下视相机。'
wait "$sim_pid"
