#!/usr/bin/env bash
# Terminal B: owns localization, planning, MAVROS, camera processing and Qt.
source "$(dirname "${BASH_SOURCE[0]}")/sim_common.sh"
lock_key="$(printf '%s' "$ROS_MASTER_URI" | sha256sum | cut -c1-16)"
exec 9>"/tmp/drone_navigation_${UID}_${lock_key}.lock"
flock -n 9 || { echo '该 ROS 地址的导航启动器已运行'; exit 1; }
mode="${1:-inspection}"
map_path="${2:-$project_root/start/maps/inspection_demo_scene.dmap}"
case "$mode" in
  mapping) [[ $# -le 1 ]] || { echo '建图模式不接受地图参数'; exit 2; }; map_path=''; online=true ;;
  inspection) [[ $# -le 2 && -f "$map_path" ]] || { echo '巡检模式需要存在的 .dmap 文件'; exit 2; }; map_path="$(realpath "$map_path")"; online=false ;;
  *) echo 'Usage: bash start/start_navigation.sh mapping | inspection [map.dmap]'; exit 2 ;;
esac
python3 - <<'PY'
import rosgraph
master=rosgraph.Master('/split_navigation_launcher')
try:
    if not master.getParam('/drone/split_world_launcher'): raise ValueError()
except Exception: raise SystemExit('请先在另一终端运行 bash start/start_gazebo.sh')
nodes={n for ns in master.getSystemState() for _, publishers in ns for n in publishers}
if '/drone_flight_manager' in nodes or '/drone_map_session' in nodes:
    raise SystemExit('定位/规划程序已运行，请勿重复启动')
if master.hasParam('/drone/simulation_spawn_request'):
    raise SystemExit('本次仿真已提交出生位姿；切换或重启模式请先停止两个入口，再重新启动')
PY
log_dir="${DRONE_LOG_DIR:-$project_root/start/logs/${mode}_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$log_dir"
setsid roslaunch drone_stack stack.launch gui:=true online_mode:="$online" prebuilt_map:="$map_path" \
  heartbeat_diagnostics:="${DRONE_RECORD_HEARTBEAT:-false}" \
  lio_latency_diagnostics:="${DRONE_RECORD_LIO_TIMING:-false}" \
  fcu_url:="${DRONE_FCU_URL:-udp://:14540@127.0.0.1:14557}" > "$log_dir/stack.log" 2>&1 &
stack_pid=$!; process_groups+=("$stack_pid")
setsid python3 "$project_root/src/drone_stack/scripts/check_sim_startup.py" \
  --mode "$mode" --output "$log_dir/startup_check.json" > "$log_dir/startup_check.log" 2>&1 &
process_groups+=("$!")
echo "已启动${mode}模式及 Qt；关键话题检查结果见 $log_dir/startup_check.log"
echo '巡检模式先在 Qt 选择位置、拖动朝向并确认。未就绪时不自动解锁或起飞。'
# Checker failure must not tear down LIO while an operator may be flying.
wait "$stack_pid"
