#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [[ -n "${DRONE_SPAWN_REQUEST:-}" ]]; then
  mapfile -t spawn_values < <(python3 - <<'PYPOSE'
import json,math,os
with open(os.environ['DRONE_SPAWN_REQUEST']) as f:r=json.load(f)
values=[float(r[k]) for k in ['x','y','z','yaw_deg']]
if not all(math.isfinite(v) for v in values):raise ValueError('Invalid spawn pose')
for v in values[:3]:print(v)
print(math.radians(values[3]))
PYPOSE
)
  [[ "${#spawn_values[@]}" == 4 ]] || { echo 'Invalid spawn request' >&2; exit 1; }
  export DRONE_SPAWN_X="${spawn_values[0]}" DRONE_SPAWN_Y="${spawn_values[1]}" DRONE_SPAWN_Z="${spawn_values[2]}" DRONE_SPAWN_YAW="${spawn_values[3]}"
fi
python3 "$project_root/src/drone_stack/scripts/prepare_px4_sitl.py"
source /opt/ros/noetic/setup.bash
export PYTHONPATH="$project_root/external/px4_python_deps:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${LD_LIBRARY_PATH:-}"
export GIT_SUBMODULES_ARE_EVIL=1
if [[ "${DRONE_PX4_INTERACTIVE:-0}" != 1 ]]; then
  export NO_PXH=1
else
  unset NO_PXH
fi
export PX4_GAZEBO_JOBS=2
export PX4_SITL_WORLD="${DRONE_SIM_WORLD:-$project_root/src/drone_stack/worlds/inspection_demo.world}"
cd "$project_root/external/PX4-Autopilot"
make px4_sitl_inspection gazebo-classic_inspection_quad
