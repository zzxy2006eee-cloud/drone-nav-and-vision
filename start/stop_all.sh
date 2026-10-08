#!/usr/bin/env bash
# Stop all simulation processes belonging to this workspace; no ROS needed.
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -ne 0 ]]; then echo 'Usage: bash start/stop_all.sh'; exit 2; fi
exec python3 "$project_root/start/manage_processes.py" stop
