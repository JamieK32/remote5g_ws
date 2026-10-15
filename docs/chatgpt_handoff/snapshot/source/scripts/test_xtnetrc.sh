#!/usr/bin/env bash
set -euo pipefail
ws_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "$ws_dir/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${XTNETRC_TEST_DOMAIN:-75}"
export GAZEBO_MASTER_URI="http://127.0.0.1:${XTNETRC_TEST_PORT:-11375}"
export GAZEBO_IP=127.0.0.1
export GAZEBO_MODEL_DATABASE_URI=''
report_dir="${1:-$ws_dir/reports/xtnetrc/latest}"
mkdir -p "$report_dir"
# Refuse to share a Gazebo master; never stop an existing server.
python3 - <<'PY'
import os,socket,urllib.parse
u=urllib.parse.urlparse(os.environ['GAZEBO_MASTER_URI'])
s=socket.socket(); s.settimeout(1)
if s.connect_ex((u.hostname,u.port))==0: raise SystemExit('Test port already occupied; choose XTNETRC_TEST_PORT')
PY
setsid ros2 launch remote5g_sim sim.launch.py gui:=false port:="${XTNETRC_TEST_PORT:-11375}" > "$report_dir/gazebo.log" 2>&1 &
launch_pid=$!
cleanup() {
  # Only the process group created by this runner is signalled.
  kill -INT -- "-$launch_pid" 2>/dev/null || true
  for _ in {1..30}; do
    kill -0 -- "-$launch_pid" 2>/dev/null || break
    sleep 0.2
  done
  kill -TERM -- "-$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
timeout --signal=INT --kill-after=10s 300s ros2 run remote5g_sim regression.py --output "$report_dir" > "$report_dir/regression.log" 2>&1
cat "$report_dir/regression.log"
