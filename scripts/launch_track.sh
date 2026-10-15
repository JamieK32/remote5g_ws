#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
color="${1:-red}"
case "$color" in red|blue) ;; *) echo 'Usage: launch_track.sh [red|blue] [Gazebo arguments...]' >&2; exit 2 ;; esac
if (( $# )); then shift; fi
export GAZEBO_MODEL_PATH="$workspace_dir/simulation/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
# Local models and worlds need no online model database.
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MASTER_URI="${REMOTE5G_GAZEBO_MASTER_URI:-http://127.0.0.1:11365}"
export GAZEBO_IP="${GAZEBO_IP:-127.0.0.1}"
exec gazebo "$workspace_dir/simulation/worlds/athletics_400m_${color}.world" "$@"
