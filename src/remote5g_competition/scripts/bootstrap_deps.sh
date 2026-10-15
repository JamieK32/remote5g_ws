#!/usr/bin/env bash
set -euo pipefail
scene_deps_dir="${REMOTE5G_SCENE_DEPS_DIR:-$HOME/.cache/remote5g_scene_libs}"
mkdir -p "$scene_deps_dir"
curl -fsSL --max-time 30 https://bootstrap.pypa.io/pip/pip.pyz -o "$scene_deps_dir/pip.pyz"
python3 "$scene_deps_dir/pip.pyz" install --target "$scene_deps_dir" qrcode==8.2 zxing-cpp==2.3.0
# Add this path to PYTHONPATH when using a custom directory; the default is auto-discovered.
