#!/usr/bin/env bash
# 与旧地图 start_prior.sh 使用相同启动方式；先激活 ros2_env。
set -eo pipefail

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$WORKSPACE_DIR/maps_output/20261110_152945/obstacle_band/start_prior.sh" "$@"
