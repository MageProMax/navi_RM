#!/usr/bin/env bash
# Replica of start_real_prior_nav.sh using the same prior map and navigation core.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "$SCRIPT_DIR/../../../.." && pwd)"
MAP_DIR="${NAV_MAP_DIR:-$WORKSPACE_DIR/maps_output/20261110_152945/obstacle_band}"
SIM_DIR="${NAV_SIM_DIR:-$MAP_DIR/simulation}"
cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
source "$MAP_DIR/start_pose.env"
python "$SCRIPT_DIR/build_real_prior_sim.py" --map-dir "$MAP_DIR" --output-dir "$SIM_DIR"
SPAWN_Z="$(python -c 'import json,sys; print(json.load(open(sys.argv[1]))["spawn_z"])' "$SIM_DIR/manifest.json")"

exec ros2 launch sim_bringup sim_nav_3d_total.launch.py \
  navigation_mode:=prior gui:=false use_rviz:=true use_ui:=true ui_start_empty:=true \
  obstacle_cloud_topic:=/fastlio2/navigation_cloud \
  use_nav_plan:=false patrol_mode:=formal \
  localizer_backend:=gicp localizer_accumulation_seconds:=0.5 \
  world:="$SIM_DIR/replica.world" \
  map:="$MAP_DIR/map.yaml" pcd_map:="$MAP_DIR/map.pcd" \
  initial_pose_x:="$NAV_INITIAL_X" initial_pose_y:="$NAV_INITIAL_Y" \
  initial_pose_yaw:="$NAV_INITIAL_YAW" initial_pose_z:="$SPAWN_Z" \
  lio_config:="$WORKSPACE_DIR/src/rm_bnrobot_nav/config/lio_sim.yaml" \
  param_file:="$WORKSPACE_DIR/src/rm_bringup/real_bringup/config/real_nav2_params_3d.yaml" \
  rviz_config_file:="$MAP_DIR/nav_filtered.rviz" \
  "$@"
