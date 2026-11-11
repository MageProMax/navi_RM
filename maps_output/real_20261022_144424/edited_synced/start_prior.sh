#!/usr/bin/env bash
# Run from the ros2_env environment with the robot at this map's recorded start.
set -eo pipefail
MAP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "$MAP_DIR/../../.." && pwd)"
cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
source "$MAP_DIR/start_pose.env"

exec ros2 launch real_bringup real_nav_3d_total.launch.py \
  navigation_mode:=prior \
  start_lidar:=true use_rviz:=true use_ui:=true ui_start_empty:=true \
  use_nav_plan:=false patrol_mode:=formal \
  localizer_backend:=gicp localizer_accumulation_seconds:=0.5 \
  map:="$MAP_DIR/map.yaml" \
  pcd_map:="$MAP_DIR/map.pcd" \
  initial_pose_x:="$NAV_INITIAL_X" \
  initial_pose_y:="$NAV_INITIAL_Y" \
  initial_pose_yaw:="$NAV_INITIAL_YAW" \
  model:="$WORKSPACE_DIR/src/rm_bnrobot_sim/urdf/bngu_sentinel/bnbot_real.xacro" \
  lio_config:="$WORKSPACE_DIR/src/rm_lio/FASTLIO2_ROS2/fastlio2/config/lio.yaml" \
  param_file:="$WORKSPACE_DIR/src/rm_bringup/real_bringup/config/real_nav2_params_3d.yaml" \
  rviz_config_file:="$WORKSPACE_DIR/src/rm_bringup/real_bringup/rviz/nav_final.rviz" \
  "$@"
