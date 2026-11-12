#!/usr/bin/env python3
"""Static checks for the canonical real/sim navigation contract."""
import json
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[4]

def remove_clock(value):
    if isinstance(value,dict):return {k:remove_clock(v) for k,v in value.items() if k!='use_sim_time'}
    if isinstance(value,list):return [remove_clock(v) for v in value]
    return value

def main():
    directory=ROOT/'src/rm_bringup/real_bringup'
    real=yaml.safe_load((directory/'config/real_nav2_params_3d.yaml').read_text())
    # The active simulation launch loads the real profile. The historical
    # sim_nav2_params_3d.yaml is not part of this chain and may be outdated.
    sim_launch=(ROOT/'src/rm_bringup/sim_bringup/launch/sim_nav_3d_total.launch.py').read_text()
    assert 'real_nav2_params_3d.yaml' in sim_launch
    for key in ['param_file','lio_config','rviz_config_file','ui_start_empty','use_nav_plan']:
        assert "'"+key+"': LaunchConfiguration('"+key+"')" in sim_launch,key
    launch_paths=[directory/'launch/real_nav_3d.launch.py',ROOT/'src/rm_bnrobot_nav/launch/nav_load_3d.launch.py']
    for path in launch_paths:
        text=path.read_text();assert 'navigation_3d_core.launch.py' in text and 'real_nav2_params_3d.yaml' in text
    for path in [directory/'launch/real_nav_3d_total.launch.py',ROOT/'src/rm_bringup/sim_bringup/launch/sim_nav_3d_total.launch.py']:
        assert 'motion_arbiter.launch.py' in path.read_text()
    real_lio=yaml.safe_load((ROOT/'src/rm_lio/FASTLIO2_ROS2/fastlio2/config/lio.yaml').read_text())
    sim_lio=yaml.safe_load((ROOT/'src/rm_bnrobot_nav/config/lio_sim.yaml').read_text())
    differences={k:[real_lio.get(k),sim_lio.get(k)] for k in set(real_lio)|set(sim_lio) if real_lio.get(k)!=sim_lio.get(k)}
    assert set(differences)=={'imu_init_max_acc_std'},differences
    assert real['bt_navigator']['ros__parameters']['robot_base_frame']=='nav_base_link'
    assert real['bt_navigator']['ros__parameters']['odom_topic']=='/navigation/odom'
    assert real['/localizer/localizer_node']['ros__parameters']['odom_topic']=='/fastlio2/lio_odom'
    for name in ['local_costmap','global_costmap']:
        c=real[name][name]['ros__parameters']
        assert c['footprint']=="[[-0.3365,-0.283],[0.3365,-0.283],[0.3365,0.283],[-0.3365,0.283]]"
        assert c['robot_base_frame']=='nav_base_link' and 'footprint_update' in c['plugins']
        assert c['obstacle_layer']['observation_sources']=='scan clearing_cloud'
    sensor=(ROOT/'src/rm_bnrobot_sim/urdf/bngu_sentinel/sensor/mid360.urdf.xacro').read_text()
    driver=(ROOT/'src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py').read_text()
    assert 'name="laser_update_rate" value="10"' in sensor
    assert 'publish_freq  = 10.0' in driver
    core=(directory/'launch/navigation_3d_core.launch.py').read_text()
    assert "executable='nav_pose'" in core and "('targetPose', 'goal_pose')" in core
    assert "'require_localization': True" in core and "'require_motion_ready': True" in core
    for path in launch_paths + [directory/'launch/real_nav_3d_total.launch.py',ROOT/'src/rm_bringup/sim_bringup/launch/sim_nav_3d_total.launch.py']:
        assert 'patrol_file' in path.read_text() and 'patrol_mode' in path.read_text()
    result={'shared_navigation_core':True,'shared_motion_arbiter':True,
            'simulation_uses_real_nav2_profile':True,'lio_profile_differences':differences,
            'sim_actuator_input':'/can_twist/command_frame -> /cmd_vel_sim',
            'allowed_backend_difference':'USB-CAN hardware vs encoded-frame Gazebo bridge',
            'scope':'software navigation parity; sensors and wheel/MCU physics are not identical'}
    path=ROOT/'log/sim_real_parity_20261014/static_parity.json';path.parent.mkdir(exist_ok=True,parents=True);path.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))

if __name__=='__main__':main()
