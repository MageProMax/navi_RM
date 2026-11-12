#!/usr/bin/env python3
"""Exercise the current real-map replica through the same client used by the UI.

Run after building: python src/rm_bringup/sim_bringup/test/real_prior_sim_smoke.py
Uses an isolated ROS domain, Gazebo port, simulated CAN, and bounded waits.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time
import traceback

import yaml

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from build_real_prior_sim import DEFAULT_MAP, build, digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-dir', type=Path, default=DEFAULT_MAP)
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'log/real_prior_sim')
    parser.add_argument('--goal', type=float, nargs=2, action='append', help='Map x y; repeat for a route')
    args = parser.parse_args()
    directory = args.output_dir.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    map_dir = args.map_dir.resolve()
    replica = build(map_dir, map_dir / 'simulation')
    # Restrict DDS to loopback and use a separate Gazebo master from normal use.
    os.environ.setdefault('ROS_DOMAIN_ID', '211')
    if os.environ['ROS_DOMAIN_ID'] == '0':
        raise ValueError('Use an isolated nonzero ROS_DOMAIN_ID')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('GAZEBO_MASTER_URI', 'http://127.0.0.1:11511')
    os.environ.setdefault('GAZEBO_MODEL_DATABASE_URI', '')
    os.environ.setdefault('RMW_FASTRTPS_PUBLICATION_MODE', 'ASYNCHRONOUS')
    os.environ.setdefault('ROS_LOG_DIR', str(directory / 'ros'))
    os.environ['NAV_MAP_DIR'] = str(map_dir)
    import rclpy
    from rclpy.parameter import Parameter, parameter_value_to_python
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    from sensor_msgs.msg import Imu, LaserScan, PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String, UInt8MultiArray
    from livox_ros_driver2.msg import CustomMsg
    from rm_interfaces.msg import Goal, NavigationStatus, OwnPara
    from rm_interfaces.srv import ConfigurePatrol
    from std_srvs.srv import Trigger
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.srv import GetParameters

    result = {'passed': False, 'replica': replica, 'received': {}, 'goals': [],
              'failures': [], 'runtime_parameters': {}, 'spin_distance_m': 0., 'spin_rotation_rad': 0.}
    registrations, statuses, frames = [], [], []
    truth = None
    spinning = False
    spin_started = None
    spin_durations = []
    permission = False
    scan_summary = {}
    spin_events = []
    previous_truth = None
    previous_yaw = None
    start_wall = time.monotonic()
    log = (directory / 'nodes.log').open('w')
    command = ['bash', str(ROOT / 'src/rm_bringup/sim_bringup/scripts/start_sim_prior_nav.sh'), 'use_rviz:=false', 'use_ui:=false']
    process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('real_prior_replica_acceptance', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def count(name):
        result['received'][name] = result['received'].get(name, 0) + 1

    def truth_cb(msg):
        nonlocal truth, previous_truth, previous_yaw
        truth = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        yaw = math.atan2(2. * (q.w * q.z + q.x * q.y), 1. - 2. * (q.y * q.y + q.z * q.z))
        if previous_truth and spinning:
            result['spin_distance_m'] += math.dist(truth, previous_truth)
            result['spin_rotation_rad'] += math.atan2(math.sin(yaw - previous_yaw), math.cos(yaw - previous_yaw))
        previous_truth = truth
        previous_yaw = yaw
        count('gazebo_truth')

    def spin_cb(msg):
        nonlocal spinning, spin_started
        if msg.data and not spinning:
            spin_started = now()
        elif spinning and not msg.data and spin_started is not None:
            spin_durations.append(now() - spin_started)
        if msg.data != spinning:
            spin_events.append({'sim_time': now(), 'active': msg.data,
                                'permission': permission, 'scan': dict(scan_summary),
                                'truth_xy': list(truth) if truth else None})
        spinning = msg.data

    def permission_cb(msg):
        nonlocal permission
        permission = msg.data

    def scan_cb(msg):
        count('scan')
        finite = [r for r in msg.ranges if math.isfinite(r)]
        scan_summary.update(min_range_m=min(finite, default=None),
                            age=now() - msg.header.stamp.sec - msg.header.stamp.nanosec * 1e-9,
                            nearby_xy=[[r * math.cos(msg.angle_min + i * msg.angle_increment),
                                        r * math.sin(msg.angle_min + i * msg.angle_increment)]
                                       for i, r in enumerate(msg.ranges) if math.isfinite(r) and r < .6])

    def status_cb(msg):
        statuses.append(msg)

    def frame_cb(msg):
        frames.append((now(), bytes(msg.data)))
        count('can_frame')

    subscriptions = [
        node.create_subscription(Odometry, '/odom', truth_cb, qos_profile_sensor_data),
        node.create_subscription(Bool, '/spin_active', spin_cb, 10),
        node.create_subscription(Bool, '/spin_motion_permitted', permission_cb, 10),
        node.create_subscription(LaserScan, '/scan', scan_cb, qos_profile_sensor_data),
        node.create_subscription(NavigationStatus, '/navigation/client/status', status_cb, 10),
        node.create_subscription(UInt8MultiArray, '/can_twist/command_frame', frame_cb, 10),
        node.create_subscription(String, '/localizer/registration_status',
                                 lambda m: registrations.append(json.loads(m.data)), 100),
    ]
    for name, topic, cls in [
        ('livox', '/livox/lidar', CustomMsg), ('imu', '/livox/imu', Imu),
        ('lio', '/fastlio2/lio_odom', Odometry),
        ('navigation_cloud', '/fastlio2/navigation_cloud', PointCloud2),
        ('clearing_cloud', '/clearing_cloud', PointCloud2),
        ('registration_cloud', '/localizer/registration_cloud', PointCloud2),
        ('navigation_odom', '/navigation/odom', Odometry),
    ]:
        subscriptions.append(node.create_subscription(cls, topic, lambda m, key=name: count(key), qos_profile_sensor_data))
    stop_pub = node.create_publisher(Bool, '/emergency_stop', 10)
    health_pub = node.create_publisher(OwnPara, '/own_para', 10)

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
            for _ in range(8):
                rclpy.spin_once(node, timeout_sec=0.)
            if process.poll() is not None:
                raise RuntimeError('Simulation launch exited; see nodes.log')

    def wait(predicate, timeout, description):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            pump(.05)
            if predicate():
                return
        raise AssertionError('Timeout: ' + description)

    def service(cls, name, request, timeout=15):
        client = node.create_client(cls, name)
        try:
            wait(lambda: client.service_is_ready(), timeout, name + ' discovery')
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                future = client.call_async(request)
                response_deadline = min(deadline, time.monotonic() + 3.) if cls in (GetState, GetParameters) else deadline
                while not future.done() and time.monotonic() < response_deadline:
                    pump(.05)
                if future.done():
                    return future.result()
                future.cancel()
            raise AssertionError('Timeout: ' + name + ' response')
        finally:
            node.destroy_client(client)

    def parameters(name, keys):
        req = GetParameters.Request(names=keys)
        values = service(GetParameters, name + '/get_parameters', req).values
        return dict(zip(keys, map(parameter_value_to_python, values)))

    def map_xy():
        pose = buffer.lookup_transform('map', 'base_link', Time())
        return [pose.transform.translation.x, pose.transform.translation.y]

    def healthy_registration():
        return any(r.get('accepted') and r.get('height_filter') for r in registrations[-20:])

    def hp(value):
        msg = OwnPara()
        msg.maximum_hp, msg.current_hp, msg.hp_deduction_reason = 400, value, 0
        health_pub.publish(msg)

    try:
        print('Waiting for Gazebo, LIO, height-filtered GICP and CAN...', flush=True)
        wait(lambda: truth and frames and statuses and healthy_registration(), 240, 'sensor/localization/motion readiness')
        lifecycle = {}
        for name in ('map_server', 'controller_server', 'planner_server', 'bt_navigator', 'velocity_smoother'):
            lifecycle[name] = service(GetState, '/' + name + '/get_state', GetState.Request()).current_state.label
        result['lifecycle'] = lifecycle
        assert all(value == 'active' for value in lifecycle.values()), lifecycle
        expected_file = ROOT / 'src/rm_bringup/real_bringup/config/real_nav2_params_3d.yaml'
        expected = yaml.safe_load(expected_file.read_text())
        # Check the important runtime values, not just source-file strings.
        contracts = {
            '/controller_server': ['FollowPath.plugin', 'FollowPath.motion_model', 'FollowPath.CostCritic.consider_footprint'],
            '/local_costmap/local_costmap': ['footprint', 'plugins', 'obstacle_layer.observation_sources', 'obstacle_layer.clearing_cloud.topic'],
            '/global_costmap/global_costmap': ['footprint', 'plugins', 'obstacle_layer.combination_method', 'obstacle_layer.footprint_clearing_enabled', 'obstacle_layer.clearing_cloud.topic'],
            '/spin_navigation_adapter': ['close_wall_mode', 'recovery.enabled'],
        }
        for name, keys in contracts.items():
            actual = parameters(name, ['use_sim_time'] + keys)
            result['runtime_parameters'][name] = actual
            assert actual['use_sim_time'] is True, (name, actual)
            config = expected
            for part in name.strip('/').split('/'):
                config = config[part]
            config = config['ros__parameters']
            for key in keys:
                if key in config:
                    value = config[key]
                else:
                    value = config
                    for part in key.split('.'):
                        value = value[part]
                assert actual[key] == value, (name, key, actual[key], value)
        arbiter = parameters('/can_twist_node', ['dry_run', 'use_sim_time', 'spin_duration'])
        assert arbiter['dry_run'] and arbiter['use_sim_time'], arbiter
        result['runtime_parameters']['/can_twist_node'] = arbiter
        localizer = parameters('/localizer/localizer_node', ['map_path', 'registration_method', 'accumulation_seconds', 'use_sim_time'])
        assert Path(localizer['map_path']).resolve() == map_dir / 'map.pcd', localizer
        assert localizer['registration_method'] == 'gicp' and localizer['accumulation_seconds'] == .5
        result['runtime_parameters']['/localizer/localizer_node'] = localizer
        scan_parameters = parameters('/pointcloud_to_laserscan', ['cloud_topic', 'ground_filter_enabled'])
        assert scan_parameters == {'cloud_topic': '/fastlio2/navigation_cloud', 'ground_filter_enabled': True}, scan_parameters
        result['runtime_parameters']['/pointcloud_to_laserscan'] = scan_parameters
        assert not any(name == 'nav_plan_node' for name, namespace in node.get_node_names_and_namespaces())
        x, y, yaw = replica['initial_pose']
        # Leave the nearby starting wall before testing the swept spin envelope.
        goals = args.goal or [(x + 1., y - 2.), (x + 1., y), (x, y)]
        hp(400)
        pump(.5)
        for index, (gx, gy) in enumerate(goals):
            goal = Goal(goal_frame_id='map', goal_x=float(gx), goal_y=float(gy), goal_z=0.)
            req = ConfigurePatrol.Request(mode='manual', frame_id='map', loop=False,
                                         wait_seconds=0., health_threshold=.25,
                                         point_names=['acceptance'], points=[goal])
            assert service(ConfigurePatrol, '/navigation/client/configure', req).success
            marker = len(statuses)
            assert service(Trigger, '/navigation/client/start', Trigger.Request()).success
            print(f'Goal {index + 1}: ({gx:.3f}, {gy:.3f})', flush=True)
            def current_statuses():
                return [s for s in statuses[marker:]
                        if abs(s.target.goal_x - gx) < 1e-6 and abs(s.target.goal_y - gy) < 1e-6]
            wait(lambda: any(s.active for s in current_statuses()), 30, 'client dispatch')
            if index == 1:
                wait(lambda: frames and math.hypot(*struct.unpack('<hhhh', frames[-1][1])[:2]) > 50.,
                     30, 'translation before hit')
                hp(399)
                wait(lambda: spinning, 20, 'hit spin')
            wait(lambda: any(s.state in ('complete', 'failed') for s in current_statuses()), 240, 'goal completion')
            state = next(s.state for s in reversed(current_statuses()) if s.state in ('complete', 'failed'))
            localized = map_xy()
            measurement = {'target': [gx, gy], 'client_state': state, 'localized_xy': localized,
                           'truth_xy': list(truth), 'truth_error_m': math.dist(truth, [gx, gy]),
                           'localized_error_m': math.dist(localized, [gx, gy]),
                           'localization_error_m': math.dist(truth, localized)}
            result['goals'].append(measurement)
            print(json.dumps(measurement), flush=True)
            if state != 'complete' or measurement['truth_error_m'] >= .35 or measurement['localization_error_m'] >= .20:
                result['failures'].append(measurement)
            wait(lambda: not spinning, 120, 'spin expiry')
        if len(goals) > 1:
            assert result['spin_distance_m'] > .3, result['spin_distance_m']
            assert result['spin_rotation_rad'] > 8., result['spin_rotation_rad']
            assert any(9. < duration < 11. for duration in spin_durations), spin_durations
        # Latched stop must zero the actual encoded payload, including any spin.
        stop_pub.publish(Bool(data=True))
        pump(.5)
        marker = len(frames)
        stationary = list(truth)
        pump(1.)
        assert frames[marker:] and all(struct.unpack('<hhhh', data)[:3] == (0, 0, 0)
                                      for _, data in frames[marker:]), 'Nonzero CAN command after emergency stop'
        assert math.dist(truth, stationary) < .05, 'Robot moved while stopped'
        result['emergency_stop_verified'] = True
        for name in ('livox', 'imu', 'lio', 'navigation_cloud', 'scan', 'clearing_cloud', 'registration_cloud', 'navigation_odom', 'can_frame'):
            assert result['received'].get(name, 0) > 0, name
        assert healthy_registration(), 'Registration stopped accepting scans'
        result['spin_durations_sim_s'] = spin_durations
        result['registration'] = {'samples': len(registrations), 'accepted': sum(r['accepted'] for r in registrations),
                                  'height_filter': registrations[-1].get('height_filter')}
        result['passed'] = not result['failures']
        assert result['passed'], result['failures']
    except BaseException as error:
        result['error'] = str(error)
        result['traceback'] = traceback.format_exc()
        raise
    finally:
        if rclpy.ok():
            stop_pub.publish(Bool(data=True))
        result['wall_seconds'] = time.monotonic() - start_wall
        result['sim_seconds'] = now()
        result['spin_events'] = spin_events
        result['spin_durations_sim_s'] = spin_durations
        result['source_unchanged'] = all(digest(map_dir / name) == replica['source_hashes'][name]
                                         for name in ('map.yaml', 'map.pcd', 'height_filter_metadata.json', 'start_pose.env'))
        image_path = map_dir / yaml.safe_load((map_dir / 'map.yaml').read_text())['image']
        result['source_unchanged'] &= digest(image_path) == replica['source_hashes']['map_image']
        if not result['source_unchanged']:
            result['passed'] = False
        (directory / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        (directory / 'registration_samples.json').write_text(json.dumps(registrations, indent=2) + '\n')
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    assert result['source_unchanged'], 'Original map inputs changed'
    print('PASS real-map replica: UI client, LIO/GICP, Nav2, CAN, truth accuracy, spin and stop', flush=True)


if __name__ == '__main__':
    main()
