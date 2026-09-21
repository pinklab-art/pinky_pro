"""Observe live ROS interfaces, TF and lifecycle state; save a JSON report."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from lifecycle_msgs.srv import GetState


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='ros-check.json')
    parser.add_argument('--map', action='store_true', dest='require_map')
    parser.add_argument('--nav', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Node('pinky_verification', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    counts, latest = Counter(), {}

    def record(topic):
        def callback(message):
            counts[topic] += 1
            latest[topic] = message
        return callback

    subscriptions = [node.create_subscription(Clock, '/clock', record('clock'), 10),
                     node.create_subscription(Odometry, '/odom', record('odom'), 10),
                     node.create_subscription(LaserScan, '/scan', record('scan'), qos_profile_sensor_data),
                     node.create_subscription(PoseStamped, '/ground_truth', record('ground_truth'), 10)]
    subscriptions.append(node.create_subscription(OccupancyGrid, '/map', record('map'),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE)))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    deadline = time.monotonic() + 20
    frames = {}
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        for parent, child in [('odom', 'base_footprint'), ('base_footprint', 'rplidar_link'),
                              ('map', 'base_footprint')] if args.require_map else [('odom', 'rplidar_link')]:
            try:
                transform = buffer.lookup_transform(parent, child, rclpy.time.Time())
                frames[f'{parent}->{child}'] = [transform.transform.translation.x,
                                               transform.transform.translation.y]
            except Exception:
                pass
        if counts['scan'] >= 50 and (not args.require_map or ('map' in latest and len(frames) == 3)):
            break
    report = {'counts': dict(counts), 'transforms': frames}
    errors = []
    for name in ('clock', 'odom', 'scan', 'ground_truth'):
        if counts[name] < 2:
            errors.append(f'Missing live topic: {name}')
    if args.require_map:
        if 'map' not in latest:
            errors.append('No SLAM map received')
        else:
            grid = latest['map']
            report['map'] = {'width': grid.info.width, 'height': grid.info.height,
                             'resolution': grid.info.resolution,
                             'known_cells': sum(v >= 0 for v in grid.data),
                             'occupied_cells': sum(v > 65 for v in grid.data)}
            if report['map']['occupied_cells'] < 20:
                errors.append('Map has too few occupied cells')
        if len(frames) != 3:
            errors.append('Incomplete map/odom/base/lidar TF chain')
    elif not frames:
        errors.append('Missing odom-to-lidar transform')
    if args.nav:
        states = {}
        for name in ('controller_server', 'planner_server', 'smoother_server', 'behavior_server', 'bt_navigator'):
            client = node.create_client(GetState, f'/{name}/get_state')
            if not client.wait_for_service(timeout_sec=3):
                errors.append(f'Lifecycle service missing: {name}')
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            state = future.result().current_state.label if future.done() else 'timeout'
            states[name] = state
            if state != 'active':
                errors.append(f'{name}: {state}')
        report['nav2_states'] = states
    if 'odom' in latest:
        p = latest['odom'].pose.pose.position
        report['odom_xy'] = [p.x, p.y]
    if 'ground_truth' in latest:
        p = latest['ground_truth'].pose.position
        report['ground_truth_xy'] = [p.x, p.y]
    report['errors'] = errors
    Path(args.output).write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(bool(errors))


if __name__ == '__main__':
    main()
