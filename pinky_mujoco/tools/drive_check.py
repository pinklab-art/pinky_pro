"""Exercise straight driving, turning and the bridge command watchdog."""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped


def main():
    rclpy.init()
    node = Node('pinky_drive_check')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    latest = []
    node.create_subscription(PoseStamped, '/ground_truth', lambda msg: latest.append(msg), 10)
    deadline = time.monotonic() + 10
    while not latest and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not latest:
        raise RuntimeError('No simulator ground truth')

    def xyz():
        p, q = latest[-1].pose.position, latest[-1].pose.orientation
        return [p.x, p.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)]

    def drive(v, w, seconds, publish=True):
        command = Twist()
        command.linear.x, command.angular.z = float(v), float(w)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if publish:
                pub.publish(command)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.02)

    result = {'start': xyz()}
    try:
        drive(0.12, 0, 3)
        result['forward'] = xyz()
        # Withhold messages: simulator must stop after 0.5 seconds.
        drive(0, 0, 1.5, publish=False)
        result['watchdog_start'] = xyz()
        drive(0, 0, 1, publish=False)
        result['watchdog_end'] = xyz()
        drive(0, 0.8, 9)
        drive(0, 0, 1)
        result['after_rotation'] = xyz()
        displacement = math.dist(result['start'][:2], result['forward'][:2])
        drift = math.dist(result['watchdog_start'][:2], result['watchdog_end'][:2])
        result['forward_distance'] = displacement
        result['watchdog_drift'] = drift
        Path('drive-check.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
        assert 0.25 < displacement < 0.45, 'Forward command did not move robot correctly'
        assert drift < 0.005, 'Command watchdog did not stop robot'
    finally:
        pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
