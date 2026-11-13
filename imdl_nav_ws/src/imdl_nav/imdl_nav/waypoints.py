"""Drive to several points from a YAML file, one after the other.

    ros2 run imdl_nav waypoints                         # config/waypoints.yaml
    ros2 run imdl_nav waypoints ~/maps/my_points.yaml

Ctrl-C cancels the drive.
"""

import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.signals import SignalHandlerOptions

from imdl_nav.nav_client import NavClient


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        path = os.path.join(
            get_package_share_directory('imdl_nav'), 'config', 'waypoints.yaml')
    with open(path) as f:
        config = yaml.safe_load(f)
    points = config['waypoints']
    repeat = int(config.get('repeat', 1))

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = NavClient()
    node.get_logger().info(f'{len(points)} points from {path}, {repeat} round(s)')
    try:
        for round_number in range(1, repeat + 1):
            for point in points:
                name = point.get('name', '?')
                node.get_logger().info(f'Round {round_number}, point "{name}"')
                if not node.drive_to(point['x'], point['y'], point.get('yaw', 0.0)):
                    node.get_logger().error(f'Point "{name}" not reached, stopping.')
                    raise SystemExit(1)
        node.get_logger().info('All points reached.')
    except KeyboardInterrupt:
        node.cancel()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
