"""Save the map automatically when explore_lite has finished exploring.

Runs on the PC, next to explore_lite.

    subscribes:  explore/status  (explore_lite_msgs/msg/ExploreStatus)
    runs:        map_saver_cli -f <map>, once the status is "exploration_complete"

    ros2 run imdl_nav save_on_complete --ros-args -p map:=~/maps/my_map
"""

import os
import subprocess

import rclpy
from explore_lite_msgs.msg import ExploreStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class SaveOnComplete(Node):

    def __init__(self):
        super().__init__('save_on_complete')
        self.map = os.path.expanduser(self.declare_parameter('map', '~/maps/explored_map').value)
        self.done = False

        # Same QoS as explore_lite: we also get the last status if we start late
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ExploreStatus, 'explore/status', self.on_status, qos)
        self.get_logger().info(f'Waiting for exploration to finish, map will be saved to {self.map}')

    def on_status(self, msg):
        if self.done or msg.status != ExploreStatus.EXPLORATION_COMPLETE:
            return
        self.save_map()
        self.done = True

    def save_map(self):
        use_sim_time = str(self.get_parameter('use_sim_time').value).lower()
        command = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', self.map, '--free', '0.196',
            '--ros-args', '-p', 'save_map_timeout:=20.0', '-p', f'use_sim_time:={use_sim_time}',
        ]
        os.makedirs(os.path.dirname(self.map), exist_ok=True)
        self.get_logger().info('Exploration complete, saving map ...')

        if subprocess.run(command).returncode == 0:
            self.get_logger().info(f'Map saved: {self.map}.yaml')
        else:
            self.get_logger().error('Saving the map failed')


def main():
    rclpy.init()
    node = SaveOnComplete()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
