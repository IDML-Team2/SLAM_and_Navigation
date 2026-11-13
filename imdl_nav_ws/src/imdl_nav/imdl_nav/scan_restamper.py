"""Replace the time stamp of each laser scan with the Jetson clock.

Runs on the Jetson.

    subscribes:  /scan_raw   (sensor_msgs/msg/LaserScan, from sllidar_node)
    publishes:   /scan       (same data, new time stamp)

Why? SLAM and AMCL ask for every scan: "Where was the robot at the time
header.stamp?" The answer comes from the TF buffer. If the scan stamp does not
match the TF stamps, the scan is dropped. We therefore use the current Jetson
time minus a small delay, so the matching transform is already in the buffer.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRestamper(Node):

    def __init__(self):
        super().__init__('scan_restamper')
        delay_sec = self.declare_parameter('delay_sec', 0.1).value
        self.delay = Duration(seconds=delay_sec)

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(
            LaserScan, '/scan_raw', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg: LaserScan):
        msg.header.stamp = (self.get_clock().now() - self.delay).to_msg()
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ScanRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
