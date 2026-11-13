"""Republish the Create3 odometry as the TF transform odom -> base_link.

Runs on the Jetson.

    subscribes:  <namespace>/odom, e.g. /Team1/odom  (nav_msgs/msg/Odometry, from the Create3)
                 (the launch file sets the topic from the namespace in robot.yaml)
    publishes:   /tf           odom -> base_link

Why? slam_toolbox, AMCL and Nav2 look up the robot position through TF, not
through the odometry topic. We also stamp the transform with the Jetson clock
so that it matches the time stamps of the laser scans (see scan_restamper.py).

Zero at start: the Create3 counts its odometry from the moment it was switched
on. So we remember the first odometry pose and publish everything relative to
it – odom -> base_link starts at (0, 0, 0) every time this node starts. That
way the origin of a new SLAM map is exactly where the robot stood when mapping
started.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def yaw_of(q):
    """Rotation about the vertical axis (rad) of a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomToTf(Node):

    def __init__(self):
        super().__init__('odom_to_tf')
        odom_topic = self.declare_parameter('odom_topic', '/Team1/odom').value
        self.zero_at_start = self.declare_parameter('zero_at_start', True).value

        self.tf_broadcaster = TransformBroadcaster(self)
        # The Create3 publishes with QoS "best effort". A "reliable" subscriber
        # would receive nothing, hence qos_profile_sensor_data.
        self.create_subscription(
            Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        self.get_logger().info(f'Waiting for odometry on {odom_topic} ...')
        self.start = None        # (x, y, yaw) of the first odometry message

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = yaw_of(msg.pose.pose.orientation)

        if self.start is None:
            self.start = (p.x, p.y, yaw) if self.zero_at_start else (0.0, 0.0, 0.0)
            self.get_logger().info(
                f'Odometry received, publishing odom -> base_link '
                f'(Create3 odometry at start: x={p.x:.2f} m, y={p.y:.2f} m, '
                f'yaw={math.degrees(yaw):.0f} deg; '
                f'{"counted as 0" if self.zero_at_start else "used as is"})')

        # Pose relative to the start pose: shift, then rotate back by the start heading
        x0, y0, yaw0 = self.start
        dx, dy = p.x - x0, p.y - y0
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        rel_yaw = yaw - yaw0

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = c * dx - s * dy
        t.transform.translation.y = s * dx + c * dy
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(rel_yaw / 2.0)
        t.transform.rotation.w = math.cos(rel_yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
