"""Send the robot to a coordinate on the map.

    ros2 run imdl_nav go_to X Y [YAW_IN_DEGREES]

    Example:  ros2 run imdl_nav go_to 1.0 0.5 90

X and Y in meters in the 'map' frame ((0, 0) = start pose of the mapping run),
YAW in degrees, counter-clockwise (0 = heading at the start of the mapping run).
Ctrl-C cancels the drive.
"""

import sys

import rclpy
from rclpy.signals import SignalHandlerOptions

from imdl_nav.nav_client import NavClient


def main():
    args = sys.argv[1:]
    if len(args) not in (2, 3):
        print(__doc__)
        sys.exit(1)
    x, y = float(args[0]), float(args[1])
    yaw_deg = float(args[2]) if len(args) == 3 else 0.0

    # Without rclpy's SIGINT handler ROS stays up after Ctrl-C, so we can
    # still cancel the running goal in Nav2.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = NavClient()
    success = False
    try:
        success = node.drive_to(x, y, yaw_deg)
    except KeyboardInterrupt:
        node.cancel()
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if success else 1)
