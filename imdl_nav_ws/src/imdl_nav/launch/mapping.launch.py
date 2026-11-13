"""JETSON: build a map (SLAM).

    ros2 launch imdl_nav mapping.launch.py

Starts robot.launch.py (lidar, time stamps, odometry TF, lidar TF from
config/robot.yaml) plus slam_toolbox:
    subscribes:  /scan, /tf
    publishes:   /map (every 5 s), TF map -> odom, /pose

The map only lives in memory until it is saved on the PC with map_saver_cli.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('imdl_nav')

    return LaunchDescription([
        # All arguments (e.g. serial_baudrate:=115200) are passed through.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'robot.launch.py')),
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[os.path.join(share, 'config', 'slam_toolbox.yaml')],
            output='screen',
        ),
    ])
