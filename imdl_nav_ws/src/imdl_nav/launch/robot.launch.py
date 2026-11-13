"""JETSON: everything the robot knows about its own body.

    ros2 launch imdl_nav robot.launch.py

Starts:
    sllidar_node       lidar driver            -> /scan_raw          (frame 'laser')
    scan_restamper     fix time stamps            /scan_raw -> /scan
    odom_to_tf         odometry as TF             <namespace>/odom -> TF odom -> base_link
    lidar_tf           where the lidar is mounted TF base_link -> laser (static)

Started on its own for navigation, and included by mapping.launch.py for mapping.

The robot's namespace (e.g. /Team1) and the lidar mounting position (in cm
and degrees) are read from config/robot.yaml.
"""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def read_namespace(context, robot_config):
    """Namespace from the launch argument, or else from robot.yaml; always with a leading '/'."""
    namespace = LaunchConfiguration('namespace').perform(context) or robot_config['namespace']
    return '/' + namespace.strip().strip('/')


def robot_nodes(context):
    with open(LaunchConfiguration('robot_config').perform(context)) as f:
        robot_config = yaml.safe_load(f)
    namespace = read_namespace(context, robot_config)
    lidar = robot_config['lidar']
    x = lidar['x_cm'] / 100.0
    y = lidar['y_cm'] / 100.0
    z = lidar['z_cm'] / 100.0
    yaw = math.radians(lidar['yaw_deg'])

    return [
        LogInfo(msg=f'Robot namespace: {namespace} (odometry from {namespace}/odom)'),
        LogInfo(msg=f'Lidar mounted at x={x:.3f} m, y={y:.3f} m, z={z:.3f} m, '
                    f'yaw={lidar["yaw_deg"]} deg (from robot.yaml)'),
        Node(
            package='imdl_nav',
            executable='odom_to_tf',
            name='odom_to_tf',
            parameters=[{'odom_topic': namespace + '/odom'}],
            output='screen',
        ),
        # Fixed transform base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=[
                '--x', str(x), '--y', str(y), '--z', str(z),
                '--yaw', str(yaw), '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'laser',
            ],
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory('imdl_nav')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace of the Create3 topics, e.g. /Team2. Empty: from robot.yaml.'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0',
            description='USB port of the lidar.'),
        DeclareLaunchArgument(
            'serial_baudrate', default_value='460800',
            description='Lidar baud rate (C1: 460800, A1/A2M8: 115200, A2M12/A3/S1: 256000).'),
        DeclareLaunchArgument(
            'robot_config', default_value=os.path.join(share, 'config', 'robot.yaml'),
            description='Robot geometry file (lidar position, robot size).'),

        # Lidar driver. It actually publishes on "scan"; the remapping
        # redirects that to /scan_raw.
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': ParameterValue(
                    LaunchConfiguration('serial_baudrate'), value_type=int),
                'frame_id': 'laser',
                'angle_compensate': True,
            }],
            remappings=[('scan', '/scan_raw')],
            output='screen',
        ),

        Node(
            package='imdl_nav',
            executable='scan_restamper',
            name='scan_restamper',
            output='screen',
        ),

        OpaqueFunction(function=robot_nodes),
    ])
