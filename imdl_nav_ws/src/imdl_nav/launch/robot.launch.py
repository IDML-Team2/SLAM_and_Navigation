"""JETSON: everything the robot knows about its own body.

    ros2 launch imdl_nav robot.launch.py

Starts:
    sllidar_node       lidar driver            -> /scan_raw          (frame 'laser')
    scan_restamper     fix time stamps            /scan_raw -> /scan
    odom_to_tf         odometry as TF             <namespace>/odom -> TF odom -> base_link
    lidar_tf           where the lidar is mounted TF base_link -> laser (static)
    camera_tf          where the camera is mounted TF base_link -> camera_link (static,
                       only if robot.yaml has a 'camera' section)

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
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def read_namespace(context, robot_config):
    """Namespace from the launch argument, or else from robot.yaml; always with a leading '/'."""
    namespace = LaunchConfiguration('namespace').perform(context) or robot_config['namespace']
    return '/' + namespace.strip().strip('/')


def lidar_tf(robot_config):
    """Fixed transform base_link -> laser (frame of the sllidar driver)."""
    lidar = robot_config['lidar']
    x = lidar['x_cm'] / 100.0
    y = lidar['y_cm'] / 100.0
    z = lidar['z_cm'] / 100.0
    yaw = math.radians(lidar['yaw_deg'])
    return [
        LogInfo(msg=f'Lidar mounted at x={x:.3f} m, y={y:.3f} m, z={z:.3f} m, '
                    f'yaw={lidar["yaw_deg"]} deg (from robot.yaml)'),
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


def camera_tf(robot_config):
    """Fixed transform base_link -> camera frame, if robot.yaml has a 'camera' section.

    The RealSense driver publishes the frames inside the camera
    (camera_link -> camera_depth_optical_frame, ...) itself.
    """
    camera = robot_config.get('camera')
    if not camera:
        return [LogInfo(msg='No camera section in robot.yaml: no camera TF published')]
    x = camera['x_cm'] / 100.0
    y = camera['y_cm'] / 100.0
    z = camera['z_cm'] / 100.0
    yaw = math.radians(camera['yaw_deg'])
    pitch = math.radians(camera['pitch_deg'])   # ROS: positive pitch = nose down
    frame = camera.get('frame', 'camera_link')
    return [
        LogInfo(msg=f'Camera ({frame}) mounted at x={x:.3f} m, y={y:.3f} m, z={z:.3f} m, '
                    f'yaw={camera["yaw_deg"]} deg, pitch={camera["pitch_deg"]} deg (from robot.yaml)'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf',
            arguments=[
                '--x', str(x), '--y', str(y), '--z', str(z),
                '--yaw', str(yaw), '--pitch', str(pitch), '--roll', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', frame,
            ],
        ),
    ]


def robot_nodes(context):
    with open(LaunchConfiguration('robot_config').perform(context)) as f:
        robot_config = yaml.safe_load(f)
    namespace = read_namespace(context, robot_config)

    return [
        LogInfo(msg=f'Robot namespace: {namespace} (odometry from {namespace}/odom)'),
        Node(
            package='imdl_nav',
            executable='odom_to_tf',
            name='odom_to_tf',
            parameters=[{'odom_topic': namespace + '/odom',
                         'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
        *lidar_tf(robot_config),
        *camera_tf(robot_config),
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
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='Simulator: skip the lidar driver and scan_restamper (the sim publishes /scan).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the simulator clock (/clock).'),

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
            condition=UnlessCondition(LaunchConfiguration('sim')),
        ),

        Node(
            package='imdl_nav',
            executable='scan_restamper',
            name='scan_restamper',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('sim')),
        ),

        OpaqueFunction(function=robot_nodes),
    ])
