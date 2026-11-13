"""PC: load the map, localize (AMCL) and navigate (Nav2).

    ros2 launch imdl_nav navigation.launch.py map:=$HOME/maps/my_map.yaml

Requires robot.launch.py running on the Jetson (NOT mapping.launch.py).

Starts:
    map_server          map from disk                    -> /map
    amcl                localization                     -> TF map -> odom, /amcl_pose
    planner_server      plan a path                      -> /plan
    controller_server   follow the path                  -> cmd_vel_nav
    behavior_server     recovery: spin, wait             -> cmd_vel_nav
    velocity_smoother   limit acceleration               cmd_vel_nav -> <namespace>/cmd_vel
    bt_navigator        sequencing (behavior_trees/navigate.xml), action /navigate_to_pose
    lifecycle_manager   brings all of the above up in order

Parameters come from config/nav2.yaml. The robot's namespace (e.g. /Team1),
size, safety margin, inflation, cost scaling, path keep-away and minimum lidar range come from
config/robot.yaml (in cm) and are written into a copy of nav2.yaml at start-up.
"""

import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nav2_params_with_robot_geometry(nav2_file, robot_file):
    """Copy nav2.yaml and fill in the values from robot.yaml (cm -> m)."""
    with open(nav2_file) as f:
        params = yaml.safe_load(f)
    with open(robot_file) as f:
        robot_config = yaml.safe_load(f)
    robot = robot_config['robot']
    margin = robot['safety_margin_cm'] / 100.0
    inflation = robot_config['navigation']['inflation_cm'] / 100.0
    cost_scaling = float(robot_config['navigation']['cost_scaling_factor'])
    keep_away = float(robot_config['navigation'].get('path_keep_away', 1.0))
    min_range = robot_config['lidar']['ignore_closer_than_cm'] / 100.0

    footprint = robot.get('footprint_cm') or []
    if footprint:
        points = [[x / 100.0, y / 100.0] for x, y in footprint]
        outer_radius = max(math.hypot(x, y) for x, y in points) + margin
        size = f'footprint {points} + {margin:.2f} m margin'
    else:
        outer_radius = robot['radius_cm'] / 100.0 + margin
        size = f'radius {outer_radius:.3f} m (incl. margin)'
    if inflation <= outer_radius:
        raise RuntimeError(
            f'robot.yaml: inflation_cm ({inflation * 100:.0f}) must be larger than the '
            f'robot size incl. safety margin ({outer_radius * 100:.0f} cm).')

    for name in ('global_costmap', 'local_costmap'):
        costmap = params[name][name]['ros__parameters']
        if footprint:
            costmap['footprint'] = str(points)
            costmap['footprint_padding'] = margin
        else:
            costmap['robot_radius'] = outer_radius
            costmap['footprint_padding'] = 0.0
        costmap['inflation_layer']['inflation_radius'] = inflation
        costmap['inflation_layer']['cost_scaling_factor'] = cost_scaling
        costmap['obstacle_layer']['scan']['obstacle_min_range'] = min_range
        costmap['obstacle_layer']['scan']['raytrace_min_range'] = min_range
    params['amcl']['ros__parameters']['laser_min_range'] = min_range
    params['planner_server']['ros__parameters']['GridBased']['cost_travel_multiplier'] = keep_away

    out = tempfile.NamedTemporaryFile('w', prefix='imdl_nav2_', suffix='.yaml', delete=False)
    yaml.safe_dump(params, out)
    out.close()
    summary = (f'Robot: {size}, inflation {inflation:.2f} m, cost scaling {cost_scaling:g}, '
               f'path keep-away {keep_away:g}, '
               f'lidar readings closer than {min_range:.2f} m ignored (from robot.yaml)')
    return out.name, summary


def nodes(context):
    share = get_package_share_directory('imdl_nav')
    robot_file = LaunchConfiguration('robot_config').perform(context)
    params, summary = nav2_params_with_robot_geometry(
        os.path.join(share, 'config', 'nav2.yaml'), robot_file)
    with open(robot_file) as f:
        namespace = LaunchConfiguration('namespace').perform(context) or yaml.safe_load(f)['namespace']
    namespace = '/' + namespace.strip().strip('/')

    # Nav2 uses the names 'odom' and 'cmd_vel' internally. The Create3 uses
    # namespaced topics. Velocity commands go through the velocity smoother:
    #   controller / behaviors -> cmd_vel_nav -> velocity_smoother -> <namespace>/cmd_vel
    odom = [('odom', namespace + '/odom')]
    to_smoother = odom + [('cmd_vel', 'cmd_vel_nav')]
    smoother = odom + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', namespace + '/cmd_vel')]
    behavior_tree = os.path.join(share, 'behavior_trees', 'navigate.xml')

    initial_pose = {
        'initial_pose.x': float(LaunchConfiguration('initial_x').perform(context)),
        'initial_pose.y': float(LaunchConfiguration('initial_y').perform(context)),
        'initial_pose.z': 0.0,
        'initial_pose.yaw': math.radians(
            float(LaunchConfiguration('initial_yaw').perform(context))),
    }

    lifecycle_nodes = [
        'map_server', 'amcl', 'planner_server', 'controller_server',
        'behavior_server', 'velocity_smoother', 'bt_navigator',
    ]

    return [
        LogInfo(msg=f'Robot namespace: {namespace} (odometry {namespace}/odom, '
                    f'commands {namespace}/cmd_vel)'),
        LogInfo(msg=summary),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[params, {'yaml_filename': LaunchConfiguration('map')}],
             output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[params, initial_pose],
             output='screen'),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[params],
             output='screen'),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[params], remappings=to_smoother,
             output='screen'),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=[params], remappings=to_smoother,
             output='screen'),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             parameters=[params], remappings=smoother,
             output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[params, {'default_nav_to_pose_bt_xml': behavior_tree}], remappings=odom,
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'node_names': lifecycle_nodes}],
             output='screen'),
    ]


def generate_launch_description():
    share = get_package_share_directory('imdl_nav')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.expanduser('~/maps/my_map.yaml'),
            description='Saved map (.yaml).'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace of the Create3 topics, e.g. /Team2. Empty: from robot.yaml.'),
        DeclareLaunchArgument(
            'robot_config', default_value=os.path.join(share, 'config', 'robot.yaml'),
            description='Robot geometry file (lidar position, robot size).'),
        DeclareLaunchArgument(
            'initial_x', default_value='0.0', description='Start position x (m).'),
        DeclareLaunchArgument(
            'initial_y', default_value='0.0', description='Start position y (m).'),
        DeclareLaunchArgument(
            'initial_yaw', default_value='0.0', description='Start heading (degrees).'),
        OpaqueFunction(function=nodes),
    ])
