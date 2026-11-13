"""PC: RViz with a ready-made view for mapping and navigation.

    ros2 launch imdl_nav rviz.launch.py

Starts:
    robot_state_publisher   publishes a small 3D model of the robot on /robot_description
    rviz2                   shows /map, /scan, /tf, costmaps, /plan, AMCL particles, robot model

The robot model is built from config/robot.yaml: the original Create3 body and
bumper (package irobot_create_description; a plain cylinder if it is not
installed) plus the lidar at its mounting position. A red "nose" on the lidar
points to its 0-degree direction. The model adds two extra TF frames,
'lidar_model' and 'lidar_model_nose' (only for the picture; the real lidar
frame 'laser' comes from the Jetson). RViz computes nothing, it only displays topics.
"""

import math
import os

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CREATE3_BODY_Z = 0.0392      # height of the body mesh above base_link (= floor), from irobot_create_description


def visual(geometry, x=0.0, y=0.0, z=0.0, yaw=0.0, material=None):
    color = f'<material name="{material}"/>' if material else ''
    return (f'<visual><origin xyz="{x} {y} {z}" rpy="0 0 {yaw}"/>'
            f'<geometry>{geometry}</geometry>{color}</visual>')


def robot_description(robot_config):
    """URDF: base_link carries the body; the lidar is drawn by two small extra links.

    (RViz uses only one color per link – the one of its first visual – and
    red if that has none. So the lidar and its nose get their own links.)
    """
    lidar = robot_config['lidar']
    lx, ly, lz = lidar['x_cm'] / 100, lidar['y_cm'] / 100, lidar['z_cm'] / 100
    lyaw = math.radians(lidar['yaw_deg'])

    try:
        get_package_share_directory('irobot_create_description')
        mesh = 'package://irobot_create_description/meshes/'
        body = (visual(f'<mesh filename="{mesh}body_visual.dae"/>', z=CREATE3_BODY_Z, yaw=math.pi / 2)
                + visual(f'<mesh filename="{mesh}bumper_visual.dae"/>', z=CREATE3_BODY_Z))
        source = 'Create3 model from irobot_create_description'
    except PackageNotFoundError:
        radius = robot_config['robot']['radius_cm'] / 100
        body = visual(f'<cylinder radius="{radius}" length="0.09"/>', z=0.045, material='body_gray')
        source = 'simple cylinder (irobot_create_description not installed)'

    # lidar: black puck + red nose pointing to its 0-degree direction
    puck = visual('<cylinder radius="0.035" length="0.04"/>', material='lidar_black')
    nose = visual('<box size="0.06 0.012 0.012"/>', material='nose_red')

    urdf = f"""<?xml version="1.0"?>
<robot name="imdl_robot">
  <material name="lidar_black"><color rgba="0.1 0.1 0.1 1"/></material>
  <material name="nose_red"><color rgba="0.9 0.1 0.1 1"/></material>
  <material name="body_gray"><color rgba="0.6 0.6 0.6 1"/></material>
  <link name="base_link">{body}</link>
  <link name="lidar_model">{puck}</link>
  <link name="lidar_model_nose">{nose}</link>
  <joint name="lidar_model_joint" type="fixed">
    <parent link="base_link"/><child link="lidar_model"/>
    <origin xyz="{lx} {ly} {lz}" rpy="0 0 {lyaw}"/>
  </joint>
  <joint name="lidar_model_nose_joint" type="fixed">
    <parent link="lidar_model"/><child link="lidar_model_nose"/>
    <origin xyz="0.05 0 0" rpy="0 0 0"/>
  </joint>
</robot>"""
    return urdf, source


def nodes(context):
    share = get_package_share_directory('imdl_nav')
    with open(LaunchConfiguration('robot_config').perform(context)) as f:
        robot_config = yaml.safe_load(f)
    urdf, source = robot_description(robot_config)
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'}
    return [
        LogInfo(msg=f'Robot model: {source}, lidar position from robot.yaml'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_model_publisher', parameters=[{'robot_description': urdf}, sim_time],
             output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'imdl_nav.rviz')],
             parameters=[sim_time], output='screen'),
    ]


def generate_launch_description():
    share = get_package_share_directory('imdl_nav')
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config', default_value=os.path.join(share, 'config', 'robot.yaml'),
            description='Robot file (lidar position for the robot model).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use the simulator clock (/clock).'),
        OpaqueFunction(function=nodes),
    ])
