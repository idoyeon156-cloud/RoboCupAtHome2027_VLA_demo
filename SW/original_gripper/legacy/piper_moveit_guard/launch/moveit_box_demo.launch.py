"""MoveIt cross-arm collision inspection with joint sliders; no hardware control."""
from pathlib import Path
import os
import shlex
import sys
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from piper_collision_guard.collision_model import load_model


def demo_nodes(context):
    share = Path(get_package_share_directory('piper_moveit_guard'))
    profile_path = LaunchConfiguration('collision_config').perform(context)
    geometry = LaunchConfiguration('geometry').perform(context)
    description, base, profile_path = load_model(share, geometry, profile_path)
    mesh_mode = geometry == 'meshes'
    if mesh_mode and LaunchConfiguration('editor').perform(context).lower() == 'true':
        raise ValueError('Mesh mode uses pre-generated polyhedra. Box editor requires geometry:=boxes editor:=true')
    common = {'robot_description': description, 'rate': 20}
    python = str(Path(os.environ['VIRTUAL_ENV'])/'bin/python') if os.environ.get('VIRTUAL_ENV') else sys.executable
    remaps = [('joint_states', '/moveit_guard/demo_joint_states'),
              ('robot_description', '/moveit_box_demo/robot_description')]
    return [
        Node(package='piper_collision_guard', executable='collision_degree_sliders',
             prefix=shlex.quote(python),
             name='moveit_box_sliders', parameters=[common], remappings=remaps,
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='moveit_box_zero_pose', parameters=[common], remappings=remaps,
             condition=UnlessCondition(LaunchConfiguration('gui'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='moveit_box_demo', parameters=[{'robot_description': description}],
             remappings=[('joint_states','/moveit_guard/demo_joint_states')]),
        Node(package='piper_moveit_guard', executable='moveit_guard', output='screen',
             parameters=[{'robot_description': description,
                          'robot_description_semantic': (share/'config/dual_piper.srdf').read_text(),
                          'demo': True, 'convex_meshes': mesh_mode}]),
        Node(package='piper_collision_guard', executable='moveit_box_editor',
             prefix=shlex.quote(python), output='screen',
             parameters=[{'base_description': base, 'profile_path': profile_path}],
             condition=IfCondition(LaunchConfiguration('editor'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share/'config/moveit_boxes.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('geometry', default_value='meshes', choices=['meshes', 'boxes']),
        DeclareLaunchArgument('editor', default_value='false'),
        DeclareLaunchArgument('collision_config', default_value=''),
        OpaqueFunction(function=demo_nodes),
    ])
