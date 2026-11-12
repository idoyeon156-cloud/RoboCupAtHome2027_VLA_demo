"""Pinocchio -> MoveIt box gate -> Piper; monitor mode by default."""
from pathlib import Path
import os
import shlex
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from piper_collision_guard.collision_model import load_model


def nodes(context):
    share = Path(get_package_share_directory('piper_moveit_guard'))
    first = Path(get_package_share_directory('piper_collision_guard'))
    enforce = LaunchConfiguration('enforce').perform(context).lower() == 'true'
    confirmed = LaunchConfiguration('geometry_confirmed').perform(context).lower() == 'true'
    python = str(Path(os.environ['VIRTUAL_ENV'])/'bin/python') if os.environ.get('VIRTUAL_ENV') else sys.executable
    geometry = LaunchConfiguration('geometry').perform(context)
    description, _, _ = load_model(share, geometry, LaunchConfiguration('collision_config').perform(context))
    processes = [
        Node(package='piper_collision_guard', executable='collision_guard', output='screen',
             prefix=shlex.quote(python),
             parameters=[str(first/'config/guard.yaml'),
                         {'enforce': enforce, 'geometry_confirmed': confirmed}],
             remappings=[('/joint_ctrl_cmd_left', '/pinocchio_checked/left_target'),
                         ('/joint_ctrl_cmd_right', '/pinocchio_checked/right_target')]),
        Node(package='piper_moveit_guard', executable='moveit_guard', output='screen',
             parameters=[{'robot_description': description, 'convex_meshes': geometry == 'meshes',
                          'robot_description_semantic': (share/'config/dual_piper.srdf').read_text(),
                          'enforce': enforce, 'geometry_confirmed': confirmed}]),
    ]
    if LaunchConfiguration('cereal').perform(context).lower() == 'true':
        sequence_path = LaunchConfiguration('sequence_path').perform(context)
        parameters = {'enforce': enforce}
        if sequence_path:
            parameters['sequence_path'] = sequence_path
        processes.append(Node(package='piper_collision_guard', executable='cereal_pour',
                              output='screen', prefix=shlex.quote(python), parameters=[parameters]))
    return processes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enforce', default_value='false'),
        DeclareLaunchArgument('geometry_confirmed', default_value='false'),
        DeclareLaunchArgument('geometry', default_value='meshes', choices=['meshes', 'boxes']),
        DeclareLaunchArgument('collision_config', default_value=''),
        DeclareLaunchArgument('cereal', default_value='false'),
        DeclareLaunchArgument('sequence_path', default_value=''),
        OpaqueFunction(function=nodes),
    ])
