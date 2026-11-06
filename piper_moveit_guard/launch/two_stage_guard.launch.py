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


def nodes(context):
    share = Path(get_package_share_directory('piper_moveit_guard'))
    first = Path(get_package_share_directory('piper_collision_guard'))
    enforce = LaunchConfiguration('enforce').perform(context).lower() == 'true'
    confirmed = LaunchConfiguration('geometry_confirmed').perform(context).lower() == 'true'
    python = str(Path(os.environ['VIRTUAL_ENV'])/'bin/python') if os.environ.get('VIRTUAL_ENV') else sys.executable
    return [
        Node(package='piper_collision_guard', executable='collision_guard', output='screen',
             prefix=shlex.quote(python),
             parameters=[str(first/'config/guard.yaml'),
                         {'enforce': enforce, 'geometry_confirmed': confirmed}],
             remappings=[('/joint_ctrl_cmd_left', '/pinocchio_checked/left_target'),
                         ('/joint_ctrl_cmd_right', '/pinocchio_checked/right_target')]),
        Node(package='piper_moveit_guard', executable='moveit_guard', output='screen',
             parameters=[{'robot_description': (share/'config/dual_piper_boxes.urdf').read_text(),
                          'robot_description_semantic': (share/'config/dual_piper.srdf').read_text(),
                          'enforce': enforce, 'geometry_confirmed': confirmed}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enforce', default_value='false'),
        DeclareLaunchArgument('geometry_confirmed', default_value='false'),
        OpaqueFunction(function=nodes),
    ])
