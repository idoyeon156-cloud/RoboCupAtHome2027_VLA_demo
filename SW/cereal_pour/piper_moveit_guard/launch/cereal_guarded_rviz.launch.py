"""MoveIt-planned, two-stage guarded, feedback-driven RViz; no hardware driver."""
import os
from pathlib import Path
import shlex
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from piper_collision_guard.guarded_launch import TOPICS, build_pipeline
from piper_collision_guard.plan_snapshot import freeze_sequence


def nodes(context):
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('Use run_cereal_rviz.sh: isolated localhost simulation is required')
    prefix = '/cereal_sim'
    sequence = LaunchConfiguration('sequence_path').perform(context)
    speed = int(LaunchConfiguration('driver_speed_ratio').perform(context))
    scale = float(LaunchConfiguration('joint_speed_scale').perform(context))
    processes, description, _, fingerprint = build_pipeline(
        enforce=True, confirmed=True, driver_speed=speed, joint_speed_scale=scale,
        cereal=False, prefix=prefix, visualize=True)
    frozen, receipt = freeze_sequence(sequence, fingerprint, speed, scale)
    # Use the immutable copy in both executor and object display.
    python = str(Path(os.environ['VIRTUAL_ENV'])/'bin/python') if os.environ.get('VIRTUAL_ENV') else sys.executable
    mapping = [(name, prefix+name) for name in TOPICS]
    share = Path(get_package_share_directory('piper_moveit_guard'))
    return processes+[
        Node(package='piper_collision_guard', executable='cereal_pour', output='screen',
             prefix=shlex.quote(python), remappings=mapping,
             parameters=[{'enforce': True, 'sequence_path': str(frozen), 'joint_speed_scale': scale,
                          'arms_only': LaunchConfiguration('arms_only').perform(context).lower() == 'true'}]),
        Node(package='piper_collision_guard', executable='cereal_simulated_driver', output='screen',
             prefix=shlex.quote(python), remappings=mapping,
             parameters=[{'snapshot_receipt': str(receipt)}]),
        Node(package='piper_collision_guard', executable='cereal_preview', output='screen',
             prefix=shlex.quote(python),
             parameters=[{'feedback_driven': True, 'sequence_path': str(frozen)}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='moveit_box_demo', parameters=[{'robot_description': description}],
             remappings=[('joint_states', prefix+'/joint_states')]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share/'config/cereal_guarded.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sequence_path'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('driver_speed_ratio', default_value='20'),
        DeclareLaunchArgument('joint_speed_scale', default_value='1.5'),
        DeclareLaunchArgument('arms_only', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=nodes),
    ])
