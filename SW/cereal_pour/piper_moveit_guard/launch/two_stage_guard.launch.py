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
from piper_collision_guard.guarded_launch import build_pipeline


def nodes(context):
    enforce = LaunchConfiguration('enforce').perform(context).lower() == 'true'
    confirmed = LaunchConfiguration('geometry_confirmed').perform(context).lower() == 'true'
    driver_speed = int(LaunchConfiguration('driver_speed_ratio').perform(context))
    joint_speed_scale = float(LaunchConfiguration('joint_speed_scale').perform(context))
    geometry = LaunchConfiguration('geometry').perform(context)
    processes, _, _, _ = build_pipeline(
        enforce=enforce, confirmed=confirmed, geometry=geometry,
        driver_speed=driver_speed, joint_speed_scale=joint_speed_scale,
        collision_config=LaunchConfiguration('collision_config').perform(context),
        cereal=LaunchConfiguration('cereal').perform(context).lower() == 'true',
        sequence_path=LaunchConfiguration('sequence_path').perform(context),
        arms_only=LaunchConfiguration('arms_only').perform(context).lower() == 'true')
    return processes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enforce', default_value='false'),
        DeclareLaunchArgument('geometry_confirmed', default_value='false'),
        DeclareLaunchArgument('geometry', default_value='meshes', choices=['meshes', 'boxes']),
        DeclareLaunchArgument('collision_config', default_value=''),
        DeclareLaunchArgument('cereal', default_value='false'),
        DeclareLaunchArgument('driver_speed_ratio', default_value='10'),
        DeclareLaunchArgument('joint_speed_scale', default_value='1.0'),
        DeclareLaunchArgument('sequence_path', default_value=''),
        DeclareLaunchArgument('arms_only', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=nodes),
    ])
