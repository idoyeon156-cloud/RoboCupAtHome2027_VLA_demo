from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory("piper_collision_guard")) / "config/guard.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("enforce", default_value="false"),
        DeclareLaunchArgument("geometry_confirmed", default_value="false"),
        DeclareLaunchArgument("config", default_value=config),
        Node(package="piper_collision_guard", executable="collision_guard", output="screen",
             parameters=[LaunchConfiguration("config"),
                         {"enforce": ParameterValue(LaunchConfiguration("enforce"), value_type=bool),
                          "geometry_confirmed": ParameterValue(LaunchConfiguration("geometry_confirmed"), value_type=bool)}]),
    ])
