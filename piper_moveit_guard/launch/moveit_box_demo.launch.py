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
from piper_collision_guard.box_tuning import tuned_description


def demo_nodes(context):
    share = Path(get_package_share_directory('piper_moveit_guard'))
    profile_path = LaunchConfiguration('collision_config').perform(context)
    base = (share/'config/dual_piper_boxes.urdf').read_text()
    root = ET.fromstring(tuned_description(share/'config/dual_piper_boxes.urdf', profile_path))
    mesh_root = Path(get_package_share_directory('piper_dual_description'))
    prefix = 'package://piper_dual_description/'
    for mesh in root.iter('mesh'):
        uri = mesh.get('filename', '')
        if uri.startswith(prefix):
            mesh.set('filename', (mesh_root/uri[len(prefix):]).resolve().as_uri())
    description = ET.tostring(root, encoding='unicode')
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
                          'demo': True}]),
        Node(package='piper_collision_guard', executable='moveit_box_editor',
             prefix=shlex.quote(python), output='screen',
             parameters=[{'base_description': base, 'profile_path': profile_path}],
             condition=IfCondition(LaunchConfiguration('editor'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share/'config/moveit_boxes.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    saved = Path.cwd()/'collision_tuning.json'
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('editor', default_value=LaunchConfiguration('gui')),
        DeclareLaunchArgument('collision_config', default_value=str(saved) if saved.is_file() else ''),
        OpaqueFunction(function=demo_nodes),
    ])
