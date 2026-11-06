"""MoveIt cross-arm collision inspection with joint sliders; no hardware control."""
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('piper_moveit_guard'))
    root = ET.parse(share/'config/dual_piper_boxes.urdf').getroot()
    mesh_root = Path(get_package_share_directory('piper_dual_description'))
    prefix = 'package://piper_dual_description/'
    for mesh in root.iter('mesh'):
        uri = mesh.get('filename', '')
        if uri.startswith(prefix):
            mesh.set('filename', (mesh_root/uri[len(prefix):]).resolve().as_uri())
    description = ET.tostring(root, encoding='unicode')
    common = {'robot_description': description, 'rate': 20}
    remaps = [('joint_states', '/moveit_guard/demo_joint_states'),
              ('robot_description', '/moveit_box_demo/robot_description')]
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
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
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share/'config/moveit_boxes.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
