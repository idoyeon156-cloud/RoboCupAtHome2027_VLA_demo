"""충돌 제한 시각화 데모. Does not launch any Piper hardware driver."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from piper_collision_guard.urdf_tools import placed_description


def demo_nodes(context):
    share = Path(get_package_share_directory('piper_collision_guard'))
    robot = Path(get_package_share_directory('piper_dual_description'))/'urdf/dual_piper.urdf'
    separation = float(LaunchConfiguration('base_separation_m').perform(context))
    description = placed_description(robot, separation, mesh_root=robot.parents[1])
    common = {'robot_description': description, 'rate': 20}
    return [
        Node(package='piper_collision_guard', executable='collision_degree_sliders',
             name='collision_demo_sliders', parameters=[common],
             remappings=[('joint_states', '/collision_preview/slider_joint_states'),
                         ('robot_description', '/collision_preview/accepted/robot_description')],
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='collision_demo_zero_state', parameters=[common],
             remappings=[('joint_states', '/collision_preview/slider_joint_states'),
                         ('robot_description', '/collision_preview/accepted/robot_description')],
             condition=UnlessCondition(LaunchConfiguration('gui'))),
        Node(package='piper_collision_guard', executable='collision_limit_visual_demo',
             name='collision_limit_visual_demo', output='screen', parameters=[{
                 'urdf_path': str(robot),
                 'robot_description': description,
                 'protection_path': LaunchConfiguration('protection_path'),
                 'stop_clearance_m': ParameterValue(LaunchConfiguration('stop_clearance_m'), value_type=float),
                 'block_unsafe': ParameterValue(LaunchConfiguration('block_unsafe'), value_type=bool)}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='accepted_robot_state_publisher', namespace='collision_preview/accepted',
             parameters=[{'robot_description': description}],
             remappings=[('joint_states', '/collision_preview/accepted_joint_states')]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='ghost_robot_state_publisher', namespace='collision_preview/ghost',
             parameters=[{'robot_description': description, 'frame_prefix': 'ghost/'}],
             remappings=[('joint_states', '/collision_preview/ghost_joint_states')]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='demo_ghost_world',
             arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'world', '--child-frame-id', 'ghost/world']),
        Node(package='rviz2', executable='rviz2', name='collision_demo_rviz',
             arguments=['-d', str(share/'config/collision_limit_visual_demo.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('piper_collision_guard'))
    robot = Path(get_package_share_directory('piper_dual_description'))/'urdf/dual_piper.urdf'
    root = ET.parse(robot).getroot()
    left = float(root.find("joint[@name='world_to_left_base']/origin").get('xyz').split()[1])
    right = float(root.find("joint[@name='world_to_right_base']/origin").get('xyz').split()[1])
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show mouse joint sliders'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('block_unsafe', default_value='true', description='Hold accepted robot at unsafe requests; false inspects every pose'),
        DeclareLaunchArgument('stop_clearance_m', default_value='0.02', description='Surface gap in metres, not joint centre distance'),
        DeclareLaunchArgument('base_separation_m', default_value=str(left-right), description='Base ORIGIN spacing, not shell gap'),
        DeclareLaunchArgument('protection_path', default_value=str(share/'config/protection.yaml')),
        OpaqueFunction(function=demo_nodes),
    ])
