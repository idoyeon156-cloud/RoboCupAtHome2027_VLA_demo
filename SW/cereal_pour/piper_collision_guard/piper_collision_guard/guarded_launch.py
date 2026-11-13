"""One command pipeline for hardware and isolated feedback-driven RViz."""
import hashlib
import os
from pathlib import Path
import shlex
import sys
import yaml

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from .collision_model import load_model, load_semantic


TOPICS = (
    '/joint_states_left', '/joint_states_right', '/arm_status_left', '/arm_status_right',
    '/joint_states_ctrl_left', '/joint_states_ctrl_right',
    '/teleop/left_target', '/teleop/right_target',
    '/pinocchio_checked/left_target', '/pinocchio_checked/right_target',
    '/joint_ctrl_cmd_left', '/joint_ctrl_cmd_right',
    '/collision_guard/status', '/collision_guard/stop_requested',
    '/collision_guard/reset', '/collision_guard/markers',
    '/moveit_guard/status', '/moveit_guard/stop_requested', '/moveit_guard/reset',
    '/moveit_guard/marker', '/moveit_guard/boxes',
    '/moveit_guard/accepted_joint_states', '/moveit_guard/check_state_validity',
    '/cereal_pour/status', '/cereal_pour/run', '/cereal_pour/prepare',
    '/cereal_pour/start', '/cereal_pour/home', '/cereal_pour/stop',
)


def build_pipeline(*, enforce, confirmed, driver_speed, joint_speed_scale,
                   sequence_path='', cereal=False, geometry='meshes',
                   collision_config='', prefix='', visualize=False, arms_only=False):
    share = Path(get_package_share_directory('piper_moveit_guard'))
    first = Path(get_package_share_directory('piper_collision_guard'))
    python = str(Path(os.environ['VIRTUAL_ENV'])/'bin/python') if os.environ.get('VIRTUAL_ENV') else sys.executable
    description, _, _ = load_model(share, geometry, collision_config)
    semantic = load_semantic(share, description)
    fingerprint = hashlib.sha256((description+'\n'+semantic).encode()).hexdigest()
    if cereal and not prefix:
        from .plan_snapshot import completed_sequence, package_root
        if (package_root()/'REAL_MOTION_HOLD_KO.md').exists():
            raise ValueError('Physical automatic motion is on hold: REAL_MOTION_HOLD_KO.md')
        frozen = completed_sequence(fingerprint, driver_speed, joint_speed_scale)
        if sequence_path and Path(sequence_path).resolve() != frozen.resolve():
            raise ValueError('Real execution must use the exact completed simulation snapshot')
        sequence_path = str(frozen)
    if cereal:
        sequence = yaml.safe_load(Path(sequence_path).read_text())
        if bool(sequence.get('settings', {}).get('execution', {}).get('arms_only', False)) != arms_only:
            raise ValueError('Sequence mode differs: complete simulation with the same arms_only setting first')
    mapping = {name: prefix+name for name in TOPICS} if prefix else {}
    first_mapping = dict(mapping)
    for side in ('left', 'right'):
        first_mapping['/joint_ctrl_cmd_'+side] = prefix+'/pinocchio_checked/'+side+'_target'
    nodes = [
        Node(package='piper_collision_guard', executable='collision_guard', output='screen',
             prefix=shlex.quote(python),
             parameters=[str(first/'config/guard.yaml'),
                         {'enforce': enforce, 'geometry_confirmed': confirmed,
                          'driver_speed_ratio': int(driver_speed)}],
             remappings=list(first_mapping.items())),
        Node(package='piper_moveit_guard', executable='moveit_guard', output='screen',
             parameters=[{'robot_description': description, 'robot_description_semantic': semantic,
                          'model_fingerprint': fingerprint, 'convex_meshes': geometry == 'meshes',
                          'visualize_collision_geometry': visualize,
                          'enforce': enforce, 'geometry_confirmed': confirmed,
                          'driver_speed_ratio': int(driver_speed)}],
             remappings=list(mapping.items())),
    ]
    if cereal:
        params = {'enforce': enforce, 'joint_speed_scale': float(joint_speed_scale), 'arms_only': arms_only}
        if sequence_path:
            params['sequence_path'] = str(sequence_path)
        nodes.append(Node(package='piper_collision_guard', executable='cereal_pour',
                          output='screen', prefix=shlex.quote(python),
                          parameters=[params], remappings=list(mapping.items())))
    return nodes, description, semantic, fingerprint
