"""Build a cereal sequence from held-object dimensions and world positions."""
import argparse
from pathlib import Path
import sys
import tempfile

import hppfcl as fcl
import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'piper_collision_guard'))
from piper_collision_guard.distance import DistanceModel

TCP = np.array([0., 0., .068])
UP = np.array([0., 0., 1.])
BOX_GRIP = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
SIDES = ('left', 'right')
WAYPOINTS = ('ready', 'align', 'tilt_25', 'tilt_45', 'tilt_65')


def vector(value, shape, label):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f'{label} needs finite values of shape {shape}')
    return result


def joint_waypoints(kinematics, bowl_center, bowl_grip, bowl_yaw, box_size,
                    grip_below_top, staging, offsets):
    model = kinematics.model
    data = model.createData()
    neutral = pin.neutral(model)
    rng = np.random.default_rng(332)
    config = yaml.safe_load((ROOT/'piper_collision_guard/config/cereal_pour.yaml').read_text())

    def solve(side, position, rotation, seed):
        ids = kinematics.slots[side]
        lo, hi = model.lowerPositionLimit[ids], model.upperPositionLimit[ids]
        frame_id = model.getFrameId(f'{side}_gripper_base')
        q = neutral.copy()

        def residual(joints):
            q[ids] = joints
            pin.framesForwardKinematics(model, data, q)
            frame = data.oMf[frame_id]
            return np.r_[frame.translation+frame.rotation@TCP-position,
                         .15*pin.log3(rotation.T@frame.rotation)]

        guesses = [seed, *rng.uniform(lo, hi, (16, 6))]
        for guess in guesses:
            result = least_squares(residual, np.clip(guess, lo+1e-8, hi-1e-8),
                                   bounds=(lo, hi), gtol=1e-10, max_nfev=140)
            if np.linalg.norm(result.fun) < 1e-6:
                return result.x
        raise ValueError(f'No in-bounds IK solution for {side} TCP {position.round(3)}')

    yaw = np.deg2rad(bowl_yaw)
    approach = np.array([np.cos(yaw), np.sin(yaw), 0.])
    bowl_orientation = np.column_stack([np.cross(UP, approach), UP, approach])
    left = solve('left', bowl_grip, bowl_orientation, np.deg2rad(config['bowl_arm_deg']))
    lip_offset = np.r_[box_size[:2]/2., grip_below_top]
    right = {}
    seed = np.deg2rad(config['right_waypoints']['ready'])
    outlets = [staging, *[bowl_center+np.r_[0., yz] for yz in offsets]]
    for name, angle, outlet in zip(WAYPOINTS, (0, 0, 25, 45, 65), outlets):
        tilt = Rotation.from_rotvec([-np.deg2rad(angle), 0., 0.]).as_matrix()
        seed = solve('right', outlet-tilt@lip_offset, tilt@BOX_GRIP, seed)
        right[name] = np.rad2deg(seed).round(3).tolist()
    config['bowl_arm_deg'] = np.rad2deg(left).round(3).tolist()
    config['right_waypoints'] = right
    return config, bowl_orientation


def preflight(kinematics, config, bowl_geometry, box_geometry, urdf):
    model = kinematics.model
    data = model.createData()
    geometry = pin.buildGeomFromUrdf(model, str(urdf), pin.GeometryType.COLLISION,
                                    package_dirs=[str(ROOT)])
    geom_data = geometry.createData()
    grouped = {side: [] for side in (*SIDES, 'environment')}
    for index, obj in enumerate(geometry.geometryObjects):
        side = obj.name.split('_', 1)[0]
        if side not in SIDES:
            side = 'environment'
        obj.geometry.buildConvexHull(True, 'Qt')
        grouped[side].append((index, obj.name, obj.geometry.convex))

    bowl_radius, bowl_depth, bowl_offset, bowl_rotation = bowl_geometry
    box_size, box_offset = box_geometry
    bowl = fcl.Cylinder(bowl_radius, bowl_depth)
    box = fcl.Box(*box_size)
    frames = {side: model.getFrameId(f'{side}_gripper_base') for side in SIDES}
    request, result = fcl.CollisionRequest(), fcl.CollisionResult()

    def intersects(a, a_tf, b, b_tf):
        result.clear()
        return fcl.collide(a, a_tf, b, b_tf, request, result) > 0

    def attached(side, reference_rotation, offset):
        frame = data.oMf[frames[side]]
        rotation = frame.rotation@reference_rotation.T
        return fcl.Transform3f(rotation, frame.translation+frame.rotation@TCP+rotation@offset)

    def check_segment(label, first, last, payload):
        limits = np.r_[[np.deg2rad(.5)]*6, .0005]
        count = max(1, int(np.ceil(np.max(np.abs(last-first)/limits))))
        for fraction in np.linspace(0., 1., count+1):
            joints = first+(last-first)*fraction
            q = kinematics.configuration(joints[0, :6], joints[1, :6])
            for side, arm in zip(SIDES, joints):
                for i, sign in ((7, 1.), (8, -1.)):
                    index = model.joints[model.getJointId(f'{side}_joint{i}')].idx_q
                    q[index] = sign*arm[-1]/2.
            if np.any(q < model.lowerPositionLimit-1e-9) or np.any(q > model.upperPositionLimit+1e-9):
                raise ValueError(f'{label}: joint outside URDF limits')
            if kinematics.clearance(joints[0, :6], joints[1, :6]).distance <= stop_clearance:
                raise ValueError(f'{label}: protective-sphere clearance below stop threshold')
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            pin.updateGeometryPlacements(model, data, geometry, geom_data)
            transforms = [fcl.Transform3f(p.rotation, p.translation) for p in geom_data.oMg]
            for i, left, a in grouped['left']:
                for j, right, b in grouped['right']:
                    if intersects(a, transforms[i], b, transforms[j]):
                        raise ValueError(f'{label}: {left} touches {right} ({fraction:.0%})')
            for i, name, a in grouped['environment']:
                for side in SIDES:
                    for j, arm_name, b in grouped[side]:
                        if intersects(a, transforms[i], b, transforms[j]):
                            raise ValueError(f'{label}: {arm_name} touches {name} ({fraction:.0%})')
            if payload:
                bowl_tf = attached('left', bowl_rotation, bowl_offset)
                box_tf = attached('right', BOX_GRIP, box_offset)
                if intersects(bowl, bowl_tf, box, box_tf):
                    raise ValueError(f'{label}: virtual bowl touches virtual box ({fraction:.0%})')
                for i, name, shape in grouped['environment']:
                    if intersects(bowl, bowl_tf, shape, transforms[i]) or intersects(box, box_tf, shape, transforms[i]):
                        raise ValueError(f'{label}: held object touches {name} ({fraction:.0%})')
                for side, obj_name, obj, obj_tf in (
                    ('right', 'virtual bowl', bowl, bowl_tf), ('left', 'virtual box', box, box_tf)
                ):
                    for i, name, shape in grouped[side]:
                        if intersects(obj, obj_tf, shape, transforms[i]):
                            raise ValueError(f'{label}: {obj_name} touches {name} ({fraction:.0%})')

    guard_config = yaml.safe_load((ROOT/'config/robot.yaml').read_text())
    stop_clearance = guard_config['joint_distance']['stop_clearance_m']
    left = np.r_[np.deg2rad(config['bowl_arm_deg']), config['bowl_opening_mm']/1000.]
    poses = [np.stack([left, np.r_[np.deg2rad(config['right_waypoints'][name]),
                                   config['box_opening_mm']/1000.]]) for name in WAYPOINTS]
    neutral = np.zeros((2, 7))
    left_first = np.stack([left, neutral[1]])
    check_segment('neutral -> left ready', neutral, left_first, False)
    check_segment('left ready -> both ready', left_first, poses[0], False)
    for a, b, first, last in zip(WAYPOINTS, WAYPOINTS[1:], poses, poses[1:]):
        check_segment(f'{a} -> {b}', first, last, True)


def plan(objects_file):
    spec = yaml.safe_load(objects_file.read_text())
    bowl, box = spec['bowl'], spec['box']
    center = vector(bowl['center_world_m'], (3,), 'bowl centre')
    size = vector(box['size_m'], (3,), 'box size')
    staging = vector(box['staging_outlet_world_m'], (3,), 'staging outlet')
    offsets = vector(spec['outlet_yz_offsets_m'], (4, 2), 'outlet offsets')
    radius, depth, opening = float(bowl['diameter_m'])/2., float(bowl['depth_m']), float(bowl['rim_grip_opening_mm'])
    grip_below_top = float(box['grip_below_top_m'])
    grip_angle, yaw = float(bowl['grip_azimuth_deg']), float(bowl['gripper_yaw_deg'])
    if (not np.isfinite([radius, depth, opening, grip_below_top, grip_angle, yaw]).all()
            or min(radius, depth, opening, *size) <= 0
            or not 0 < grip_below_top < size[2] or np.any(offsets[:, 1] <= 0)):
        raise ValueError('Dimensions and outlet heights must be positive, with grip below the box top')
    azimuth = np.deg2rad(grip_angle)
    bowl_grip = center + radius*np.array([np.cos(azimuth), np.sin(azimuth), 0.])
    protection = yaml.safe_load((ROOT/'piper_collision_guard/config/protection.yaml').read_text())
    urdf = ROOT/'piper_moveit_guard/config/dual_piper_meshes.urdf'
    joint_distance = yaml.safe_load((ROOT/'config/robot.yaml').read_text())['joint_distance']
    kinematics = DistanceModel(urdf, joint_distance['link_samples'], protection)
    if opening/1000. > kinematics.max_opening['left'] or size[1] > kinematics.max_opening['right']:
        raise ValueError('Requested opening exceeds the gripper URDF limits')
    config, bowl_rotation = joint_waypoints(kinematics, center, bowl_grip, yaw,
                                            size, grip_below_top, staging, offsets)
    config['bowl_opening_mm'] = opening
    config['box_opening_mm'] = float(size[1]*1000.)
    bowl_geometry = (radius, depth, center-bowl_grip-depth/2.*UP, bowl_rotation)
    box_geometry = (size, np.array([size[0]/2., 0., grip_below_top-size[2]/2.]))
    preflight(kinematics, config, bowl_geometry, box_geometry, urdf)
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--objects', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        config = plan(args.objects)
        with tempfile.NamedTemporaryFile('w', dir=args.output.parent, delete=False) as stream:
            temporary_path = Path(stream.name)
            yaml.safe_dump(config, stream, sort_keys=False)
        temporary_path.replace(args.output)
    except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as error:
        parser.exit(1, f'Could not generate cereal sequence: {error}\n')
    print(f'Validated joint sequence written to {args.output}')


if __name__ == '__main__':
    main()
