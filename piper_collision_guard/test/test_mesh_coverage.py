"""Regress the reported pose and prove source mesh box-cell coverage."""
from pathlib import Path
import struct
import itertools
import xml.etree.ElementTree as ET
import numpy as np
import pinocchio as pin
import pytest
import yaml
from scipy.spatial import cKDTree
from piper_collision_guard.distance import DistanceModel, closest
from piper_collision_guard.preview import PreviewState

ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT/'piper_dual_description/urdf/dual_piper.urdf'
CONFIG = yaml.safe_load((ROOT/'piper_collision_guard/config/protection.yaml').read_text())


def test_all_source_triangles_and_finger_travel_are_inside_generated_cell_spheres():
    fk = DistanceModel(URDF, protection=CONFIG)
    robot = ET.parse(URDF).getroot()
    for side in ('left', 'right'):
        assert len(CONFIG[side]['mesh_covers']) == 10
        for spec, (_, centres, radii, _) in zip(CONFIG[side]['mesh_covers'], fk.covers[side]):
            frame = f"{side}_{spec['frame']}"
            uri = robot.find(f"link[@name='{frame}']/collision/geometry/mesh").get('filename')
            raw = (ROOT/'piper_dual_description'/uri.removeprefix('package://piper_dual_description/')).read_bytes()
            count = struct.unpack_from('<I', raw, 80)[0]
            vertices = np.array([struct.unpack_from('<fff', raw, 84+i*50+12+j*12)
                                 for i in range(count) for j in range(3)])
            lo, hi = np.array(spec['bounds_min_m']), np.array(spec['bounds_max_m'])
            assert np.all(vertices >= lo - 1e-12) and np.all(vertices <= hi + 1e-12)
            # Each entire AABB cell is in its sphere, not just its vertices.
            counts = np.ceil((hi-lo)/spec['cell_size_m']).astype(int)
            half_diagonal = np.linalg.norm((hi-lo)/counts)/2
            assert np.all(radii >= half_diagonal + .000999999)
            assert cKDTree(centres).query(vertices)[0].max() <= radii[0]
            joint = next((j for j in robot.findall('joint') if j.find('child').get('link') == frame), None)
            if joint is not None and joint.get('type') == 'prismatic':
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                for limit in ('lower', 'upper'):
                    moved = vertices + float(joint.find('limit').get(limit))*axis
                    assert np.all(moved >= lo-1e-12) and np.all(moved <= hi+1e-12)


def test_reported_left_joint1_fifteen_degrees_is_blocked_and_zero_is_allowed():
    state = PreviewState(URDF, CONFIG)
    assert state.clearance().distance > state.stop
    initial = state.accepted.copy()
    positions = np.zeros(len(state.names))
    positions[state.names.index('left_joint1')] = np.deg2rad(15)
    state.update(state.names, positions)
    assert state.blocked and state.reason == 'distance_limit'
    assert state.clearance().distance < .02
    np.testing.assert_array_equal(state.accepted, initial)
    state.update(state.names, np.zeros(len(state.names)))
    assert not state.blocked


def test_new_cover_never_reports_more_clearance_than_source_mesh_in_example_sweep():
    fk = DistanceModel(URDF, protection=CONFIG)
    geometry = pin.buildGeomFromUrdf(fk.model, str(URDF), pin.GeometryType.COLLISION, package_dirs=[str(ROOT)])
    for i, left in enumerate(geometry.geometryObjects):
        for j, right in enumerate(geometry.geometryObjects):
            if left.name.startswith('left_') and right.name.startswith('right_'):
                geometry.addCollisionPair(pin.CollisionPair(i,j))
    data = geometry.createData()
    for degrees in (0, 4, 6, 10, 15):
        left, right = np.zeros(6), np.zeros(6)
        left[0] = np.deg2rad(degrees)
        pin.updateGeometryPlacements(fk.model, fk.data, geometry, data, fk.configuration(left, right))
        pin.computeDistances(geometry, data)
        mesh_gap = min(d.min_distance for d in data.distanceResults)
        assert fk.clearance(left, right).distance <= mesh_gap + 1e-9
        if degrees == 15:
            assert mesh_gap == pytest.approx(.0013456736, abs=1e-7)


def test_radius_group_search_matches_all_pairs_for_unequal_radii():
    rng = np.random.default_rng(7)
    left, right = rng.normal(size=(130, 3)), rng.normal(size=(110, 3))
    ra, rb = rng.choice([.01,.05,.1],130), rng.choice([.02,.08,.3],110)
    expected = np.min(np.linalg.norm(left[:,None,:]-right[None,:,:],axis=-1)-ra[:,None]-rb[None,:])
    result = closest(left,right,list(map(str,range(130))),list(map(str,range(110))),ra,rb)
    assert result.distance == pytest.approx(expected)
