"""Replacement gripper mount registration, enclosure and collision regressions."""
from pathlib import Path
import copy
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pinocchio as pin
import hppfcl
import yaml
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from prepare_model import mesh_triangles, rotation, sections
from piper_collision_guard.distance import DistanceModel
URDF = ROOT/'piper_dual_description/urdf/dual_piper.urdf'


def test_mount_registration_and_both_envelopes_enclose_source():
    robot = ET.parse(URDF).getroot()
    cfg = yaml.safe_load((ROOT/'config/robot.yaml').read_text())
    protection = yaml.safe_load((ROOT/'piper_collision_guard/config/protection.yaml').read_text())
    fk = DistanceModel(URDF, 0, protection)
    faces = mesh_triangles(ROOT/'piper_dual_description/meshes/d405_mount_v6_assembly.STL')
    model = ET.parse(ROOT/'piper_moveit_guard/config/dual_piper_meshes.urdf').getroot()
    for side in ('left', 'right'):
        joint = robot.find(f"joint[@name='{side}_d405_mount_fixed']")
        assert joint.find('parent').get('link') == f'{side}_gripper_base'
        pose = joint.find('origin')
        rot = rotation(list(map(float, pose.get('rpy').split())))
        offset = np.fromstring(pose.get('xyz'), sep=' ')
        # CAD closed-pad TCP must coincide with the replacement URDF TCP.
        np.testing.assert_allclose(rot @ np.array([0., .1549, -.0024])+offset, [0, 0, .138], atol=1e-12)
        _, centres, radii, _ = fk.attachment_covers[side][0]
        assert np.max(cKDTree(centres).query(faces.reshape(-1, 3)*.001)[0]) < radii[0]-.004999
        spec = protection[side]['attachment_covers'][0]
        lo, hi = np.array(spec['bounds_min_m']), np.array(spec['bounds_max_m'])
        count = np.ceil((hi-lo)/spec['cell_size_m'])
        assert radii[0] >= np.linalg.norm((hi-lo)/count)/2 + .005 - 1e-12
        from scipy.spatial import ConvexHull
        parts = model.find(f"link[@name='{side}_gripper_flange']").findall('collision')[1:]
        assert len(parts) == 4
        for pts, part in zip(sections(faces, cfg['polyhedra']['sections']['d405_mount_v6_assembly']), parts):
            mesh = ROOT/'piper_moveit_guard/meshes/collision'/Path(part.find('geometry/mesh').get('filename')).name
            hull = ConvexHull(mesh_triangles(mesh).reshape(-1, 3))
            assert (pts*.001 @ hull.equations[:, :3].T+hull.equations[:, 3]).max() <= -.005+1e-7
        q = np.zeros(6); q[5] = .8
        fk.points(q, q)
        frame, local, _, _ = fk.attachment_covers[side][0]
        placement = fk.data.oMf[frame]
        points, names, _ = fk.points(q, q)[side]
        selection = [i for i, n in enumerate(names) if n.startswith(f'{side}_d405_mount_cover_')]
        np.testing.assert_allclose(points[selection], local @ placement.rotation.T+placement.translation)


def test_distance_guard_detects_mount_proximity_missed_by_old_joint_spheres():
    p = yaml.safe_load((ROOT/'piper_collision_guard/config/protection.yaml').read_text())
    p.pop('environment', None)  # Isolate the existing mount regression from the fixed post.
    before = copy.deepcopy(p)
    for side in before.values():
        side.pop('attachment_covers')
    q = np.deg2rad([[13.055644,47.141500,-9.605154,-21.395293,-43.449846,-27.957028],
                   [20.215390,17.328793,-45.556956,-72.800026,-55.050099,-36.809060]])
    assert DistanceModel(URDF, 0, before).clearance(*q).distance > .08
    result = DistanceModel(URDF, 0, p).clearance(*q)
    assert result.distance < .01
    assert 'd405_mount_cover' in result.right_name


def test_source_mount_clears_wrist_and_fingers_over_their_ranges():
    model = pin.buildModelFromUrdf(str(URDF)); data = model.createData()
    geom = pin.buildGeomFromUrdf(model, str(URDF), pin.GeometryType.COLLISION, package_dirs=[str(ROOT)])
    gd = geom.createData(); q = pin.neutral(model)
    mount = next(i for i, o in enumerate(geom.geometryObjects) if o.name.startswith('left_gripper_flange_') and o.meshPath.endswith('d405_mount_v6_assembly.STL'))
    for angle in np.linspace(-2.0944, 2.0944, 49):
        q[model.joints[model.getJointId('left_joint6')].idx_q] = angle
        for opening in (0, .05, .10):
            q[model.joints[model.getJointId('left_joint7')].idx_q] = opening/2
            q[model.joints[model.getJointId('left_joint8')].idx_q] = -opening/2
            pin.updateGeometryPlacements(model, data, geom, gd, q)
            for other in ('link5', 'link6', 'link7', 'link8'):
                i = next(i for i, o in enumerate(geom.geometryObjects) if o.name.startswith('left_'+other+'_'))
                gap = hppfcl.distance(geom.geometryObjects[mount].geometry, gd.oMg[mount],
                                      geom.geometryObjects[i].geometry, gd.oMg[i],
                                      hppfcl.DistanceRequest(), hppfcl.DistanceResult())
                assert gap > .0025
    semantic = ET.parse(ROOT/'piper_moveit_guard/config/dual_piper.srdf').getroot()
    for pair in semantic.findall('disable_collisions'):
        a, b = pair.get('link1'), pair.get('link2')
        assert not (a.startswith('left_') and b.startswith('right_') or a.startswith('right_') and b.startswith('left_'))
        assert 'left_d405_mount' not in (a,b)
    assert robot_mount_is_visual_only()


def robot_mount_is_visual_only():
    robot = ET.parse(URDF).getroot()
    return all(robot.find(f"link[@name='{side}_d405_mount']/visual") is not None
               and robot.find(f"link[@name='{side}_d405_mount']/collision") is None
               and len(robot.find(f"link[@name='{side}_gripper_flange']").findall('collision')) == 2
               for side in ('left','right'))
