"""Rear-flush 3030 post: geometry, distance and preview regressions."""
from pathlib import Path
import copy
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml
from scipy.spatial import ConvexHull, cKDTree

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from prepare_model import mesh_triangles
from piper_collision_guard.distance import DistanceModel
from piper_collision_guard.preview import PreviewState
URDF=ROOT/'piper_dual_description/urdf/dual_piper.urdf'
PROFILE_POSE=np.deg2rad([[70.37398553,66.28560393,-116.11908577,24.52879148,-15.16341032,-55.90519164], [0,0,0,0,0,0]])


def test_post_rear_plane_height_and_collision_envelopes():
    source=ET.parse(URDF).getroot()
    joint=source.find("joint[@name='rear_profile_3030_fixed']")
    assert joint.get('type')=='fixed' and joint.find('parent').get('link')=='world'
    xyz=np.fromstring(joint.find('origin').get('xyz'),sep=' ')
    size=np.fromstring(source.find("link[@name='rear_profile_3030']/collision/geometry/box").get('size'),sep=' ')
    np.testing.assert_allclose(size,[.03,.03,.5])
    base=mesh_triangles(ROOT/'piper_dual_description/meshes/base_link.STL').reshape(-1,3)
    assert xyz[0]-size[0]/2 == pytest.approx(base[:,0].min(),abs=1e-8)
    assert xyz[1]==0 and xyz[2]-size[2]/2==0 and xyz[2]+size[2]/2==.5
    fitted=ET.parse(ROOT/'piper_moveit_guard/config/dual_piper_meshes.urdf').getroot()
    mesh=fitted.find("link[@name='rear_profile_3030']/collision/geometry/mesh")
    pts=mesh_triangles(ROOT/'piper_moveit_guard/meshes/collision'/Path(mesh.get('filename')).name).reshape(-1,3)
    hull=ConvexHull(pts)
    from itertools import product
    corners=np.asarray(list(product((-1,1),repeat=3)))*size/2
    assert (corners@hull.equations[:,:3].T+hull.equations[:,3]).max()<=-.005+1e-7
    p=yaml.safe_load((ROOT/'piper_collision_guard/config/protection.yaml').read_text())
    model=DistanceModel(URDF,0,p)
    frame,centres,radii,_=model.environment_covers[0]
    assert model.model.frames[frame].name=='rear_profile_3030'
    assert cKDTree(centres).query(corners)[0].max() <= radii[0]-.004999


def test_both_arms_check_post_and_preview_holds_unsafe_pose():
    p=yaml.safe_load((ROOT/'piper_collision_guard/config/protection.yaml').read_text())
    model=DistanceModel(URDF,0,p)
    without=copy.deepcopy(p);without.pop('environment')
    assert DistanceModel(URDF,0,without).clearance(*PROFILE_POSE).distance>.08
    result=model.clearance(*PROFILE_POSE)
    assert result.distance<0 and result.right_name.startswith('rear_profile_3030')
    # Reflect across world Y: left/right base and arm placements are symmetric.
    mirrored=PROFILE_POSE[0].copy();mirrored[[0,3,5]]*=-1
    result=model.clearance(np.zeros(6),mirrored)
    assert result.distance<0 and result.right_name.startswith('rear_profile_3030')
    before=model.points(np.zeros(6),np.zeros(6))['environment'][0]
    after=model.points(*PROFILE_POSE)['environment'][0]
    np.testing.assert_allclose(before,after)
    preview=PreviewState(URDF,p,stop_clearance=.01)
    initial=preview.accepted.copy()
    values=dict.fromkeys(preview.names,0.)
    for j,q in enumerate(PROFILE_POSE[0],1):values[f'left_joint{j}']=q
    preview.update(list(values),list(values.values()))
    assert preview.blocked and preview.clearance().right_name.startswith('rear_profile_3030')
    np.testing.assert_array_equal(preview.accepted,initial)
