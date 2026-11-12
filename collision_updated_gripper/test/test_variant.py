"""Regression tests for configuration changes, not new robot control logic."""
from pathlib import Path
import copy
import hashlib
import importlib.util
import itertools
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
sys.path.insert(0,str(ROOT/'piper_collision_guard'))
from prepare_model import prepare, rotation, mesh_triangles
from piper_collision_guard.distance import DistanceModel, arm_state


@pytest.fixture(scope='module')
def variant(tmp_path_factory):
    root=tmp_path_factory.mktemp('variant')
    for folder in ('piper_moveit_guard','piper_dual_description','piper_collision_guard','model'):
        (root/folder).mkdir()
    cfg=yaml.safe_load((ROOT/'config/robot.yaml').read_text())
    cfg['source_urdf']=str(ROOT/'model/dual_piper.source.urdf')
    cfg['mounts']['left']={'xyz':[.1,.30,.15],'rpy':[.1,.2,.3]}
    cfg['mounts']['right']={'xyz':[-.1,-.20,.40],'rpy':[-.1,.1,-.2]}
    cfg['grippers']['left']['max_opening_m']=.10
    cfg['grippers']['right']['max_opening_m']=.06
    cfg['attachments']=[dict(name='head_camera',parent='world',xyz=[0.,0.,.9],rpy=[.2,.3,.4],
                             geometry={'box':[.04,.05,.06]},touch_links=[]),
                        dict(name='left_sensor',parent='left_link6',xyz=[0.,.05,0.],rpy=[0.,0.,.4],
                             geometry={'cylinder':[.01,.04]},touch_links=['left_link6'])]
    config=root/'robot.yaml';config.write_text(yaml.safe_dump(cfg))
    manifest=prepare(config,root)
    return root,cfg,manifest


def test_placement_finger_limits_and_point_model_agree(variant):
    root,cfg,_=variant
    source=ET.parse(root/'piper_dual_description/urdf/dual_piper.urdf').getroot()
    convex=ET.parse(root/'piper_moveit_guard/config/dual_piper_meshes.urdf').getroot()
    for side in ('left','right'):
        name='world_to_'+side+'_base'
        assert ET.tostring(source.find(f"joint[@name='{name}']")) == ET.tostring(convex.find(f"joint[@name='{name}']"))
        maximum=cfg['grippers'][side]['max_opening_m']
        assert float(convex.find(f"joint[@name='{side}_joint7']/limit").get('upper'))==pytest.approx(maximum/2)
        assert float(convex.find(f"joint[@name='{side}_joint8']/limit").get('lower'))==pytest.approx(-maximum/2)
    fk=DistanceModel(root/'piper_dual_description/urdf/dual_piper.urdf',0,cfg['joint_distance']['protection'])
    assert fk.max_opening==pytest.approx({'left':.10,'right':.06})
    # The first sample is link1's joint origin, transformed by the configured mount.
    points=fk.points(np.zeros(6),np.zeros(6))
    for side in ('left','right'):
        m=cfg['mounts'][side]
        expected=rotation(m['rpy'])@np.array([0,0,.123])+m['xyz']
        np.testing.assert_allclose(points[side][0][0],expected,atol=1e-12)
    names=[f'joint{i}' for i in range(1,7)]+['gripper']
    arm_state(names,[0]*6+[.09],'left',fk.max_opening['left'])
    with pytest.raises(ValueError):
        arm_state(names,[0]*6+[.09],'right',fk.max_opening['right'])


def test_attachments_and_contact_exclusions_are_preserved(variant):
    root,_,manifest=variant
    convex=ET.parse(root/'piper_moveit_guard/config/dual_piper_meshes.urdf').getroot()
    for name in ('head_camera','left_sensor'):
        mesh=convex.find(f"link[@name='{name}']/collision/geometry/mesh")
        path=root/'piper_moveit_guard/meshes/collision'/Path(mesh.get('filename')).name
        assert path.is_file()
    srdf=ET.parse(root/'piper_moveit_guard/config/dual_piper.srdf').getroot()
    pairs={frozenset((p.get('link1'),p.get('link2'))) for p in srdf.findall('disable_collisions')}
    assert frozenset(('left_sensor','left_link6')) in pairs
    assert frozenset(('left_sensor','left_link3')) not in pairs
    assert not any('head_camera' in pair for pair in pairs)
    assert len(manifest['parts'])==66


def test_nonidentity_origins_scales_and_multiple_collision_shapes(tmp_path):
    root=tmp_path/'variant';root.mkdir()
    for folder in ('piper_moveit_guard','piper_dual_description','piper_collision_guard','model'):
        (root/folder).mkdir()
    cfg=yaml.safe_load((ROOT/'config/robot.yaml').read_text())
    robot=ET.parse(ROOT/'model/dual_piper.source.urdf').getroot()
    link=robot.find("link[@name='left_link6']")
    coll=link.find('collision');coll.find('origin').set('xyz','0.01 0.02 0.03');coll.find('origin').set('rpy','.2 .3 .4')
    coll.find('geometry/mesh').set('scale','1.1 0.9 1.2')
    extra=ET.SubElement(link,'collision');ET.SubElement(ET.SubElement(extra,'geometry'),'box',size='.01 .02 .03')
    source=root/'source.urdf';ET.ElementTree(robot).write(source)
    cfg['source_urdf']=str(source)
    config=root/'robot.yaml';config.write_text(yaml.safe_dump(cfg))
    manifest=prepare(config,root)
    result=ET.parse(root/'piper_moveit_guard/config/dual_piper_meshes.urdf').getroot()
    parts=result.find("link[@name='left_link6']").findall('collision')
    assert len(parts)==2
    assert parts[0].find('origin').get('xyz')=='0.01 0.02 0.03'
    original=mesh_triangles(ROOT/'piper_dual_description/meshes/link6.STL').reshape(-1,3)*[1.1,.9,1.2]
    planes=np.array(manifest['parts']['left_link6:1']['support_planes_m'])
    assert (original@planes[:,:3].T+planes[:,3]).max()<=1e-7


def test_bad_config_does_not_replace_last_good_generation(variant,tmp_path):
    root,cfg,_=variant
    path=root/'piper_moveit_guard/config/dual_piper_meshes.urdf'
    before=path.read_bytes()
    invalid=copy.deepcopy(cfg);invalid['polyhedra']['sections']['link2']['cuts_m']=[100.]
    config=tmp_path/'bad.yaml';config.write_text(yaml.safe_dump(invalid))
    with pytest.raises(ValueError,match='Section cuts'):
        prepare(config,root)
    assert path.read_bytes()==before


def test_existing_non_gripper_convex_envelopes_unchanged():
    # Compare geometric envelopes rather than triangulation order.
    if not (ROOT.parent/"collision").is_dir():
        pytest.skip("Legacy comparison model is only present in the full repository")
    new=ET.parse(ROOT/'piper_moveit_guard/config/dual_piper_meshes.urdf').getroot()
    old=ET.parse(ROOT.parent/'collision/piper_moveit_guard/config/dual_piper_meshes.urdf').getroot()
    for link in old.findall('link'):
        if link.get('name').endswith('_gripper_base'):
            continue
        counterparts=new.find(f"link[@name='{link.get('name')}']").findall('collision')
        originals=link.findall('collision')
        assert len(counterparts)==len(originals)
        for a,b in zip(originals,counterparts):
            def vertices(c,base):
                file=Path(c.find('geometry/mesh').get('filename')).name
                pts=mesh_triangles(base/'piper_moveit_guard/meshes/collision'/file).reshape(-1,3)
                return np.unique(pts,axis=0)
            np.testing.assert_allclose(vertices(a,ROOT.parent/'collision'),vertices(b,ROOT),atol=1e-7)
