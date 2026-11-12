"""Convex collision solids cover every clipped source face and preserve chamfers."""
import importlib.util
import json
from pathlib import Path
import sys
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
import pytest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from generate_collision_boxes import triangles
from generate_collision_meshes import source_sections


def test_all_parts_enclose_their_source_faces_with_padding_and_are_closed_convex():
    meta=json.loads((ROOT/'piper_moveit_guard/config/collision_meshes.json').read_text())
    assert meta['margin_m']==.005
    assert len(meta['parts'])==11
    assert sum(map(len,meta['parts'].values()))==32
    for stem,parts in meta['parts'].items():
        sections=source_sections(triangles(ROOT/f'piper_dual_description/meshes/{stem}.STL'),stem)
        for points,entry in zip(sections,parts):
            faces=np.array(triangles(ROOT/'piper_moveit_guard/meshes/collision'/entry['file']))
            vertices=np.unique(faces.reshape(-1,3),axis=0)
            hull=ConvexHull(vertices)
            assert len(faces)<1024
            # All vertices of every clipped source triangle are in this convex
            # solid with at least 5 mm clearance from every boundary plane.
            assert (points @ hull.equations[:,:3].T+hull.equations[:,3]).max() <= -.005+2e-7
            assert np.all(vertices.min(0)>=points.min(0)-.005-2e-7)
            assert np.all(vertices.max(0)<=points.max(0)+.005+2e-7)
            edges={}
            for face in faces:
                normal=np.cross(face[1]-face[0],face[2]-face[0])
                assert np.dot(normal,face.mean(0)-vertices.mean(0))>0
                for a,b in ((0,1),(1,2),(2,0)):
                    edge=tuple(sorted((tuple(face[a]),tuple(face[b]))))
                    edges[edge]=edges.get(edge,0)+1
            assert set(edges.values())=={2}  # Watertight closed surfaces.
            old_volume=np.prod(points.max(0)-points.min(0)+.010)
            assert hull.volume < old_volume*.98  # Cut former empty corners.


def test_mesh_model_preserves_all_joints_visuals_and_both_arms():
    source=ET.parse(ROOT/'piper_dual_description/urdf/dual_piper.urdf').getroot()
    model=ET.parse(ROOT/'piper_moveit_guard/config/dual_piper_meshes.urdf').getroot()
    assert [ET.tostring(x).strip() for x in source.findall('joint')]==[ET.tostring(x).strip() for x in model.findall('joint')]
    assert len(model.findall('link/collision'))==64
    for old,new in zip(source.findall('link'),model.findall('link')):
        assert ET.tostring(old.find('visual')).strip()==ET.tostring(new.find('visual')).strip() if old.find('visual') is not None else new.find('visual') is None
        for c in new.findall('collision'):
            assert c.find('geometry/mesh') is not None
    for left in model.findall('link'):
        if not left.get('name').startswith('left_'):continue
        right=model.find("link[@name='right_"+left.get('name')[5:]+"']")
        assert [c.find('geometry/mesh').get('filename') for c in left.findall('collision')]==[c.find('geometry/mesh').get('filename') for c in right.findall('collision')]


def test_loader_defaults_to_meshes_and_explicitly_separates_legacy_profiles(monkeypatch,tmp_path):
    from piper_collision_guard import collision_model
    monkeypatch.setattr(collision_model,'get_package_share_directory',lambda name:str(ROOT/name))
    # A stale box profile must not silently replace the new mesh default.
    monkeypatch.chdir(tmp_path)
    (tmp_path/'collision_tuning.json').write_text('{}')
    description,_,_=collision_model.load_model(ROOT/'piper_moveit_guard')
    model=ET.fromstring(description)
    assert all(c.find('geometry/mesh') is not None for c in model.findall('link/collision'))
    assert all(Path(unquote(urlparse(m.get('filename')).path)).is_file() for m in model.iter('mesh'))
    with pytest.raises(ValueError,match='box profile'):
        collision_model.load_model(ROOT/'piper_moveit_guard','meshes','old.json')
