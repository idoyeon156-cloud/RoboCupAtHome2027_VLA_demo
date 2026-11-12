"""Keep both guards tied to the separately published replacement robot."""
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from piper_collision_guard.distance import DistanceModel, arm_state

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / 'piper_dual_description'
STANDALONE = ROOT.parent / 'robot_description/piper_dual_updated_gripper_description'


def test_guard_model_and_meshes_match_published_replacement():
    if not STANDALONE.is_dir():
        pytest.skip('Standalone package comparison requires the full repository')
    published = (STANDALONE / 'urdf/dual_piper.urdf').read_text()
    expected = published.replace('piper_dual_updated_gripper_description', 'piper_dual_description')
    assert (DESCRIPTION / 'urdf/dual_piper.urdf').read_text() == expected
    for mesh in (STANDALONE / 'meshes').glob('*.STL'):
        assert hashlib.sha256(mesh.read_bytes()).digest() == hashlib.sha256(
            (DESCRIPTION / 'meshes' / mesh.name).read_bytes()).digest()


def test_new_flange_and_gripper_are_in_moveit_cross_arm_checks():
    source = ET.parse(DESCRIPTION / 'urdf/dual_piper.urdf').getroot()
    mesh = ET.parse(ROOT / 'piper_moveit_guard/config/dual_piper_meshes.urdf').getroot()
    assert [ET.tostring(j) for j in source.findall('joint')] == [
        ET.tostring(j) for j in mesh.findall('joint')]
    for side in ('left', 'right'):
        for suffix in ('gripper_flange', 'gripper_base'):
            link = mesh.find(f"link[@name='{side}_{suffix}']")
            assert link is not None and link.findall('collision')
            assert all(c.find('geometry/mesh') is not None for c in link.findall('collision'))
    srdf = ET.parse(ROOT / 'piper_moveit_guard/config/dual_piper.srdf').getroot()
    for exclusion in srdf.findall('disable_collisions'):
        a, b = exclusion.get('link1'), exclusion.get('link2')
        assert not (a.startswith('left_') and b.startswith('right_'))
        assert not (a.startswith('right_') and b.startswith('left_'))


def test_distance_guard_uses_replacement_opening_and_protection_frame():
    protection = yaml.safe_load((ROOT / 'piper_collision_guard/config/protection.yaml').read_text())
    fk = DistanceModel(DESCRIPTION / 'urdf/dual_piper.urdf', 0, protection)
    assert fk.max_opening == pytest.approx({'left': .100, 'right': .100})
    for side in ('left', 'right'):
        assert fk.offsets[side][-1, 2] == pytest.approx(.138)
        names = [f'joint{i}' for i in range(1, 7)] + ['gripper']
        _, opening = arm_state(names, [0.] * 6 + [.100], side, fk.max_opening[side])
        assert opening == .100
        with pytest.raises(ValueError):
            arm_state(names, [0.] * 6 + [.101], side, fk.max_opening[side])
    assert fk.clearance(np.zeros(6), np.zeros(6)).distance == pytest.approx(.095)
