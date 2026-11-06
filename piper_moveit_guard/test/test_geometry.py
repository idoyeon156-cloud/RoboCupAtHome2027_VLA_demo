"""Geometry checks independent of ROS middleware."""
from pathlib import Path
import importlib.util
import xml.etree.ElementTree as ET
import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('boxes', ROOT/'tools/generate_collision_boxes.py')
boxes = importlib.util.module_from_spec(spec)
spec.loader.exec_module(boxes)


def test_boxes_cover_source_mesh_with_five_mm_margin_and_preserve_kinematics():
    source = ET.parse(ROOT/'piper_dual_description/urdf/dual_piper.urdf').getroot()
    target = ET.parse(ROOT/'piper_moveit_guard/config/dual_piper_boxes.urdf').getroot()
    for old, new in zip(source.findall('joint'), target.findall('joint')):
        assert ET.tostring(old).strip() == ET.tostring(new).strip()
    count = 0
    for link in source.findall('link'):
        if link.find('collision') is None:
            continue
        new = target.find(f"link[@name='{link.get('name')}']")
        assert ET.tostring(link.find('visual')).strip() == ET.tostring(new.find('visual')).strip()
        uri = link.find('collision/geometry/mesh').get('filename')
        lo, hi = boxes.bounds(ROOT/'piper_dual_description'/uri.split('piper_dual_description/')[1])
        center = list(map(float, new.find('collision/origin').get('xyz').split()))
        size = list(map(float, new.find('collision/geometry/box').get('size').split()))
        for k in range(3):
            assert center[k]-size[k]/2 == pytest.approx(lo[k]-.005, abs=1e-10)
            assert center[k]+size[k]/2 == pytest.approx(hi[k]+.005, abs=1e-10)
        count += 1
    assert count == 20
    left = float(target.find("joint[@name='world_to_left_base']/origin").get('xyz').split()[1])
    right = float(target.find("joint[@name='world_to_right_base']/origin").get('xyz').split()[1])
    assert left-right == pytest.approx(.175)


def test_acm_never_excludes_cross_arm_collisions():
    srdf = ET.parse(ROOT/'piper_moveit_guard/config/dual_piper.srdf').getroot()
    exclusions = srdf.findall('disable_collisions')
    assert len(exclusions) == 90
    for pair in exclusions:
        assert pair.get('link1').split('_')[0] == pair.get('link2').split('_')[0]
