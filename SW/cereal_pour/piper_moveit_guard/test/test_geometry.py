"""Source-surface coverage and compound-box checks without ROS middleware."""
from pathlib import Path
import importlib.util
import itertools
import math
import xml.etree.ElementTree as ET
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('boxes', ROOT/'tools/generate_collision_boxes.py')
boxes = importlib.util.module_from_spec(spec)
spec.loader.exec_module(boxes)

EXPECTED_COUNTS = {'link2': 5, 'link3': 5, 'link4': 3, 'link5': 2,
                   'gripper_base': 5, 'link7': 4, 'link8': 4}


def box_bounds(link):
    origins = np.array([list(map(float, c.find('origin').get('xyz').split()))
                        for c in link.findall('collision')])
    sizes = np.array([list(map(float, c.find('geometry/box').get('size').split()))
                     for c in link.findall('collision')])
    return origins-sizes/2, origins+sizes/2


def test_boxes_cover_source_faces_with_five_mm_margin_and_preserve_kinematics():
    source = ET.parse(ROOT/'piper_dual_description/urdf/dual_piper.urdf').getroot()
    target = ET.parse(ROOT/'piper_moveit_guard/config/dual_piper_boxes.urdf').getroot()
    assert [ET.tostring(j).strip() for j in source.findall('joint')] == [
        ET.tostring(j).strip() for j in target.findall('joint')]
    count = 0
    for link in source.findall('link'):
        if link.find('collision') is None:
            continue
        name = link.get('name')
        new = target.find(f"link[@name='{name}']")
        assert ET.tostring(link.find('visual')).strip() == ET.tostring(new.find('visual')).strip()
        uri = link.find('collision/geometry/mesh').get('filename')
        path = ROOT/'piper_dual_description'/uri.split('piper_dual_description/')[1]
        faces = np.array(boxes.triangles(path))
        lower, upper = box_bounds(new)
        assert 1 <= len(lower) <= 8
        assert np.all(upper-lower > .010)
        if path.stem in boxes.SECTIONS:
            axis, cuts = boxes.SECTIONS[path.stem]
            # Independently construct every clipped triangle's possible vertices:
            # original vertices plus intersections of its edges and cut planes.
            # AABB containment of those vertices also contains each clipped face.
            samples = [faces.reshape(-1, 3)]
            for cut in cuts:
                for a, b in ((0, 1), (1, 2), (2, 0)):
                    start, end = faces[:, a], faces[:, b]
                    delta = end[:, axis]-start[:, axis]
                    crossing = ((start[:, axis]-cut)*(end[:, axis]-cut) <= 0) & (delta != 0)
                    t = (cut-start[crossing, axis])/delta[crossing]
                    samples.append(start[crossing]+t[:, None]*(end[crossing]-start[crossing]))
            samples = np.concatenate(samples)
            assert len(lower) == EXPECTED_COUNTS[path.stem]
        else:
            samples = faces.reshape(-1, 3)
            lo, hi = faces.min(axis=(0, 1)), faces.max(axis=(0, 1))
            assert lower[0] == pytest.approx(lo-.005, abs=1e-10)
            assert upper[0] == pytest.approx(hi+.005, abs=1e-10)
        # Every face point must fit inside one box with 5 mm remaining on all
        # axes. This checks padding as well as coverage, including section seams.
        covered = np.zeros(len(samples), dtype=bool)
        for lo, hi in zip(lower, upper):
            covered |= np.all(samples >= lo+.005-1e-10, axis=1) & np.all(samples <= hi-.005+1e-10, axis=1)
        assert covered.all(), name
        count += len(lower)
    assert count == 64
    left = float(target.find("joint[@name='world_to_left_base']/origin").get('xyz').split()[1])
    right = float(target.find("joint[@name='world_to_right_base']/origin").get('xyz').split()[1])
    assert left-right == pytest.approx(.175)


def test_both_arms_have_identical_sections_and_less_empty_volume():
    target = ET.parse(ROOT/'piper_moveit_guard/config/dual_piper_boxes.urdf').getroot()
    for name in boxes.SECTIONS:
        left = target.find(f"link[@name='left_{name}']")
        right = target.find(f"link[@name='right_{name}']")
        lo, hi = box_bounds(left)
        other_lo, other_hi = box_bounds(right)
        assert lo == pytest.approx(other_lo)
        assert hi == pytest.approx(other_hi)
        original_lo, original_hi = boxes.bounds(ROOT/f'piper_dual_description/meshes/{name}.STL')
        old_volume = math.prod(b-a+.010 for a, b in zip(original_lo, original_hi))
        # Compute the union, rather than double-counting 5 mm overlaps at
        # seams. The wrist is rounder; body and fingers have larger voids.
        edges = [sorted(set(lo[:, k]) | set(hi[:, k])) for k in range(3)]
        volume = 0.
        for cell in itertools.product(*[list(zip(e, e[1:])) for e in edges]):
            midpoint = np.array([(a+b)/2 for a, b in cell])
            if np.any(np.all(midpoint >= lo, axis=1) & np.all(midpoint <= hi, axis=1)):
                volume += math.prod(b-a for a, b in cell)
        minimum_reduction = .08 if name == 'link4' else .20
        assert volume < (1-minimum_reduction)*old_volume
    for side in ('left', 'right'):
        assert len(target.find(f"link[@name='{side}_link6']").findall('collision')) == 1


def test_gripper_side_wing_void_and_narrow_fingertip_are_not_filled():
    target = ET.parse(ROOT/'piper_moveit_guard/config/dual_piper_boxes.urdf').getroot()
    # Points that the former full bounding boxes included, but that sit in
    # empty space below the side wing and next to the tapered fingertip.
    for side in ('left', 'right'):
        for name, point in [('gripper_base', [.025, .060, .010]),
                            ('link7', [.025, -.010, .020]),
                            ('link8', [.025, -.010, .020])]:
            lo, hi = box_bounds(target.find(f"link[@name='{side}_{name}']"))
            point = np.array(point)
            assert np.all(point >= lo.min(axis=0)) and np.all(point <= hi.max(axis=0))
            assert not np.any(np.all(point >= lo, axis=1) & np.all(point <= hi, axis=1))


def test_long_triangle_crossing_sections_with_no_internal_stl_vertices():
    faces = [((0., 0., 0.), (1., 0., 0.), (0., 1., 1.))]
    sections = boxes.section_bounds(faces, 0, (.25, .75))
    assert len(sections) == 3
    assert sections[1][0] == pytest.approx([.25, 0., 0.])
    assert sections[1][1] == pytest.approx([.75, .75, .75])
    for lower, upper in sections:
        assert np.all(np.array(upper) >= lower)


def test_acm_never_excludes_cross_arm_collisions():
    srdf = ET.parse(ROOT/'piper_moveit_guard/config/dual_piper.srdf').getroot()
    exclusions = srdf.findall('disable_collisions')
    assert len(exclusions) == 110
    for pair in exclusions:
        assert pair.get('link1').split('_')[0] == pair.get('link2').split('_')[0]
