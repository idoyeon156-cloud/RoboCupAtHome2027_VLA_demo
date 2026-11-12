#!/usr/bin/env python3
"""Fit link-local compound boxes to Piper STL sections, preserving kinematics."""
import argparse
import csv
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]

# Cuts are in each source STL's local frame (metres). Separate thick housings,
# narrow connecting arms, gripper body/side wings and tapered fingers. Keep
# each part on its original moving link, with at most eight boxes per link.
# The thin link6 flange stays in its own enclosing box.
SECTIONS = {
    'link2': (0, (0.055, 0.105, 0.195, 0.235)),
    'link3': (1, (-0.180, -0.145, -0.065, -0.030)),
    'link4': (2, (-0.020, 0.012)),
    'link5': (1, (-0.020,)),
    'gripper_base': (1, (-0.048, -0.024, 0.024, 0.048)),
    'link7': (1, (-0.060, -0.040, -0.020)),
    'link8': (1, (-0.060, -0.040, -0.020)),
}


def triangles(path):
    raw = path.read_bytes()
    n = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) != 84 + n * 50 or not n:
        raise ValueError(f'Expected nonempty binary STL: {path}')
    return [tuple(struct.unpack_from('<fff', raw, 84 + i*50 + 12 + j*12)
                  for j in range(3)) for i in range(n)]


def point_bounds(pts):
    return ([min(v[k] for v in pts) for k in range(3)],
            [max(v[k] for v in pts) for k in range(3)])


def bounds(path):
    return point_bounds([v for triangle in triangles(path) for v in triangle])


def clip_polygon(polygon, axis, plane, keep_above):
    """Clip a triangle/polygon against a closed axis-aligned half-space.

    Include edge/plane intersections: selecting only existing STL vertices
    would miss long faces crossing a section that contains no vertices.
    """
    if not polygon:
        return []
    result = []
    previous = polygon[-1]
    previous_inside = (previous[axis] >= plane if keep_above else previous[axis] <= plane)
    for current in polygon:
        inside = (current[axis] >= plane if keep_above else current[axis] <= plane)
        if inside != previous_inside:
            t = (plane-previous[axis])/(current[axis]-previous[axis])
            intersection = [a+t*(b-a) for a, b in zip(previous, current)]
            intersection[axis] = plane
            result.append(tuple(intersection))
        if inside:
            result.append(current)
        previous, previous_inside = current, inside
    return result


def section_bounds(faces, axis, cuts):
    """Enclose all clipped triangle surfaces in each section, before padding."""
    lo, hi = point_bounds([v for face in faces for v in face])
    planes = [lo[axis], *cuts, hi[axis]]
    if any(a >= b for a, b in zip(planes, planes[1:])):
        raise ValueError('Section cuts must be ordered and inside the STL bounds')
    sections = []
    for lower, upper in zip(planes, planes[1:]):
        points = []
        for face in faces:
            if max(v[axis] for v in face) < lower or min(v[axis] for v in face) > upper:
                continue
            polygon = clip_polygon(face, axis, lower, True)
            points.extend(clip_polygon(polygon, axis, upper, False))
        if not points:
            raise ValueError('Empty STL section')
        sections.append(point_bounds(points))
    return sections


def generate(margin=0.005):
    import math
    if not math.isfinite(margin) or margin < 0:
        raise ValueError('margin must be finite and nonnegative')
    package = ROOT / 'piper_dual_description'
    tree = ET.parse(package/'urdf/dual_piper.urdf')
    rows = []
    cache = {}
    for link in tree.getroot().findall('link'):
        for collision in link.findall('collision'):
            geom = collision.find('geometry')
            mesh = geom.find('mesh')
            uri = mesh.get('filename')
            prefix = 'package://piper_dual_description/'
            if not uri.startswith(prefix):
                raise ValueError(uri)
            path = package / uri[len(prefix):]
            if uri not in cache:
                faces = triangles(path)
                cache[uri] = (section_bounds(faces, *SECTIONS[path.stem])
                              if path.stem in SECTIONS else
                              [point_bounds([v for face in faces for v in face])])
            # Source collision transforms/scales must be identity for this generator.
            origin = collision.find('origin')
            if mesh.get('scale', '1 1 1') != '1 1 1' or any(
                    float(x) != 0 for field in ('xyz', 'rpy')
                    for x in origin.get(field, '0 0 0').split()):
                raise ValueError('Nonidentity source collision geometry')
            link.remove(collision)
            for index, (lo, hi) in enumerate(cache[uri], 1):
                center = [(a+b)/2 for a,b in zip(lo, hi)]
                size = [b-a+2*margin for a,b in zip(lo, hi)]
                part = ET.SubElement(link, 'collision', name=f'{link.get("name")}_box_{index}')
                ET.SubElement(part, 'origin', xyz=' '.join(f'{x:.12f}' for x in center), rpy='0 0 0')
                geometry = ET.SubElement(part, 'geometry')
                ET.SubElement(geometry, 'box', size=' '.join(f'{x:.12f}' for x in size))
                rows.append([link.get('name'), *center, *size, margin])
    ET.indent(tree, space='  ')
    out = ROOT/'piper_moveit_guard/config'
    tree.write(out/'dual_piper_boxes.urdf', encoding='utf-8', xml_declaration=True)
    with (out/'collision_boxes.csv').open('w') as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(['link','center_x_m','center_y_m','center_z_m','size_x_m','size_y_m','size_z_m','margin_per_face_m'])
        writer.writerows(rows)
    # Scope: cross-arm checking only. Same-arm checks are intentionally excluded.
    srdf = ET.Element('robot', name='dual_piper')
    for side in ('left', 'right'):
        group = ET.SubElement(srdf, 'group', name=f'{side}_arm')
        for i in range(1, 7):
            ET.SubElement(group, 'joint', name=f'{side}_joint{i}')
        names = [l.get('name') for l in tree.getroot().findall('link') if l.get('name').startswith(side+'_')]
        for i, a in enumerate(names):
            for b in names[i+1:]:
                ET.SubElement(srdf, 'disable_collisions', link1=a, link2=b, reason='SameArmExcluded')
    group = ET.SubElement(srdf, 'group', name='both_arms')
    for side in ('left', 'right'):
        ET.SubElement(group, 'group', name=f'{side}_arm')
    stree = ET.ElementTree(srdf)
    ET.indent(stree, space='  ')
    stree.write(out/'dual_piper.srdf', encoding='utf-8', xml_declaration=True)
    print(f'Generated {len(rows)} boxes, {margin*1000:g} mm extra on each face')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--margin', type=float, default=0.005)
    generate(parser.parse_args().margin)
