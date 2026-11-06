#!/usr/bin/env python3
"""Generate link-local enclosing boxes; preserve source visuals and kinematics."""
import argparse
import csv
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def bounds(path):
    raw = path.read_bytes()
    n = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) != 84 + n * 50 or not n:
        raise ValueError(f'Expected nonempty binary STL: {path}')
    pts = [struct.unpack_from('<fff', raw, 84 + i*50 + 12 + j*12)
           for i in range(n) for j in range(3)]
    return ([min(v[k] for v in pts) for k in range(3)],
            [max(v[k] for v in pts) for k in range(3)])


def generate(margin=0.005):
    import math
    if not math.isfinite(margin) or margin < 0:
        raise ValueError('margin must be finite and nonnegative')
    package = ROOT / 'piper_dual_description'
    tree = ET.parse(package/'urdf/dual_piper.urdf')
    rows = []
    for link in tree.getroot().findall('link'):
        for collision in link.findall('collision'):
            geom = collision.find('geometry')
            mesh = geom.find('mesh')
            uri = mesh.get('filename')
            prefix = 'package://piper_dual_description/'
            if not uri.startswith(prefix):
                raise ValueError(uri)
            lo, hi = bounds(package / uri[len(prefix):])
            # Source collision transforms/scales must be identity for this generator.
            origin = collision.find('origin')
            if mesh.get('scale', '1 1 1') != '1 1 1' or any(
                    float(x) != 0 for field in ('xyz', 'rpy')
                    for x in origin.get(field, '0 0 0').split()):
                raise ValueError('Nonidentity source collision geometry')
            center = [(a+b)/2 for a,b in zip(lo, hi)]
            size = [b-a+2*margin for a,b in zip(lo, hi)]
            origin.set('xyz', ' '.join(f'{x:.12f}' for x in center))
            geom.remove(mesh)
            ET.SubElement(geom, 'box', size=' '.join(f'{x:.12f}' for x in size))
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
