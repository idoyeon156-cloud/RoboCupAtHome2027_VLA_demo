#!/usr/bin/env python3
"""Conservative, low-face-count convex parts for every Piper collision link.

Clip source triangles at the existing link-local sections before fitting each
part. Use supporting planes (never an inward vertex decimation) and shift them
outwards by the requested margin. Axial planes bound the result by the previous
padded section boxes. Keep the dominant source face normals to retain chamfers.
"""
import argparse
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection
from generate_collision_boxes import ROOT, SECTIONS, triangles, clip_polygon


def source_sections(faces, stem):
    points = np.asarray(faces).reshape(-1, 3)
    if stem not in SECTIONS:
        return [points]
    axis, cuts = SECTIONS[stem]
    planes = [points[:, axis].min(), *cuts, points[:, axis].max()]
    result = []
    for low, high in zip(planes, planes[1:]):
        section = []
        for face in faces:
            if max(v[axis] for v in face) < low or min(v[axis] for v in face) > high:
                continue
            section.extend(clip_polygon(clip_polygon(face, axis, low, True), axis, high, False))
        result.append(np.unique(section, axis=0))
    return result


def enclosing_polyhedron(points, margin):
    hull = ConvexHull(points)
    faces = points[hull.simplices]
    area = np.linalg.norm(np.cross(faces[:, 1]-faces[:, 0], faces[:, 2]-faces[:, 0]), axis=1)
    # Preserve large sloping faces, plus broadly distributed support directions.
    directions = [*np.eye(3), *-np.eye(3)]
    for i in np.argsort(-area):
        normal = hull.equations[i, :3]
        if all(np.dot(normal, old) < .999 for old in directions):
            directions.append(normal)
        if len(directions) >= 54:
            break
    for i in range(64):
        z = 1-2*(i+.5)/64
        phi = i*np.pi*(3-np.sqrt(5))
        directions.append(np.array([np.sqrt(1-z*z)*np.cos(phi), np.sqrt(1-z*z)*np.sin(phi), z]))
    normals = np.asarray(directions)
    support = (points[hull.vertices] @ normals.T).max(axis=0)+margin
    halfspaces = np.column_stack([normals, -support])
    interior = points[hull.vertices].mean(axis=0)
    vertices = HalfspaceIntersection(halfspaces, interior).intersections
    outer = ConvexHull(vertices)
    faces = []
    for indices, equation in zip(outer.simplices, outer.equations):
        triangle = vertices[indices].copy()
        if np.dot(np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0]), equation[:3]) < 0:
            triangle[[1, 2]] = triangle[[2, 1]]
        faces.append(triangle)
    return np.asarray(faces), halfspaces


def write_stl(path, faces):
    raw = bytearray(b'Piper enclosing convex collision part'.ljust(80, b' '))
    raw.extend(struct.pack('<I', len(faces)))
    for face in faces:
        normal = np.cross(face[1]-face[0], face[2]-face[0])
        normal /= np.linalg.norm(normal)
        raw.extend(struct.pack('<12fH', *normal, *face.flatten(), 0))
    path.write_bytes(raw)


def generate(margin=.005):
    if not np.isfinite(margin) or margin < 0:
        raise ValueError('margin must be finite and nonnegative')
    tree = ET.parse(ROOT/'piper_dual_description/urdf/dual_piper.urdf')
    output = ROOT/'piper_moveit_guard'
    mesh_dir = output/'meshes/collision'
    mesh_dir.mkdir(parents=True, exist_ok=True)
    parts = {}
    for path in sorted((ROOT/'piper_dual_description/meshes').glob('*.STL')):
        entries = []
        for i, points in enumerate(source_sections(triangles(path), path.stem), 1):
            faces, planes = enclosing_polyhedron(points, margin)
            filename = f'{path.stem}_part{i}.stl'
            write_stl(mesh_dir/filename, faces)
            entries.append({'file': filename, 'triangles': len(faces), 'support_planes_m': planes.tolist()})
        parts[path.stem] = entries
    for link in tree.getroot().findall('link'):
        for old in link.findall('collision'):
            uri = old.find('geometry/mesh').get('filename')
            origin = old.find('origin')
            if any(float(x) for key in ('xyz', 'rpy') for x in origin.get(key, '0 0 0').split()):
                raise ValueError('Nonidentity source collision origin')
            if old.find('geometry/mesh').get('scale', '1 1 1') != '1 1 1':
                raise ValueError('Nonidentity source mesh scale')
            stem = Path(uri).stem
            link.remove(old)
            for i, entry in enumerate(parts[stem], 1):
                collision = ET.SubElement(link, 'collision', name=f'{link.get("name")}_part{i}')
                ET.SubElement(collision, 'origin', xyz='0 0 0', rpy='0 0 0')
                geometry = ET.SubElement(collision, 'geometry')
                ET.SubElement(geometry, 'mesh', filename='package://piper_moveit_guard/meshes/collision/'+entry['file'])
    ET.indent(tree, space='  ')
    tree.write(output/'config/dual_piper_meshes.urdf', encoding='utf-8', xml_declaration=True)
    (output/'config/collision_meshes.json').write_text(json.dumps(
        {'margin_m': margin, 'method': 'outward_support_planes_per_source_section', 'parts': parts}, indent=2)+'\n')
    print(f'Generated {sum(map(len,parts.values()))} unique convex parts; shared by both arms, {sum(e["triangles"] for p in parts.values() for e in p)} triangles per arm')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--margin', type=float, default=.005)
    generate(parser.parse_args().margin)
