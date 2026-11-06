#!/usr/bin/env python3
"""Export the URDF zero pose to binary STL in millimetres for Fusion viewing.

The STL is a static visualisation; it carries no joints or collision logic.
Run from any directory: python tools/export_preview.py (requires numpy).
"""
from pathlib import Path
import math
import struct
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "piper_dual_description"


def transform(origin):
    result = np.eye(4)
    if origin is None:
        return result
    r, p, y = map(float, origin.get("rpy", "0 0 0").split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]]
    result[:3, 3] = list(map(float, origin.get("xyz", "0 0 0").split()))
    return result


def read_stl(path):
    raw = path.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    if len(raw) != 84 + 50 * count:
        raise ValueError(f"Expected binary STL: {path}")
    dtype = np.dtype([("normal", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(raw, dtype=dtype, count=count, offset=84)["v"].astype(float)


def main():
    robot = ET.parse(PACKAGE / "urdf/dual_piper.urdf").getroot()
    frames = {"world": np.eye(4)}
    pending = list(robot.findall("joint"))
    while pending:
        ready = [j for j in pending if j.find("parent").get("link") in frames]
        if not ready:
            raise ValueError("Disconnected or cyclic URDF")
        for joint in ready:
            frames[joint.find("child").get("link")] = (
                frames[joint.find("parent").get("link")] @ transform(joint.find("origin"))
            )  # all movable joints are zero
            pending.remove(joint)
    triangles = []
    for link in robot.findall("link"):
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                raise ValueError("This exporter expects mesh visuals")
            uri = mesh.get("filename")
            prefix = "package://piper_dual_description/"
            if not uri.startswith(prefix):
                raise ValueError(f"Unexpected mesh URI: {uri}")
            vertices = read_stl(PACKAGE / uri[len(prefix):])
            vertices *= np.array(list(map(float, mesh.get("scale", "1 1 1").split())))
            pose = frames[link.get("name")] @ transform(visual.find("origin"))
            triangles.append((vertices @ pose[:3, :3].T + pose[:3, 3]) * 1000.0)
    vertices = np.concatenate(triangles)
    assert np.isfinite(vertices).all()
    normals = np.cross(vertices[:, 1]-vertices[:, 0], vertices[:, 2]-vertices[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    nonzero = lengths > 0
    normals[nonzero] /= lengths[nonzero, None]
    dtype = np.dtype([("normal", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    records = np.zeros(len(vertices), dtype=dtype)
    records["normal"], records["v"] = normals, vertices
    output = ROOT / "preview/dual_piper_zero_pose_mm.stl"
    output.parent.mkdir(exist_ok=True)
    output.write_bytes(b"Dual Piper zero pose; units mm; static visual only".ljust(80, b" ")
                       + struct.pack("<I", len(vertices)) + records.tobytes())
    assert abs(frames["left_base_link"][1, 3] - frames["right_base_link"][1, 3] - 0.175) < 1e-12
    print(f"Exported {len(vertices)} triangles to {output}")
    print("Bounds (mm):", vertices.reshape(-1, 3).min(0), vertices.reshape(-1, 3).max(0))


if __name__ == "__main__":
    main()
