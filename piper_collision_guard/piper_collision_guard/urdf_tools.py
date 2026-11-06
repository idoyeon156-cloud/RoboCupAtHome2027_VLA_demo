"""URDF placement helpers, without Pinocchio imports in the ROS launcher."""
import math
import xml.etree.ElementTree as ET


def placed_description(path, separation):
    """Change only the two fixed base translations for visual experiments."""
    if not math.isfinite(separation) or separation <= 0:
        raise ValueError('base separation must be positive and finite')
    root = ET.parse(path).getroot()
    for side, sign in [('left', 1), ('right', -1)]:
        origin = root.find(f"joint[@name='world_to_{side}_base']/origin")
        if origin is None:
            raise ValueError('missing fixed base joint')
        xyz = origin.get('xyz').split()
        xyz[1] = str(sign * separation / 2)
        origin.set('xyz', ' '.join(xyz))
    return ET.tostring(root, encoding='unicode')
