"""URDF placement helpers, without Pinocchio imports in the ROS launcher."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def placed_description(path, separation, mesh_root=None):
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
    if mesh_root is not None:
        # RViz's package URI expansion produces an unescaped file:// URL.
        # libcurl rejects spaces/non-ASCII in that URL. Encode the actual file
        # path here; preserve the portable package URIs in the saved URDF.
        prefix = 'package://piper_dual_description/'
        for mesh in root.iter('mesh'):
            uri = mesh.get('filename', '')
            if uri.startswith(prefix):
                filename = (Path(mesh_root) / uri[len(prefix):]).resolve()
                if not filename.is_file():
                    raise ValueError(f'missing mesh: {filename}')
                mesh.set('filename', filename.as_uri())
    return ET.tostring(root, encoding='unicode')
