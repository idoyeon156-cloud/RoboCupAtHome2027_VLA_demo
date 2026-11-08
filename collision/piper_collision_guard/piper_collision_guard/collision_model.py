"""Load the same collision geometry and resolve mesh URIs in both launchers."""
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from .box_tuning import tuned_description


def load_model(share, geometry='meshes', profile_path=''):
    share = Path(share)
    if geometry not in ('meshes', 'boxes'):
        raise ValueError('geometry must be meshes or boxes')
    if geometry == 'meshes':
        if profile_path:
            raise ValueError('collision_config is a box profile; use geometry:=boxes for legacy box editing')
        base = (share/'config/dual_piper_meshes.urdf').read_text()
        description = base
    else:
        base = (share/'config/dual_piper_boxes.urdf').read_text()
        saved = Path.cwd()/'collision_tuning.json'
        if not profile_path and saved.is_file():
            profile_path = str(saved)
        description = tuned_description(share/'config/dual_piper_boxes.urdf', profile_path)
    root = ET.fromstring(description)
    for mesh in root.iter('mesh'):
        uri = mesh.get('filename', '')
        if uri.startswith('package://'):
            package, relative = uri[len('package://'):].split('/', 1)
            if package not in ('piper_moveit_guard', 'piper_dual_description'):
                raise ValueError('Unexpected mesh package')
            mesh.set('filename', (Path(get_package_share_directory(package))/relative).resolve().as_uri())
    return ET.tostring(root, encoding='unicode'), base, profile_path
