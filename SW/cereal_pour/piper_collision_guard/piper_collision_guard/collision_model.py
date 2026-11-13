"""Load the same collision geometry and resolve mesh URIs in both launchers."""
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from .box_tuning import tuned_description


def load_semantic(share, description):
    """Inspect arm exteriors against the OTHER arm and environment.

    Preserve the deployment's assembly-internal exclusions. Do not report
    contact between separately meshed parts of the same arm as an external
    collision. No cross-arm or robot/environment exclusion is retained.
    """
    semantic = ET.fromstring((Path(share)/'config/dual_piper.srdf').read_text())
    robot = ET.fromstring(description)
    for entry in list(semantic.findall('disable_collisions')):
        a, b = entry.get('link1', ''), entry.get('link2', '')
        if not any(a.startswith(side+'_') and b.startswith(side+'_') for side in ('left', 'right')):
            semantic.remove(entry)
    existing = {frozenset((e.get('link1'), e.get('link2')))
                for e in semantic.findall('disable_collisions')}
    for side in ('left', 'right'):
        links = sorted(e.get('name') for e in robot.findall('link')
                       if e.get('name').startswith(side+'_') and e.find('collision') is not None)
        for i, a in enumerate(links):
            for b in links[i+1:]:
                key = frozenset((a, b))
                if key not in existing:
                    ET.SubElement(semantic, 'disable_collisions', link1=a, link2=b,
                                  reason='SameArmAssemblyInternal')
                    existing.add(key)
    return ET.tostring(semantic, encoding='unicode')


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
            mesh.set('filename', (Path(get_package_share_directory(package))/relative).resolve().as_uri())
    return ET.tostring(root, encoding='unicode'), base, profile_path
