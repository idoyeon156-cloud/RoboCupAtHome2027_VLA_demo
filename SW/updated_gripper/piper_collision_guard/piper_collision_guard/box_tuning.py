"""Validated, portable collision-box profiles. Units on disk: mm and degrees."""
import copy
import json
import math
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


class BoxProfile:
    def __init__(self, description):
        self.base = description
        root = ET.fromstring(description)
        self.defaults = {}
        for link in root.findall('link'):
            entries = []
            for collision in link.findall('collision'):
                box = collision.find('geometry/box')
                if box is None:
                    raise ValueError('Box-only collision model required')
                origin = collision.find('origin')
                size = [float(x)*1000 for x in box.get('size').split()]
                entries.append({'center_mm': [float(x)*1000 for x in origin.get('xyz', '0 0 0').split()],
                                'rotation_deg': [math.degrees(float(x)) for x in origin.get('rpy', '0 0 0').split()],
                                'size_mm': [x-10 for x in size], 'margin_mm': 5.0})
            if entries:
                self.defaults[link.get('name')] = entries
        self.links = copy.deepcopy(self.defaults)

    def validate(self, links):
        if not isinstance(links, dict) or set(links) != set(self.defaults):
            raise ValueError('Profile must include exactly the original collision links')
        for entries in links.values():
            if not isinstance(entries, list) or not 1 <= len(entries) <= 8:
                raise ValueError('Each link needs 1 to 8 boxes')
            for box in entries:
                if not isinstance(box, dict) or set(box) != {'center_mm','rotation_deg','size_mm','margin_mm'}:
                    raise ValueError('Invalid box fields')
                for key, lower, upper in [('center_mm',-2000,2000),('rotation_deg',-360,360),('size_mm',1,2000)]:
                    values = box[key]
                    if not isinstance(values, list) or len(values) != 3 or any(
                        isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) or not lower <= x <= upper for x in values):
                        raise ValueError(f'Invalid {key}')
                margin = box['margin_mm']
                if isinstance(margin,bool) or not isinstance(margin,(int,float)) or not math.isfinite(margin) or not 0 <= margin <= 100:
                    raise ValueError('margin_mm must be in [0,100]')

    def load(self, path):
        data = json.loads(Path(path).read_text())
        if not isinstance(data,dict) or data.get('schema_version') != 1:
            raise ValueError('Unsupported profile version')
        self.validate(data.get('links'))
        self.links = copy.deepcopy(data['links'])

    def save(self, path):
        self.validate(self.links)
        path = Path(path).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        data = json.dumps({'schema_version':1,'units':'mm/degrees','links':self.links},indent=2,allow_nan=False)+'\n'
        tmp = None
        try:
            with tempfile.NamedTemporaryFile(mode='w',dir=path.parent,delete=False,encoding='utf-8') as stream:
                tmp = stream.name
                stream.write(data)
            os.replace(tmp,path)
        finally:
            if tmp and os.path.exists(tmp): os.unlink(tmp)
        return path

    def description(self):
        self.validate(self.links)
        root = ET.fromstring(self.base)
        for link in root.findall('link'):
            if link.get('name') not in self.links:
                continue
            for old in list(link.findall('collision')): link.remove(old)
            for box in self.links[link.get('name')]:
                collision = ET.SubElement(link,'collision')
                ET.SubElement(collision,'origin',xyz=' '.join(str(x/1000) for x in box['center_mm']),
                              rpy=' '.join(str(math.radians(x)) for x in box['rotation_deg']))
                geometry = ET.SubElement(collision,'geometry')
                ET.SubElement(geometry,'box',size=' '.join(str((x+2*box['margin_mm'])/1000) for x in box['size_mm']))
        return ET.tostring(root,encoding='unicode')


def tuned_description(base_path, profile_path=''):
    profile = BoxProfile(Path(base_path).read_text())
    if profile_path: profile.load(Path(profile_path).expanduser())
    return profile.description()
