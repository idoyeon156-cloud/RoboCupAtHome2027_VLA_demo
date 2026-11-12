#!/usr/bin/env python3
"""Prepare the configurable workspace only; never write to legacy collision/.

Existing outward-support convex fitting is retained. Install mounts and total
finger opening are applied once, before generating both stages' models.
"""
import argparse
import copy
import hashlib
import itertools
import json
import os
from pathlib import Path
import struct
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
import yaml
from generate_collision_meshes import enclosing_polyhedron, write_stl
from generate_collision_boxes import clip_polygon

ROOT = Path(__file__).resolve().parents[1]


def vector(value, length=3, positive=False):
    a = np.asarray(value, dtype=float)
    if a.shape != (length,) or not np.isfinite(a).all() or (positive and np.any(a <= 0)):
        raise ValueError(f'Expected {length} finite values' + (' > 0' if positive else ''))
    return a


def scalar(value, positive=False):
    x = float(value)
    if not np.isfinite(x) or x < 0 or (positive and x <= 0):
        raise ValueError('Expected finite nonnegative/positive value')
    return x


def origin(element, xyz, rpy):
    pose = element.find('origin')
    if pose is None:
        pose = ET.SubElement(element, 'origin')
    pose.set('xyz', ' '.join(map(str, vector(xyz))))
    pose.set('rpy', ' '.join(map(str, vector(rpy))))


def rotation(rpy):
    r, p, y = vector(rpy)
    cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def resolve_mesh(uri):
    if uri.startswith('package://'):
        package, relative = uri[10:].split('/', 1)
        local = ROOT/package
        if not local.is_dir():
            from ament_index_python.packages import get_package_share_directory
            local = Path(get_package_share_directory(package))
        path = local/relative
    elif uri.startswith('file://'):
        from urllib.parse import unquote, urlparse
        path = Path(unquote(urlparse(uri).path))
    else:
        path = Path(uri)
        if not path.is_absolute():
            path = ROOT/path
    if not path.is_file():
        raise ValueError(f'Missing mesh: {path}')
    return path.resolve()


def mesh_triangles(path):
    raw = path.read_bytes()
    if len(raw) >= 84:
        n = struct.unpack_from('<I', raw, 80)[0]
        if n and len(raw) == 84+n*50:
            faces = [struct.unpack_from('<9f',raw,84+i*50+12) for i in range(n)]
            result = np.asarray(faces,dtype=float).reshape(-1,3,3)
            if not np.isfinite(result).all():
                raise ValueError('Nonfinite STL')
            return result
    vertices = [list(map(float,line.split()[1:])) for line in raw.decode('utf8').splitlines()
                if line.strip().startswith('vertex ')]
    if not vertices or len(vertices)%3:
        raise ValueError(f'Expected binary or ASCII triangle STL: {path}')
    result=np.asarray(vertices).reshape(-1,3,3)
    if not np.isfinite(result).all():
        raise ValueError('Nonfinite STL')
    return result


def primitive_points(geometry):
    if geometry.find('box') is not None:
        size=vector(geometry.find('box').get('size').split(),positive=True)
        return np.asarray(list(itertools.product((-1,1),repeat=3)))*size/2
    if geometry.find('cylinder') is not None:
        e=geometry.find('cylinder');r=scalar(e.get('radius'),True);h=scalar(e.get('length'),True)
        # Circumscribed polygon encloses the circular surface, including between vertices.
        a=np.arange(32)*2*np.pi/32
        return np.array([[r/np.cos(np.pi/32)*np.cos(t),r/np.cos(np.pi/32)*np.sin(t),z]
                         for z in (-h/2,h/2) for t in a])
    if geometry.find('sphere') is not None:
        r=scalar(geometry.find('sphere').get('radius'),True)
        # Conservative cube hull; use a custom STL when a tighter sphere is needed.
        return np.asarray(list(itertools.product((-r,r),repeat=3)))
    raise ValueError('Supported collision shapes: STL mesh, box, cylinder, sphere')


def sections(faces, spec):
    points=faces.reshape(-1,3)
    if spec is None:
        return [points]
    axis=spec['axis']
    if isinstance(axis,str):
        axis={'x':0,'y':1,'z':2}[axis]
    if axis not in (0,1,2):
        raise ValueError('section axis must be 0,1,2 or x,y,z')
    cuts=vector(spec['cuts_m'],len(spec['cuts_m']))
    planes=[points[:,axis].min(),*cuts,points[:,axis].max()]
    if any(a>=b for a,b in zip(planes,planes[1:])):
        raise ValueError('Section cuts must be ordered and inside source mesh bounds; update this part config')
    result=[]
    for lo,hi in zip(planes,planes[1:]):
        pts=[]
        for face in faces:
            if face[:,axis].max()<lo or face[:,axis].min()>hi:
                continue
            pts.extend(clip_polygon(clip_polygon(face.tolist(),axis,lo,True),axis,hi,False))
        if not pts:
            raise ValueError('Empty mesh section')
        result.append(np.unique(pts,axis=0))
    if len(result)>8:
        raise ValueError('At most eight convex parts per collision shape')
    return result


def write_xml(path, robot):
    tree=ET.ElementTree(robot);ET.indent(tree,space='  ')
    tree.write(path,encoding='utf-8',xml_declaration=True)


def attachment_geometry(g, config):
    keys=set(config)-{'scale'}
    if len(keys)!=1:
        raise ValueError('Specify exactly one attachment geometry')
    kind=next(iter(keys));value=config[kind]
    if kind=='box':
        ET.SubElement(g,'box',size=' '.join(map(str,vector(value,positive=True))))
    elif kind=='sphere':
        ET.SubElement(g,'sphere',radius=str(scalar(value,True)))
    elif kind=='cylinder':
        r,h=vector(value,2,True);ET.SubElement(g,'cylinder',radius=str(r),length=str(h))
    elif kind=='mesh':
        ET.SubElement(g,'mesh',filename=str(value),scale=' '.join(map(str,vector(config.get('scale',[1,1,1]),positive=True))))
    else:
        raise ValueError(f'Unknown shape {kind}')


def prepare(config_path, output_root=ROOT):
    output_root=Path(output_root)
    if (output_root.name=='collision' or not (output_root/'piper_moveit_guard').is_dir()):
        raise ValueError('Output must be a separate configurable workspace, never legacy collision/')
    cfg=yaml.safe_load(Path(config_path).read_text())
    allowed={'source_urdf','mounts','grippers','polyhedra','joint_distance','attachments','allowed_collision_pairs'}
    if set(cfg)!=allowed:
        raise ValueError(f'Unexpected/missing configuration keys: {set(cfg)^allowed}')
    source=Path(cfg['source_urdf'])
    if not source.is_absolute():
        source=ROOT/source
    robot=ET.parse(source).getroot()
    # Extra movable axes are deliberately outside this variant's supported scope.
    expected={f'{s}_joint{i}' for s in ('left','right') for i in range(1,9)}
    actual={j.get('name') for j in robot.findall('joint') if j.get('type')!='fixed'}
    if actual!=expected:
        raise ValueError('Keep the existing Piper 6-axis + two-finger joint structure')
    core={s:{f'{s}_{name}' for name in ['base_link',*[f'link{i}' for i in range(1,9)],'gripper_base','gripper_flange']}
          for s in ('left','right')}
    for side in ('left','right'):
        m=cfg['mounts'][side]
        j=robot.find(f"joint[@name='world_to_{side}_base']")
        if j is None or j.get('type')!='fixed':
            raise ValueError('Missing fixed base mount')
        origin(j,m['xyz'],m['rpy'])
        opening=scalar(cfg['grippers'][side]['max_opening_m'],True)
        for index,lo,hi in ((7,0,opening/2),(8,-opening/2,0)):
            joint=robot.find(f"joint[@name='{side}_joint{index}']")
            if joint.get('type')!='prismatic' or joint.find('mimic') is not None:
                raise ValueError('Expected the original independent prismatic finger joints')
            limit=joint.find('limit');limit.set('lower',str(lo));limit.set('upper',str(hi))
    attachments=cfg['attachments']
    touch=[]
    for item in attachments:
        if set(item)-{'name','parent','xyz','rpy','geometry','touch_links'}:
            raise ValueError('Unknown attachment setting')
        name=item['name'];parent=item['parent']
        if not name or robot.find(f"link[@name='{name}']") is not None or robot.find(f"link[@name='{parent}']") is None:
            raise ValueError('Attachment needs unique name and existing parent; list parents first')
        link=ET.SubElement(robot,'link',name=name)
        for tag in ('visual','collision'):
            shape=ET.SubElement(link,tag);origin(shape,[0,0,0],[0,0,0])
            attachment_geometry(ET.SubElement(shape,'geometry'),item['geometry'])
        joint=ET.SubElement(robot,'joint',name=name+'_fixed',type='fixed')
        ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link=name)
        origin(joint,item['xyz'],item['rpy'])
        touch.extend((name,other) for other in item.get('touch_links',[]))
    # Duplicate names are invalid even for manually edited source URDFs.
    for tag in ('link','joint'):
        names=[e.get('name') for e in robot.findall(tag)]
        if len(names)!=len(set(names)):
            raise ValueError(f'Duplicate {tag} names')
    margin=scalar(cfg['polyhedra']['margin_m'])
    fitted=copy.deepcopy(robot);cache={};mesh_inputs={};part_manifest={}
    mesh_output=output_root/'piper_moveit_guard/meshes/collision'
    # Stage all expensive/validated results first. A manifest is committed last.
    with tempfile.TemporaryDirectory(prefix='prepare-',dir=output_root) as tmp:
        staging=Path(tmp)
        for link in fitted.findall('link'):
            count=0
            for ci,old in enumerate(list(link.findall('collision'))):
                geom=old.find('geometry')
                if geom is None:
                    raise ValueError('Missing collision geometry')
                mesh=geom.find('mesh')
                if mesh is not None:
                    path=resolve_mesh(mesh.get('filename'));digest=hashlib.sha256(path.read_bytes()).hexdigest()
                    mesh_inputs[str(path.relative_to(output_root)) if path.is_relative_to(output_root) else str(path)]=digest
                    scale=vector(mesh.get('scale','1 1 1').split(),positive=True)
                    spec=cfg['polyhedra']['sections'].get(link.get('name'),cfg['polyhedra']['sections'].get(path.stem))
                    key=(digest,tuple(scale),json.dumps(spec,sort_keys=True),margin)
                    if key not in cache:
                        # Cuts remain in source STL coordinates; scale is applied afterwards.
                        cache[key]=[]
                        for i,pts in enumerate(sections(mesh_triangles(path),spec),1):
                            faces,planes=enclosing_polyhedron(pts*scale,margin)
                            filename=hashlib.sha256(repr(key).encode()).hexdigest()[:20]+f'_part{i}.stl'
                            write_stl(staging/filename,faces)
                            cache[key].append((filename,planes.tolist()))
                    entries=cache[key]
                else:
                    pts=primitive_points(geom)
                    key=ET.tostring(geom)+str(margin).encode()
                    filename=hashlib.sha256(key).hexdigest()[:20]+'.stl'
                    if not (staging/filename).exists():
                        faces,planes=enclosing_polyhedron(pts,margin);write_stl(staging/filename,faces)
                    entries=[(filename,[])]
                pose=old.find('origin')
                if pose is not None:
                    vector(pose.get('xyz','0 0 0').split());vector(pose.get('rpy','0 0 0').split())
                link.remove(old)
                for filename,planes in entries:
                    count+=1
                    part=ET.SubElement(link,'collision',name=f'{link.get("name")}_part{count}')
                    if pose is not None:
                        part.append(copy.deepcopy(pose))
                    else:
                        origin(part,[0,0,0],[0,0,0])
                    ET.SubElement(ET.SubElement(part,'geometry'),'mesh',filename='package://piper_moveit_guard/meshes/collision/'+filename)
                    part_manifest[f'{link.get("name")}:{count}']={'file':filename,'support_planes_m':planes}
            if count>8:
                raise ValueError(f'{link.get("name")}: more than eight convex parts')
        srdf=ET.Element('robot',name=robot.get('name'))
        for side in ('left','right'):
            group=ET.SubElement(srdf,'group',name=side+'_arm')
            for i in range(1,7):
                ET.SubElement(group,'joint',name=f'{side}_joint{i}')
            for a,b in itertools.combinations(sorted(core[side]),2):
                ET.SubElement(srdf,'disable_collisions',link1=a,link2=b,reason='SameArmExcluded')
        both=ET.SubElement(srdf,'group',name='both_arms')
        for s in ('left','right'):
            ET.SubElement(both,'group',name=s+'_arm')
        link_names={e.get('name') for e in robot.findall('link')}
        for a,b in [*touch,*cfg['allowed_collision_pairs']]:
            if a not in link_names or b not in link_names or a==b:
                raise ValueError(f'Invalid allowed collision pair {a}, {b}')
            if (a.startswith('left_') and b.startswith('right_')) or (b.startswith('left_') and a.startswith('right_')):
                raise ValueError('Cross-arm exclusions are not supported')
            ET.SubElement(srdf,'disable_collisions',link1=a,link2=b,reason='ExplicitAttachmentContact')
        distance=cfg['joint_distance'];protection=distance['protection']
        if set(protection)!={'left','right'}:
            raise ValueError('Configure both arms protection')
        for side in ('left','right'):
            p=protection[side]
            vector(p['radii_m'],7,True)
            offsets=np.asarray(p['offsets_m'],dtype=float)
            if offsets.shape!=(7,3) or not np.isfinite(offsets).all():
                raise ValueError('Expected seven finite protection offsets per arm')
        stop=scalar(distance['stop_clearance_m'],True);resume=scalar(distance['resume_clearance_m'],True)
        if resume<=stop or not isinstance(distance['link_samples'],int) or not 0<=distance['link_samples']<=20:
            raise ValueError('Need resume > stop > 0 and 0..20 link samples')
        guard={'piper_collision_guard':{'ros__parameters':dict(enforce=False,geometry_confirmed=False,
            rate_hz=50.,stop_clearance_m=stop,resume_clearance_m=resume,link_samples=distance['link_samples'],
            feedback_timeout_s=.15,target_timeout_s=.15,max_arm_skew_s=.05,path_step_rad=.02,
            max_path_samples=100,driver_speed_ratio=10,gripper_effort_nm=1.)}}
        write_xml(staging/'dual_piper.urdf',robot);write_xml(staging/'dual_piper_meshes.urdf',fitted)
        write_xml(staging/'dual_piper.srdf',srdf)
        (staging/'protection.yaml').write_text(yaml.safe_dump(protection,sort_keys=False))
        (staging/'guard.yaml').write_text(yaml.safe_dump(guard,sort_keys=False))
        destinations={
            'dual_piper.urdf':output_root/'piper_dual_description/urdf/dual_piper.urdf',
            'dual_piper_meshes.urdf':output_root/'piper_moveit_guard/config/dual_piper_meshes.urdf',
            'dual_piper.srdf':output_root/'piper_moveit_guard/config/dual_piper.srdf',
            'protection.yaml':output_root/'piper_collision_guard/config/protection.yaml',
            'guard.yaml':output_root/'piper_collision_guard/config/guard.yaml'}
        manifest={'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
                  'config_sha256':hashlib.sha256(Path(config_path).read_bytes()).hexdigest(),
                  'margin_m':margin,'parts':part_manifest,'mesh_inputs':mesh_inputs,'files':{}}
        mesh_output.mkdir(parents=True,exist_ok=True)
        for path in staging.glob('*.stl'):
            destinations[path.name]=mesh_output/path.name
        for name,dest in destinations.items():
            dest.parent.mkdir(parents=True,exist_ok=True)
            manifest['files'][str(dest.relative_to(output_root))]=hashlib.sha256((staging/name).read_bytes()).hexdigest()
            os.replace(staging/name,dest)
        manifest['source_path']=str(source.resolve().relative_to(output_root)) if source.resolve().is_relative_to(output_root) else str(source.resolve())
        manifest['config_path']=str(Path(config_path).resolve().relative_to(output_root)) if Path(config_path).resolve().is_relative_to(output_root) else str(Path(config_path).resolve())
        (staging/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        os.replace(staging/'manifest.json',output_root/'model/manifest.json')
    print(f'Prepared {len(part_manifest)} convex parts, mounts and finger limits from {config_path}')
    return manifest


def verify(config_path):
    manifest=json.loads((ROOT/'model/manifest.json').read_text())
    checks={**{ROOT/p:h for p,h in manifest['files'].items()},
            **{ROOT/Path(p):h for p,h in manifest['mesh_inputs'].items()},
            ROOT/Path(manifest['source_path']):manifest['source_sha256'],
            Path(config_path):manifest['config_sha256']}
    for path,expected in checks.items():
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            raise ValueError(f'Model is stale or incomplete: {path}. Run prepare_model.py again.')
    print('Model configuration and generated files verified')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'config/robot.yaml')
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    try:
        verify(args.config) if args.check else prepare(args.config)
    except Exception as e:
        parser.exit(1,f'Model preparation failed: {e}\n')
