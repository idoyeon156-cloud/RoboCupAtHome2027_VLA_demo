"""Offline IK + collision-checked joint-space planning for RViz and gated real-robot cereal replay."""
import copy
import hashlib
import itertools
import json
from pathlib import Path
import time

import hppfcl as fcl
import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

SIDES=('left','right')


def vec(value,n=3):
    a=np.asarray(value,dtype=float)
    if a.shape!=(n,) or not np.isfinite(a).all():raise ValueError(f'Expected {n} finite numbers')
    return a


def transform(xyz,rpy_deg):
    a=np.eye(4);a[:3,:3]=Rotation.from_euler('xyz',vec(rpy_deg),degrees=True).as_matrix();a[:3,3]=vec(xyz);return a


def object_definitions(spec):
    objects={}
    for name,side in [('bowl','left'),('box','right')]:
        item=spec[name]
        pose=transform(item['bottom_center_world_m'],item.get('rpy_deg',[0,0,0]))
        size=vec(item['size_m']) if name=='box' else np.array([float(item['diameter_m'])]*2+[float(item['height_m'])])
        if np.any(size<=0):raise ValueError(f'{name}: dimensions must be positive')
        offset=vec(item['grasp_offset_world_m'])
        if offset[2]<0 or offset[2]>size[2]+1e-6:raise ValueError(f'{name}: grasp height exceeds object height')
        rotation=pose[:3,:3]@Rotation.from_euler('xyz',vec(item['gripper_rpy_in_object_deg']),degrees=True).as_matrix()
        objects[name]={'side':side,'pose':pose,'size':size,'grasp_world_m':pose[:3,3]+offset,'gripper_rotation':rotation,
                       'closed_opening_m':float(item['gripper_closed_opening_mm'])/1000.,
                       'open_opening_m':float(item['gripper_open_opening_mm'])/1000.}
        if not 0<=objects[name]['closed_opening_m']<objects[name]['open_opening_m']<=.1:raise ValueError(f'{name}: invalid openings')
    return objects


class Scene:
    def __init__(self,root,spec):
        from .distance import DistanceModel
        self.root=Path(root);self.spec=spec;self.objects=object_definitions(spec)
        self.tcp=vec(spec['tool']['tcp_in_gripper_base_m'])
        protection=yaml.safe_load((self.root/'piper_collision_guard/config/protection.yaml').read_text())
        self.fk=DistanceModel(self.root/'piper_moveit_guard/config/dual_piper_meshes.urdf',0,protection)
        self.model=self.fk.model;self.data=self.model.createData()
        self.frames={s:self.model.getFrameId(s+'_gripper_base') for s in SIDES}
        self.geometry=pin.buildGeomFromUrdf(self.model,str(self.root/'piper_moveit_guard/config/dual_piper_meshes.urdf'),pin.GeometryType.COLLISION,package_dirs=[str(self.root)])
        self.gdata=self.geometry.createData();self.shapes=[];self.groups={s:[] for s in (*SIDES,'environment')}
        for i,o in enumerate(self.geometry.geometryObjects):
            o.geometry.buildConvexHull(True,'Qt');shape=o.geometry.convex;shape.computeLocalAABB();self.shapes.append(shape)
            side=o.name.split('_',1)[0];self.groups[side if side in SIDES else 'environment'].append(i)
        self.pairs=[*itertools.product(self.groups['left'],self.groups['right']),
                    *itertools.product(self.groups['left']+self.groups['right'],self.groups['environment'])]
        self.obj_shapes={}
        for name,item in self.objects.items():
            size=item['size'];self.obj_shapes[name]=fcl.Box(*size) if name=='box' else fcl.Cylinder(size[0]/2,size[2])
            self.obj_shapes[name].computeLocalAABB()
        self.backend=None;self.attachments={};self.last_error='';self.calls=0
        self.stop=float(yaml.safe_load((self.root/'config/robot.yaml').read_text())['joint_distance']['stop_clearance_m'])
        self.request=fcl.CollisionRequest();self.result=fcl.CollisionResult()
        self.check_distance=bool(spec['planning'].get('check_joint_distance',True))
        if float(spec['planning']['collision_step_deg'])<=0:raise ValueError('Positive collision step required')

    def state_q(self,state):
        q=self.fk.configuration(state[0,:6],state[1,:6])
        for k,s in enumerate(SIDES):
            if not 0<=state[k,6]<=self.fk.max_opening[s]:raise ValueError('Opening exceeds model range')
            for j,sign in ((7,1),(8,-1)):
                q[self.model.joints[self.model.getJointId(f'{s}_joint{j}')].idx_q]=sign*state[k,6]/2
        return q

    def update(self,state):
        q=self.state_q(state);pin.framesForwardKinematics(self.model,self.data,q)
        pin.updateGeometryPlacements(self.model,self.data,self.geometry,self.gdata,q)

    def object_pose(self,name,held):
        item=self.objects[name]
        if name in held:
            frame=self.data.oMf[self.frames[item['side']]]
            return frame.homogeneous@self.attachments[name]
        return item['pose'].copy()

    def attach(self,name,state):
        self.update(state);frame=self.data.oMf[self.frames[self.objects[name]['side']]]
        self.attachments[name]=np.linalg.inv(frame.homogeneous)@self.objects[name]['pose']

    @staticmethod
    def placed(shape,pose):
        # World AABB rejection makes the large number of convex pairs inexpensive.
        rot=pose[:3,:3];centre=rot@shape.aabb_local.center()+pose[:3,3]
        half=np.abs(rot)@((shape.aabb_local.max_-shape.aabb_local.min_)/2)
        return fcl.Transform3f(rot,pose[:3,3]),centre-half,centre+half

    def intersects(self,a,pa,b,pb):
        if np.any(pa[2]<pb[1]) or np.any(pb[2]<pa[1]):return False
        self.result.clear();return fcl.collide(a,pa[0],b,pb[0],self.request,self.result)>0

    def valid(self,state,held=()):
        self.calls+=1
        try:
            if self.check_distance:
                gap=self.fk.clearance(state[0,:6],state[1,:6])
                if gap.distance<=self.stop:
                    self.last_error=f'distance {gap.left_name} / {gap.right_name}: {gap.distance*1000:.1f} mm';return False
            self.update(state)
        except ValueError as e:self.last_error=str(e);return False
        placed=[self.placed(s,p.homogeneous) for s,p in zip(self.shapes,self.gdata.oMg)]
        for i,j in self.pairs:
            if self.intersects(self.shapes[i],placed[i],self.shapes[j],placed[j]):
                self.last_error=f'{self.geometry.geometryObjects[i].name} / {self.geometry.geometryObjects[j].name}';return False
        obj_placed={}
        for name,item in self.objects.items():
            pose=self.object_pose(name,held);centre=pose.copy();centre[:3,3]+=pose[:3,:3]@np.array([0,0,item['size'][2]/2])
            obj_placed[name]=self.placed(self.obj_shapes[name],centre)
            # Only the intended jaw contact is exempted. Own wrist/body still checked.
            touch=(item['side']+'_link7_',item['side']+'_link8_')
            for i,robot in enumerate(self.geometry.geometryObjects):
                if robot.name.startswith(touch):continue
                if self.intersects(self.obj_shapes[name],obj_placed[name],self.shapes[i],placed[i]):
                    self.last_error=f'{name} / {robot.name}';return False
        if self.intersects(self.obj_shapes['bowl'],obj_placed['bowl'],self.obj_shapes['box'],obj_placed['box']):
            self.last_error='bowl / box';return False
        return True

    def edge(self,a,b,held=(),step_deg=None):
        step=np.deg2rad(step_deg or self.spec['planning']['collision_step_deg'])
        n=max(1,int(np.ceil(max(np.max(np.abs(b[:,:6]-a[:,:6]))/step,np.max(np.abs(b[:,6]-a[:,6]))/.002))))
        for t in np.linspace(0,1,n+1):
            state=a+(b-a)*t
            if not self.valid(state,held):return False
            if self.backend is None:raise ValueError('MoveIt backend is required for every executable edge')
            if not self.backend.check_state(state,held):return False
        return True


class Planner:
    def __init__(self,scene):
        self.scene=scene;self.spec=scene.spec;self.rng=np.random.default_rng(int(self.spec['planning']['random_seed']))
        self.state=np.zeros((2,7));self.held=[];self.waypoints=[]
        self.add('home',self.state,hold=float(self.spec['motion']['home_hold_s']))
        self.log=[]

    def add(self,name,state,hold=0):
        self.state=np.asarray(state).copy()
        self.waypoints.append(dict(name=name,left_deg=np.rad2deg(state[0,:6]).tolist(),right_deg=np.rad2deg(state[1,:6]).tolist(),
                                   opening_mm=(state[:,6]*1000).tolist(),held_objects=list(self.held),hold_s=float(hold)))

    def route(self,goal,active):
        start=self.state.copy();scene=self.scene;held=tuple(self.held)
        if scene.backend is None:raise ValueError('MoveIt planner required; custom RRT fallback is disabled')
        path=scene.backend.plan(start,goal,active,held)
        for left,right in zip(path,path[1:]):
            if not scene.edge(left,right,held):
                raise ValueError(f'MoveIt route rejected by execution constraints: {scene.last_error}')
        return path

    def move(self,name,goal,active):
        print(f'{name}: MoveIt planning...',flush=True)
        started=time.monotonic();path=self.route(goal,active)
        # OMPL emits interpolation samples along straight joint edges. Keep
        # corners, but avoid stop/settle at every collinear sampling point.
        compressed=[]
        for state in path:
            compressed.append(state)
            while len(compressed)>=3:
                a,b,c=compressed[-3:]
                edge=(c-a).ravel();middle=(b-a).ravel();length=float(edge@edge)
                if length<=1e-20:break
                t=float(middle@edge/length)
                if not 0.<t<1.:break
                deviation = np.abs(middle-t*edge).reshape(a.shape)
                exact = np.max(deviation) <= 1e-10
                near = (self.spec.get('execution', {}).get('arms_only', False)
                        and np.max(deviation[:, :6]) <= np.deg2rad(.05)
                        and np.max(deviation[:, 6]) <= 1e-10)
                if not exact:
                    # Numerical OMPL bends can create a full stop just before
                    # alpha. NEVER bypass a non-collinear corner without
                    # rechecking the replacement edge with MoveIt/objects.
                    if not near or not self.scene.edge(a,c,self.held):break
                compressed.pop(-2)
        path=compressed
        for i,state in enumerate(path[1:]):self.add(name if i==len(path)-2 else f'{name}_via_{i+1}',state)
        self.log.append(dict(stage=name,waypoints=len(path),seconds=round(time.monotonic()-started,3)))
        print(f'{name}: {len(path)-1} segments',flush=True)

    def opening(self,name,side,value):
        if self.spec.get('execution', {}).get('arms_only', False):
            value = 0.  # Replan with CLOSED jaws; never merely ignore tracking.
        goal=self.state.copy();goal[SIDES.index(side),6]=value
        if not self.scene.edge(self.state,goal,self.held):raise ValueError(f'{name}: {self.scene.last_error}')
        self.add(name,goal)

    def ik(self,side,position,rotation,seed):
        scene=self.scene;ids=scene.fk.slots[side];lo=scene.model.lowerPositionLimit[ids];hi=scene.model.upperPositionLimit[ids]
        data=scene.model.createData();neutral=pin.neutral(scene.model);frame_id=scene.frames[side]
        def residual(x):
            q=neutral.copy();q[ids]=x;pin.framesForwardKinematics(scene.model,data,q);frame=data.oMf[frame_id]
            return np.r_[frame.act(scene.tcp)-position,.15*pin.log3(rotation.T@frame.rotation)]
        guesses=[seed,*self.rng.uniform(lo,hi,(int(self.spec['planning']['ik_restarts']),6))]
        found=[]
        for guess in guesses:
            result=least_squares(residual,np.clip(guess,lo+1e-9,hi-1e-9),bounds=(lo,hi),max_nfev=180,gtol=1e-10)
            if np.linalg.norm(result.fun)<1e-5 and not any(np.linalg.norm(result.x-x)<.01 for x in found):
                found.append(result.x)
                if len(found)>=8:break
        # Preserve the configured grasp pose, but prefer an IK branch that
        # rotates J4/J6 less. The old equal-weight nearest solution could
        # trade a large wrist turn for a small shoulder/elbow improvement.
        wrist_weight=float(self.spec['planning'].get('wrist_ik_weight',3.))
        if not np.isfinite(wrist_weight) or wrist_weight<1.:
            raise ValueError('wrist_ik_weight must be finite and >=1')
        weights=np.array([1.,1.,1.,wrist_weight,1.,wrist_weight])
        return sorted(found,key=lambda x:np.linalg.norm(weights*(x-seed)))

    def target(self,name,side,position,rotation):
        i=SIDES.index(side);solutions=self.ik(side,vec(position),rotation,self.state[i,:6]);errors=[]
        for q in solutions:
            goal=self.state.copy();goal[i,:6]=q
            if not self.scene.valid(goal,self.held):errors.append(self.scene.last_error);continue
            try:
                wrist_delta=np.rad2deg(q[[3,5]]-self.state[i,[3,5]])
                self.move(name,goal,(side,))
                self.log[-1].update(ik_candidates=len(solutions), wrist_delta_j4_j6_deg=wrist_delta.tolist())
                return
            except ValueError as e:errors.append(str(e))
        raise ValueError(f'{name}: cannot reach {np.round(position,4).tolist()}; '+('; '.join(errors) if errors else 'no in-bounds IK solution'))

    def pickup(self,name):
        scene=self.scene;item=scene.objects[name];side=item['side'];i=SIDES.index(side)
        self.opening(f'{name}_open',side,item['open_opening_m'])
        retreat=float(self.spec['motion']['approach_distance_m'])
        pre=item['grasp_world_m']-item['gripper_rotation'][:,2]*retreat
        self.target(f'{name}_pregrasp',side,pre,item['gripper_rotation'])
        # Small Cartesian approach steps retain the IK branch and are collision checked.
        n=max(1,int(np.ceil(retreat/float(self.spec['motion']['cartesian_step_m']))))
        for j in range(1,n+1):self.target(f'{name}_approach_{j}',side,pre+(item['grasp_world_m']-pre)*j/n,item['gripper_rotation'])
        self.opening(f'{name}_close',side,item['closed_opening_m'])
        scene.attach(name,self.state);self.held.append(name);self.add(f'{name}_attached',self.state,hold=self.spec['motion']['grasp_hold_s'])
        self.target(f'{name}_lift',side,item['grasp_world_m']+np.array([0,0,self.spec['motion']['lift_m']]),item['gripper_rotation'])

    def generate(self):
        scene=self.scene
        if not scene.valid(self.state):raise ValueError(f'Initial pose collision: {scene.last_error}')
        alpha=self.state.copy()
        for i,s in enumerate(SIDES):alpha[i,:6]=np.deg2rad(vec(self.spec['alpha'].get(s+'_joint_deg',self.spec['alpha']['joint_deg']),6))
        self.move('alpha',alpha,SIDES);self.waypoints[-1]['hold_s']=self.spec['motion']['alpha_hold_s']
        self.pickup('bowl')
        bowl=scene.objects['bowl'];pour_bottom=vec(self.spec['pour']['bowl_bottom_center_world_m'])
        self.target('bowl_hold','left',bowl['grasp_world_m']+pour_bottom-bowl['pose'][:3,3],bowl['gripper_rotation'])
        self.pickup('box')
        box=scene.objects['box'];mouth=vec(self.spec['box']['outlet_in_object_m']);grasp=vec(self.spec['box']['grasp_offset_world_m'])
        # Tool-to-object transform from the actual IK grasp; do not assume grasp yaw.
        attached=scene.attachments['box'];tool_obj=attached[:3,:3]
        offsets=self.spec['pour']['outlet_offsets_world_m'];angles=self.spec['pour']['tilt_deg']
        if len(angles)!=len(offsets) or len(angles)<1:raise ValueError('One outlet offset is required for each tilt angle')
        top=pour_bottom+np.array([0,0,bowl['size'][2]])
        axis=vec(self.spec['pour']['tilt_axis_world']);axis/=np.linalg.norm(axis)
        for angle,offset in zip(angles,offsets):
            object_rot=Rotation.from_rotvec(axis*np.deg2rad(float(angle))).as_matrix()@box['pose'][:3,:3]
            gripper_rot=object_rot@tool_obj.T
            desired_mouth=top+vec(offset)
            bottom=desired_mouth-object_rot@mouth
            # gripper base = object bottom - R_gripper * attachment translation
            base=bottom-gripper_rot@attached[:3,3]
            self.target(f'pour_{float(angle):g}','right',base+gripper_rot@scene.tcp,gripper_rot)
        self.waypoints[-1]['hold_s']=float(self.spec['motion']['pour_hold_s'])
        forward=copy.deepcopy(self.waypoints)
        for wp in reversed(forward[:-1]):
            state=np.stack([np.r_[np.deg2rad(wp['left_deg']),wp['opening_mm'][0]/1000],np.r_[np.deg2rad(wp['right_deg']),wp['opening_mm'][1]/1000]])
            # Reverse the verified route, including release at the original pickup position.
            if not scene.edge(self.state,state,self.held):raise ValueError(f'Return to {wp["name"]}: {scene.last_error}')
            self.held=list(wp['held_objects']);self.add('return_'+wp['name'],state,hold=wp['hold_s'] if wp['name']=='alpha' else 0)
        return dict(version=2,coordinate_frame='world',object_reference='bottom_center',settings=copy.deepcopy(self.spec),
                    object_attachments={n:t.tolist() for n,t in scene.attachments.items()},waypoints=self.waypoints,
                    planning_log=self.log,collision_checks=scene.calls)


def generate(root,config_path,arms_only=False):
    from .moveit_backend import MoveItBackend
    spec=yaml.safe_load(Path(config_path).read_text())
    spec['execution']={'arms_only': bool(arms_only), 'fixed_opening_mm': [0., 0.] if arms_only else None}
    scene=Scene(root,spec)
    with MoveItBackend(scene) as backend:
        scene.backend=backend
        result=Planner(scene).generate()
    result['planner']={'backend':'moveit_ompl','collision':'MoveIt Bullet + Pinocchio distance + task objects',
                       'ik':'Pinocchio FK + scipy least_squares', 'fallback':False}
    root=Path(root).resolve()
    inputs=[Path(config_path).resolve(),root/'model/manifest.json',root/'piper_collision_guard/piper_collision_guard/cereal_planning.py',
            root/'piper_collision_guard/piper_collision_guard/moveit_backend.py',
            root/'piper_collision_guard/piper_collision_guard/collision_model.py',
            root/'piper_collision_guard/piper_collision_guard/distance.py',
            root/'piper_collision_guard/piper_collision_guard/segment_motion.py',
            root/'piper_moveit_guard/src/cereal_moveit_planner.cpp',
            root/'piper_moveit_guard/config/dual_piper.srdf',
            root/'piper_collision_guard/config/protection.yaml']
    result['input_sha256']={str(p.relative_to(root)) if p.is_relative_to(root) else str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    return result
