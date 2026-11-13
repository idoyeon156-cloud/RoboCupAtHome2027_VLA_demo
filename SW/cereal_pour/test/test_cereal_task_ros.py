"""Isolated ROS tests; no hardware driver and no final command publisher."""
import os
from pathlib import Path
import time
import numpy as np
import pytest
import yaml

pytestmark=pytest.mark.skipif(os.environ.get('RUN_SENSOR_MOUNT_ROS_TESTS')!='1',reason='isolated ROS tests')
ROOT=Path(__file__).resolve().parents[1]


def test_best_effort_feedback_full_prepare_and_dropout(tmp_path):
    """Virtual encoders through both actual gates; never on the hardware domain."""
    assert os.environ['ROS_DOMAIN_ID']=='181'
    import json
    import signal
    import subprocess
    import sys
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    rclpy.init();node=rclpy.create_node('virtual_encoder_prepare_test')
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    status_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    positions={s:np.r_[np.deg2rad([0,20,0,0,-15,0]),.01].tolist()
               for s in ('left','right')};statuses={};subs=[]
    pubs={s:node.create_publisher(JointState,f'/joint_states_{s}',qos) for s in positions}
    for side in positions:
        subs.append(node.create_subscription(JointState,f'/joint_ctrl_cmd_{side}',
                    lambda m,s=side:positions.update({s:list(m.position)}),1))
    for name in ('collision_guard','moveit_guard','cereal_pour'):
        subs.append(node.create_subscription(String,f'/{name}/status',
                    lambda m,n=name:statuses.update({n:json.loads(m.data)}),status_qos))
    def pump(duration, feedback=True):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            if feedback:
                for side,pub in pubs.items():
                    msg=JointState();msg.header.stamp=node.get_clock().now().to_msg()
                    msg.name=[f'joint{i}' for i in range(1,7)]+['gripper'];msg.position=list(positions[side])
                    if side=='left' and abs(msg.position[2])<1e-9:
                        msg.position[2]=float(np.deg2rad(.025986157))
                    if side=='right' and abs(msg.position[6])<1e-9:
                        msg.position[6]=-.0002  # Observed closed-gripper encoder offset.
                    if abs(msg.position[4]-np.deg2rad(-70.))<1e-9:
                        msg.position[4]=float(np.deg2rad(-70.087))
                    pub.publish(msg)
            rclpy.spin_once(node,timeout_sec=.001)
            time.sleep(.004)
    def control(action):
        with (tmp_path/(action+'.log')).open('w') as log:
            command=subprocess.Popen(['bash',str(ROOT/'run_real_control.sh'),action],stdout=log,stderr=log)
            deadline=time.monotonic()+(180 if action in ('start','run') else 45)
            while command.poll() is None and time.monotonic()<deadline:pump(.01)
            if command.poll() is None:
                command.kill();command.wait()
            assert command.returncode==0,(tmp_path/(action+'.log')).read_text()
    with (tmp_path/'launch.log').open('w') as log:
        proc=subprocess.Popen([sys.executable,'/opt/ros/humble/bin/ros2','launch','piper_moveit_guard',
              'two_stage_guard.launch.py','cereal:=true','enforce:=true','geometry_confirmed:=true',
              f'sequence_path:={ROOT}/model/cereal_task_sequence.yaml'],stdout=log,stderr=log,start_new_session=True)
        try:
            deadline=time.monotonic()+15
            while len(statuses)<3 and time.monotonic()<deadline:pump(.05)
            assert len(statuses)==3,(tmp_path/'launch.log').read_text()
            pump(.5)
            control('ready')
            control('home')
            assert statuses['cereal_pour']['detail']=='Finished; holding home',statuses
            for q in positions.values():
                np.testing.assert_allclose(q,[0.]*7,atol=.002)
            control('run')
            assert statuses['cereal_pour']['detail']=='Finished; holding return_home',statuses
            for q in positions.values():
                np.testing.assert_allclose(q,[0.]*7,atol=.002)
            pump(.5,feedback=False)
            assert statuses['collision_guard']['latched'],statuses
            assert statuses['moveit_guard']['latched'],statuses
            assert statuses['cereal_pour']['state']=='stopped',statuses
        finally:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=6)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            node.destroy_node();rclpy.shutdown()


def test_real_planned_prepare_start_and_guard_pause(monkeypatch):
    assert os.environ['ROS_DOMAIN_ID']=='181'
    import rclpy
    from std_srvs.srv import Trigger
    import piper_collision_guard.cereal_pour as module
    sequence=ROOT/'model/cereal_task_sequence.yaml'
    rclpy.init(args=['--ros-args','-p',f'sequence_path:={sequence}'])
    node=module.CerealPour()
    try:
        assert node.planned
        response=node.prepare(None,Trigger.Response())
        assert not response.success and 'feedback' in response.message
        clock=[time.monotonic()]
        monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
        home=node.planned_steps[0][1].copy()
        node.inputs=lambda now: node.command.copy() if node.command is not None else home.copy()
        node.guard_status={key:({'blocked':False,'latched':False},clock[0]) for key in ('collision_guard','moveit_guard')}
        assert not node.start(None,Trigger.Response()).success
        assert node.prepare(None,Trigger.Response()).success
        initial=node.command.copy()
        node.guard_status['moveit_guard'][0]['blocked']=True
        clock[0]+=.02;node.tick()
        np.testing.assert_array_equal(initial,node.command)
        node.guard_status['moveit_guard'][0]['blocked']=False
        def finish():
            visited=set()
            for _ in range(18000):
                clock[0]+=.02;node.tick();visited.add(node.detail)
                if node.state!='running':break
            assert node.state=='complete',node.detail
            return visited
        finish()
        np.testing.assert_allclose(node.command[:,:6],np.tile(np.deg2rad([0,90,0,0,-70,0]),(2,1)),atol=1e-8)
        assert node.start(None,Trigger.Response()).success
        visited=finish()
        assert {'bowl_pregrasp','box_pregrasp','pour_65','return_alpha'}.issubset(visited)
        np.testing.assert_allclose(node.command,home,atol=1e-8)
        # A single service request traverses alpha without a separate start.
        assert node.run(None,Trigger.Response()).success
        assert node.steps[0][0]=='alpha' and node.steps[-1][0]=='return_home'
        visited=finish()
        assert {'alpha','bowl_pregrasp','pour_65','return_alpha'}.issubset(visited)
        np.testing.assert_allclose(node.command,home,atol=1e-8)
        node.command=None
        node.inputs=lambda now: node.poses['ready'].copy()
        assert not node.run(None,Trigger.Response()).success
        assert node.count_publishers('/joint_ctrl_cmd_left')==0
    finally:
        node.destroy_node();rclpy.shutdown()


def test_home_uses_feedback_gripper_stages_and_guards(monkeypatch):
    assert os.environ['ROS_DOMAIN_ID']=='181'
    import rclpy
    from std_srvs.srv import Trigger
    import piper_collision_guard.cereal_pour as module
    rclpy.init(args=['--ros-args','-p',f'sequence_path:={ROOT}/model/cereal_task_sequence.yaml'])
    node=module.CerealPour()
    try:
        assert not node.home(None,Trigger.Response()).success
        clock=[time.monotonic()]
        monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
        measured=np.tile(np.r_[np.deg2rad([0,20,0,0,-15,0]),.01],(2,1))
        node.inputs=lambda now: node.command.copy() if node.command is not None else measured.copy()
        node.guard_status={key:({'blocked':False,'latched':False},clock[0])
                           for key in ('collision_guard','moveit_guard')}
        assert not node.prepare(None,Trigger.Response()).success
        assert node.home(None,Trigger.Response()).success
        np.testing.assert_array_equal(node.command,measured)
        assert not node.home(None,Trigger.Response()).success
        assert np.all(node.active_rates[:6]<=np.deg2rad(5.))
        for _ in range(2):
            node.guard_status['moveit_guard'][0]['blocked']=True
            previous=node.command.copy();clock[0]+=.02;node.tick()
            np.testing.assert_array_equal(node.command,previous)
        node.guard_status['moveit_guard'][0]['blocked']=False
        for _ in range(500):
            previous=node.command.copy();clock[0]+=.02;node.tick()
            assert np.all(np.abs(node.command-previous)<=node.active_rates*.020001)
            if node.detail=='home_joints':
                np.testing.assert_array_equal(node.command[:,6],measured[:,6])
            if node.state=='complete':break
        assert node.detail=='Finished; holding home'
        np.testing.assert_allclose(node.command,node.planned_steps[0][1])
        assert node.prepare(None,Trigger.Response()).success
        np.testing.assert_array_equal(node.active_rates,node.rates)
        def lost_feedback(now):
            raise ValueError('Both arms need fresh joint feedback')
        node.inputs=lost_feedback
        clock[0]+=.02;node.tick()
        assert node.state=='stopped' and node.command is None
        assert node.count_publishers('/joint_ctrl_cmd_left')==0
    finally:
        node.destroy_node();rclpy.shutdown()


def test_rviz_all_segments_publish_objects_and_joints():
    assert os.environ['ROS_DOMAIN_ID']=='181'
    import rclpy
    from piper_collision_guard.cereal_preview import CerealPreview
    from visualization_msgs.msg import MarkerArray
    from sensor_msgs.msg import JointState
    sequence=ROOT/'model/cereal_task_sequence.yaml'
    rclpy.init(args=['--ros-args','-p',f'sequence_path:={sequence}'])
    node=CerealPreview();observer=rclpy.create_node('cereal_marker_test');markers=[];joints=[]
    observer.create_subscription(MarkerArray,'/cereal_task/objects',markers.append,10)
    observer.create_subscription(JointState,'joint_states',joints.append,10)
    try:
        deadline=time.monotonic()+3
        while (not markers or not joints) and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02);rclpy.spin_once(observer,timeout_sec=.02)
        assert markers and joints
        assert len(markers[-1].markers)==5 and len(joints[-1].position)==16
        for a,b,d,name,held in node.timeline.segments:
            node.publish_objects((a+b)/2,name,held)
            node.publish_joints((a+b)/2)
        assert node.count_publishers('/joint_ctrl_cmd_left')==0
    finally:
        observer.destroy_node();node.destroy_node();rclpy.shutdown()


def test_whole_loop_in_actual_moveit_backend(tmp_path):
    assert os.environ['ROS_DOMAIN_ID']=='181'
    import signal
    import subprocess
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from moveit_msgs.srv import GetStateValidity
    from piper_collision_guard.collision_model import load_model
    from piper_collision_guard.cereal_timeline import Timeline
    config=yaml.safe_load((ROOT/'model/cereal_task_sequence.yaml').read_text())
    description,_,_=load_model(ROOT/'piper_moveit_guard')
    params={'robot_description':description,'robot_description_semantic':(ROOT/'piper_moveit_guard/config/dual_piper.srdf').read_text(),
            'convex_meshes':True,'demo':True}
    path=tmp_path/'params.yaml';path.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
    with (tmp_path/'moveit.log').open('w') as log:
        proc=subprocess.Popen([get_package_prefix('piper_moveit_guard')+'/lib/piper_moveit_guard/moveit_guard','--ros-args','--params-file',str(path)],stdout=log,stderr=log)
        rclpy.init();node=rclpy.create_node('cereal_moveit_test')
        try:
            client=node.create_client(GetStateValidity,'/moveit_guard/check_state_validity')
            assert client.wait_for_service(timeout_sec=15),(tmp_path/'moveit.log').read_text()
            for a,b,d,name,held in Timeline(config).segments:
                for t in np.linspace(0,1,5):
                    q=a+(b-a)*t;req=GetStateValidity.Request()
                    req.robot_state.joint_state.name=[f'{side}_joint{i}' for side in ('left','right') for i in range(1,9)]
                    req.robot_state.joint_state.position=np.column_stack((q[:,:6],q[:,6]/2,-q[:,6]/2)).flatten().tolist()
                    future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
                    assert future.done() and future.result().valid,(name,t,[(c.contact_body_1,c.contact_body_2) for c in future.result().contacts])
        finally:
            node.destroy_node();rclpy.shutdown();proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()


def test_real_launch_connects_both_guards_and_planned_replay(tmp_path):
    assert os.environ['ROS_DOMAIN_ID']=='181'
    import json
    import signal
    import subprocess
    import sys
    import rclpy
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,DurabilityPolicy
    rclpy.init();node=rclpy.create_node('cereal_real_launch_test');statuses={}
    for name in ('collision_guard','moveit_guard','cereal_pour'):
        node.create_subscription(String,f'/{name}/status',lambda m,n=name:statuses.update({n:json.loads(m.data)}),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    with (tmp_path/'launch.log').open('w') as log:
        proc=subprocess.Popen([sys.executable,'/opt/ros/humble/bin/ros2','launch','piper_moveit_guard','two_stage_guard.launch.py',
                               'cereal:=true',f'sequence_path:={ROOT}/model/cereal_task_sequence.yaml'],stdout=log,stderr=log,start_new_session=True)
        try:
            deadline=time.monotonic()+15
            while len(statuses)<3 and proc.poll() is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
            assert len(statuses)==3,(tmp_path/'launch.log').read_text()
            assert statuses['collision_guard']['mode']=='monitor'
            assert statuses['moveit_guard']['mode']=='monitor'
            assert statuses['moveit_guard']['geometry']=='convex_meshes'
            assert statuses['cereal_pour']['state']=='idle'
            assert node.count_publishers('/joint_ctrl_cmd_left')==0
        finally:
            node.destroy_node();rclpy.shutdown();os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=6)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()


def test_python_feedback_normalization_and_strict_targets():
    assert os.environ['ROS_DOMAIN_ID']=='181'
    import rclpy
    from sensor_msgs.msg import JointState
    from piper_collision_guard.node import CollisionGuard
    from piper_collision_guard.cereal_pour import CerealPour
    rclpy.init(args=['--ros-args','-p',f'sequence_path:={ROOT}/model/cereal_task_sequence.yaml'])
    guard=CollisionGuard();replay=CerealPour()
    try:
        msg=JointState();msg.header.stamp=guard.get_clock().now().to_msg()
        msg.name=[f'joint{i}' for i in range(1,7)]+['gripper']
        msg.position=[0.,0.,np.deg2rad(.025986157),0.,0.,0.,0.]
        guard.receive(msg,'left',False);replay.receive_feedback('left',msg)
        assert guard.feedback['left'][0][2]==0
        assert replay.feedback['left'][0][2]==0
        guard.receive(msg,'left',True)
        assert 'left' not in guard.targets and guard.lock.latched
        msg.position[2]=np.deg2rad(.10);msg.header.stamp=guard.get_clock().now().to_msg()
        guard.receive(msg,'left',False);replay.receive_feedback('left',msg)
        assert 'left' not in guard.feedback and 'left' not in replay.feedback
        msg.position=[0.,np.deg2rad(90),0.,0.,np.deg2rad(-70.087),0.,0.]
        msg.header.stamp=guard.get_clock().now().to_msg()
        guard.receive(msg,'left',False);replay.receive_feedback('left',msg)
        assert guard.feedback['left'][0][4]==pytest.approx(np.deg2rad(-70))
        assert replay.feedback['left'][0][4]==pytest.approx(np.deg2rad(-70))
        guard.receive(msg,'left',True)
        assert 'left' not in guard.targets
        msg.position[4]=np.deg2rad(-70.151)
        guard.receive(msg,'left',False);replay.receive_feedback('left',msg)
        assert 'left' not in guard.feedback and 'left' not in replay.feedback
    finally:
        replay.destroy_node();guard.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize('rejection', ['gripper_target', 'j5_target', 'j5_feedback', 'j3_target'])
def test_moveit_accepts_only_tiny_measured_boundary_noise(tmp_path, rejection):
    assert os.environ['ROS_DOMAIN_ID']=='181'
    import json,signal,subprocess
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from ament_index_python.packages import get_package_prefix
    from piper_collision_guard.collision_model import load_model
    description,_,_=load_model(ROOT/'piper_moveit_guard')
    params={'robot_description':description,'robot_description_semantic':(ROOT/'piper_moveit_guard/config/dual_piper.srdf').read_text(),
            'convex_meshes':True,'enforce':False,'input_timeout_s':.5,'max_arm_skew_s':.1}
    path=tmp_path/'params.yaml';path.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
    with (tmp_path/'moveit.log').open('w') as log:
        proc=subprocess.Popen([get_package_prefix('piper_moveit_guard')+'/lib/piper_moveit_guard/moveit_guard','--ros-args','--params-file',str(path)],stdout=log,stderr=log)
        rclpy.init();node=rclpy.create_node('test_moveit_encoder_noise');statuses=[]
        node.create_subscription(String,'/moveit_guard/status',lambda m:statuses.append(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        feedback={s:node.create_publisher(JointState,'/joint_states_'+s,1) for s in ('left','right')}
        targets={s:node.create_publisher(JointState,'/teleop/'+s+'_target',1) for s in ('left','right')}
        def run_for(seconds,noise,send_target=False,opening=0.,j5=0.):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                msg=JointState();msg.header.stamp=node.get_clock().now().to_msg();msg.name=[f'joint{i}' for i in range(1,7)]+['gripper']
                msg.position=[0.,float(np.deg2rad(90 if j5 else 0)),float(np.deg2rad(noise)),0.,float(np.deg2rad(j5)),0.,opening]
                for pub in feedback.values():pub.publish(msg)
                if send_target:
                    for pub in targets.values():pub.publish(msg)
                rclpy.spin_once(node,timeout_sec=.01)
        try:
            run_for(1.5,.025986157)
            assert statuses,(tmp_path/'moveit.log').read_text()
            assert not statuses[-1]['latched'],statuses[-1]
            run_for(.15,0.,j5=-70.087)
            assert not statuses[-1]['latched'],statuses[-1]
            run_for(.15,0.,opening=-.0002)
            assert not statuses[-1]['latched'],statuses[-1]
            if rejection=='gripper_target':
                run_for(.15,0.,True,opening=-.0002)
                expected='invalid_target: gripper_bounds'
            elif rejection=='j5_target':
                run_for(.15,0.,True,j5=-70.087)
                expected='invalid_target: joint_bounds'
            elif rejection=='j5_feedback':
                run_for(.15,0.,j5=-70.151)
                expected='invalid_feedback: joint_bounds'
            else:
                run_for(.15,.025986157,True)
                expected='invalid_target: joint_bounds'
            assert statuses[-1]['latched'] and statuses[-1]['first_stop_reason'].startswith(expected),statuses[-1]
        finally:
            node.destroy_node();rclpy.shutdown();proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
