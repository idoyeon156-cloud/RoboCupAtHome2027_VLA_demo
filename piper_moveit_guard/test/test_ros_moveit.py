"""Real MoveIt/FCL and ROS tests in an isolated domain; no hardware driver."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest
import yaml

pytestmark = pytest.mark.skipif(os.environ.get('RUN_MOVEIT_GUARD_TESTS') != '1',
                                reason='opt-in isolated ROS tests')
ROOT = Path(__file__).resolve().parents[2]


def test_moveit_collision_service_and_command_interlock(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') == '179'
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from moveit_msgs.srv import GetStateValidity
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from ament_index_python.packages import get_package_prefix

    share=ROOT/'piper_moveit_guard/config'
    params={'robot_description': (share/'dual_piper_boxes.urdf').read_text(),
            'robot_description_semantic': (share/'dual_piper.srdf').read_text(),
            'enforce': True, 'geometry_confirmed': True,
            'input_timeout_s': .5, 'max_arm_skew_s': .2,
            'calculation_budget_s': .3, 'max_path_samples': 10000}
    config=tmp_path/'params.yaml';config.write_text(yaml.safe_dump({'/**': {'ros__parameters':params}}))
    log=(tmp_path/'node.log').open('w')
    proc=subprocess.Popen([get_package_prefix('piper_moveit_guard')+'/lib/piper_moveit_guard/moveit_guard',
                           '--ros-args','--params-file',str(config)],stdout=log,stderr=log)
    rclpy.init();node=Node('test_moveit_no_hardware')
    client=node.create_client(GetStateValidity,'/moveit_guard/check_state_validity')
    reset=node.create_client(Trigger,'/moveit_guard/reset')
    status=[]; outputs={'left':[],'right':[]};subscriptions=[]
    states={};targets={}
    for side in outputs:
        states[side]=node.create_publisher(JointState,'/joint_states_'+side,1)
        targets[side]=node.create_publisher(JointState,'/pinocchio_checked/'+side+'_target',1)
        subscriptions.append(node.create_subscription(JointState,'/joint_ctrl_cmd_'+side,
            lambda m,s=side: outputs[s].append(m),10))
    subscriptions.append(node.create_subscription(String,'/moveit_guard/status',lambda m:status.append(json.loads(m.data)),10))
    upstream=node.create_publisher(Bool,'/collision_guard/stop_requested',
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    names=[f'{s}_joint{j}' for s in ('left','right') for j in range(1,9)]

    def call(client,req):
        future=client.call_async(req);end=time.monotonic()+3
        while not future.done() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.005)
        assert future.done(), 'service response timeout'
        return future.result()

    def validity(q, ns=names):
        request=GetStateValidity.Request()
        request.robot_state.joint_state.name=list(ns)
        request.robot_state.joint_state.position=list(map(float,q))
        return call(client,request)

    def publish(pub,q):
        msg=JointState();msg.header.stamp=node.get_clock().now().to_msg()
        msg.name=[f'joint{j}' for j in range(1,7)]+['gripper']
        msg.position=list(map(float,q))+[.02];pub.publish(msg)

    def pump(duration, feedback=None, target=None, stage1=False):
        until=time.monotonic()+duration
        while time.monotonic()<until:
            upstream.publish(Bool(data=stage1))
            if feedback is not None:
                for i,s in enumerate(('left','right')): publish(states[s],feedback[i])
            if target is not None:
                for i,s in enumerate(('left','right')): publish(targets[s],target[i])
            rclpy.spin_once(node,timeout_sec=.005)

    zeros=[np.zeros(6),np.zeros(6)]
    try:
        assert client.wait_for_service(timeout_sec=10), (tmp_path/'node.log').read_text()
        assert validity(np.zeros(16)).valid
        assert not validity([0.]*15,names[:15]).valid
        bad=np.zeros(16);bad[0]=float('nan');assert not validity(bad).valid
        bad=np.zeros(16);bad[1]=-1.;assert not validity(bad).valid
        rng=np.random.default_rng(23)
        collision_q=None
        for _ in range(300):
            q=np.zeros(16)
            for start in (0,8):
                q[start:start+6]=rng.uniform([-.9,.1,-2.3,-.7,-.7,-.7],[.9,2.,-.1,.7,.7,.7])
                q[start+6]=.01;q[start+7]=-.01
            result=validity(q)
            if not result.valid and result.contacts:
                collision_q=q;break
        assert collision_q is not None
        assert result.contacts[0].depth >= 0.
        assert any(c.contact_body_1.startswith('left_') != c.contact_body_2.startswith('left_') for c in result.contacts)
        pump(.25,zeros,zeros)
        assert call(reset,Trigger.Request()).success
        pump(.2,zeros,zeros)
        assert status and not status[-1]['blocked'], status[-5:]
        assert all(outputs.values())
        # Safe input actually reaches final topics, preserving the configured speed.
        moved=[np.array([.01,0,0,0,0,0]),np.zeros(6)]
        pump(.15,zeros,moved)
        assert outputs['left'][-1].position[0] == pytest.approx(.01)
        assert outputs['left'][-1].velocity[6] == 10.
        # A known volume collision must hold both arms at measured positions.
        colliding=[collision_q[:6],collision_q[8:14]]
        pump(.25,zeros,colliding)
        assert status[-1]['blocked'] and status[-1]['latched'],status[-5:]
        assert status[-1]['reason'] in ('predicted_box_collision','inputs_expired_during_check','calculation_timeout')
        assert outputs['left'][-1].position[:6] == pytest.approx([0.]*6)
        pump(.1,zeros,zeros)
        assert status[-1]['latched']
        assert call(reset,Trigger.Request()).success
        pump(.15,zeros,zeros)
        # Stage-1 denial blocks even a geometrically safe target.
        pump(.1,zeros,moved,stage1=True)
        assert status[-1]['blocked']
        assert outputs['left'][-1].position[:6] == pytest.approx([0.]*6)
        # Lost feedback must stop even repeated hold publication.
        pump(.7)
        count={s:len(v) for s,v in outputs.items()}
        pump(.1)
        assert count=={s:len(v) for s,v in outputs.items()}
        assert status[-1]['blocked']
        print('Known cross-arm collision pose:',collision_q.tolist())
    finally:
        node.destroy_node();rclpy.shutdown()
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        log.close()


def test_two_stage_launch_routes_only_through_moveit(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') == '179'
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    import sys
    env=os.environ.copy();env['VIRTUAL_ENV']=str(Path(sys.executable).parent.parent)
    log=(tmp_path/'launch.log').open('w')
    proc=subprocess.Popen([sys.executable,'/opt/ros/humble/bin/ros2','launch',
                           'piper_moveit_guard','two_stage_guard.launch.py',
                           'enforce:=true','geometry_confirmed:=true'],env=env,
                          stdout=log,stderr=log,start_new_session=True)
    rclpy.init();node=Node('test_two_stage_no_hardware')
    statuses={'pinocchio':[],'moveit':[]};subs=[];pubs={}
    for key,topic in [('pinocchio','/collision_guard/status'),('moveit','/moveit_guard/status')]:
        subs.append(node.create_subscription(String,topic,
          lambda m,k=key:statuses[k].append(json.loads(m.data)),10))
    for s in ('left','right'):
        for topic in ('/joint_states_'+s,'/teleop/'+s+'_target'):
            pubs[topic]=node.create_publisher(JointState,topic,1)
    clients=[node.create_client(Trigger,t) for t in ('/collision_guard/reset','/moveit_guard/reset')]
    def pump(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            for pub in pubs.values():
                m=JointState();m.header.stamp=node.get_clock().now().to_msg()
                m.name=[f'joint{j}' for j in range(1,7)]+['gripper'];m.position=[0.]*6+[.02]
                pub.publish(m)
            rclpy.spin_once(node,timeout_sec=.005)
    try:
        pump(2.)
        assert all(statuses.values()),(tmp_path/'launch.log').read_text()
        for client in clients:
            assert client.wait_for_service(timeout_sec=1)
            f=client.call_async(Trigger.Request());end=time.monotonic()+1
            while not f.done() and time.monotonic()<end:pump(.01)
            assert f.done() and f.result().success
        pump(.3)
        assert not statuses['pinocchio'][-1]['blocked'],statuses
        assert not statuses['moveit'][-1]['blocked'],statuses
        final=node.get_publishers_info_by_topic('/joint_ctrl_cmd_left')
        assert len(final)==1 and final[0].node_name=='piper_moveit_guard'
        intermediate=node.get_publishers_info_by_topic('/pinocchio_checked/left_target')
        assert len(intermediate)==1 and intermediate[0].node_name=='piper_collision_guard'
    finally:
        node.destroy_node();rclpy.shutdown();os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        log.close()


@pytest.mark.parametrize('launch_file,extra',[
    ('moveit_box_demo.launch.py',['gui:=false','rviz:=false']),
    ('moveit_box_demo.launch.py',['gui:=false','rviz:=false','editor:=true']),
    ('two_stage_guard.launch.py',[]),
])
def test_monitor_and_headless_demo_never_publish_driver_commands(tmp_path,launch_file,extra):
    assert os.environ.get('ROS_DOMAIN_ID') == '179'
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    import sys
    env=os.environ.copy();env['VIRTUAL_ENV']=str(Path(sys.executable).parent.parent)
    log=(tmp_path/'launch.log').open('w')
    proc=subprocess.Popen([sys.executable,'/opt/ros/humble/bin/ros2','launch','piper_moveit_guard',launch_file,*extra],
                          env=env,stdout=log,stderr=log,start_new_session=True)
    rclpy.init();node=Node('test_monitor_no_hardware');statuses=[];applied=[]
    from rclpy.qos import QoSProfile, DurabilityPolicy
    ack_sub=node.create_subscription(String,'/moveit_guard/applied_description',lambda m:applied.append(m.data),
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    sub=node.create_subscription(String,'/moveit_guard/status',lambda m:statuses.append(json.loads(m.data)),10)
    try:
        end=time.monotonic()+3
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        assert statuses,(tmp_path/'launch.log').read_text()
        if 'editor:=true' in extra:
            assert any('<robot' in text for text in applied), (tmp_path/'launch.log').read_text()
        assert node.count_publishers('/joint_ctrl_cmd_left')==0
        assert node.count_publishers('/joint_ctrl_cmd_right')==0
        if launch_file.startswith('moveit_box'):
            assert statuses[-1]['mode']=='demo' and not statuses[-1]['pose_collision']
    finally:
        node.destroy_node();rclpy.shutdown();os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        log.close()


def test_live_tuning_changes_moveit_geometry_and_saved_profile_loads_in_robot_mode(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID')=='179'
    import sys
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from moveit_msgs.srv import GetStateValidity
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from piper_collision_guard.box_tuning import BoxProfile
    base=(ROOT/'piper_moveit_guard/config/dual_piper_boxes.urdf').read_text()
    profile=BoxProfile(base);profile.links['left_base_link'][0]['size_mm']=[1000.,1000.,1000.]
    saved=profile.save(tmp_path/'collision_tuning.json')
    env=os.environ.copy();env['VIRTUAL_ENV']=str(Path(sys.executable).parent.parent)

    def run(launch,extra,live):
        log=(tmp_path/(launch+'.log')).open('w')
        proc=subprocess.Popen([sys.executable,'/opt/ros/humble/bin/ros2','launch','piper_moveit_guard',launch,*extra],
                              env=env,stdout=log,stderr=log,start_new_session=True)
        rclpy.init();node=Node('test_tuning_no_hardware')
        client=node.create_client(GetStateValidity,'/moveit_guard/check_state_validity')
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher=node.create_publisher(String,'/moveit_guard/tuned_description',qos)
        acks=[];sub=node.create_subscription(String,'/moveit_guard/applied_description',lambda m:acks.append(m.data),qos)
        def spin_until(predicate,seconds=3.):
            end=time.monotonic()+seconds
            while not predicate() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
            assert predicate(),(tmp_path/(launch+'.log')).read_text()
        def valid():
            req=GetStateValidity.Request();req.robot_state.joint_state.name=[f'{s}_joint{j}' for s in ('left','right') for j in range(1,9)]
            req.robot_state.joint_state.position=[0.]*16
            f=client.call_async(req);spin_until(f.done);return f.result().valid
        try:
            assert client.wait_for_service(timeout_sec=6),(tmp_path/(launch+'.log')).read_text()
            if live:
                assert valid()
                tuned=profile.description();publisher.publish(String(data=tuned));spin_until(lambda:tuned in acks)
                assert not valid()  # Actual FCL geometry changed, not just the drawing.
                restored=BoxProfile(base).description();publisher.publish(String(data=restored));spin_until(lambda:restored in acks)
                assert valid()
                # Invalid updates preserve the previously accepted model.
                publisher.publish(String(data='<broken/>'));spin_until(lambda:any(a.startswith('ERROR:') for a in acks))
                assert valid()
            else:
                assert not valid()  # Robot-mode launch loaded exactly the saved profile.
                # No live geometry-reload subscriber in monitor/enforce operation.
                spin_until(lambda:node.count_subscribers('/moveit_guard/tuned_description')==0)
            assert node.count_publishers('/joint_ctrl_cmd_left')==0
        finally:
            node.destroy_node();rclpy.shutdown();os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            log.close()
    run('moveit_box_demo.launch.py',['gui:=false','rviz:=false','editor:=false'],True)
    run('two_stage_guard.launch.py',['collision_config:='+str(saved)],False)
