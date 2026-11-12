"""Isolated ROS tests; no hardware driver and no final command publisher."""
import os
from pathlib import Path
import time
import numpy as np
import pytest
import yaml

pytestmark=pytest.mark.skipif(os.environ.get('RUN_SENSOR_MOUNT_ROS_TESTS')!='1',reason='isolated ROS tests')
ROOT=Path(__file__).resolve().parents[1]


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
