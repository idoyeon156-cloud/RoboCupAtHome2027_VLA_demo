"""Isolated ROS smoke/regression tests; no hardware drivers."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import pytest
import yaml
pytestmark=pytest.mark.skipif(os.environ.get('RUN_VARIANT_ROS_TESTS')!='1',reason='opt-in isolated ROS test')
ROOT=Path(__file__).resolve().parents[1]

def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid,signal.SIGINT)
        try: proc.wait(timeout=8)
        except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGKILL);proc.wait()

@pytest.mark.parametrize('mode,package,launch,topic',[
 ('distance_demo','piper_collision_guard','collision_limit_visual_demo.launch.py','/collision_preview/status'),
 ('distance_real','piper_collision_guard','guard.launch.py','/collision_guard/status'),
 ('meshes_demo','piper_moveit_guard','moveit_box_demo.launch.py','/moveit_guard/status'),
 ('meshes_real','piper_moveit_guard','two_stage_guard.launch.py','/moveit_guard/status')])
def test_four_entry_points(tmp_path,mode,package,launch,topic):
    assert os.environ.get('ROS_DOMAIN_ID')=='181'
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from ament_index_python.packages import get_package_share_directory
    assert Path(get_package_share_directory(package)).resolve().is_relative_to(ROOT)
    args=['gui:=false','rviz:=false'] if mode.endswith('demo') else []
    env=os.environ.copy();env['VIRTUAL_ENV']=str(Path(sys.executable).parent.parent);env['ROS_LOG_DIR']=str(tmp_path/'ros_logs')
    with (tmp_path/'launch.log').open('w') as log:
        proc=subprocess.Popen([sys.executable,'/opt/ros/humble/bin/ros2','launch',package,launch,*args],env=env,stdout=log,stderr=log,start_new_session=True)
        rclpy.init();node=Node('variant_test_'+mode);statuses=[]
        sub=node.create_subscription(String,topic,lambda m:statuses.append(json.loads(m.data)),QoSProfile(depth=1))
        try:
            end=time.monotonic()+20
            while not statuses and time.monotonic()<end and proc.poll() is None: rclpy.spin_once(node,timeout_sec=.05)
            assert statuses,(tmp_path/'launch.log').read_text()
            assert proc.poll() is None,(tmp_path/'launch.log').read_text()
            if mode=='distance_demo':
                assert statuses[-1]['joint_center_limit_m']==0
                assert statuses[-1]['stop_clearance_m']==pytest.approx(.01)
            if mode.startswith('meshes'): assert statuses[-1]['geometry']=='convex_meshes'
            if mode.endswith('real'): assert statuses[-1]['mode']=='monitor'
            assert not node.get_publishers_info_by_topic('/joint_ctrl_cmd_left')
            assert not node.get_publishers_info_by_topic('/joint_ctrl_cmd_right')
        finally: node.destroy_node();rclpy.shutdown();stop(proc)

def test_custom_width_and_added_camera_in_real_moveit(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID')=='181'
    sys.path.insert(0,str(ROOT/'tools'))
    from prepare_model import prepare
    import rclpy
    from rclpy.node import Node
    from moveit_msgs.srv import GetStateValidity
    from ament_index_python.packages import get_package_prefix
    out=tmp_path/'variant';out.mkdir()
    for f in ('piper_moveit_guard','piper_collision_guard','piper_dual_description','model'): (out/f).mkdir()
    cfg=yaml.safe_load((ROOT/'config/robot.yaml').read_text());cfg['source_urdf']=str(ROOT/'model/dual_piper.source.urdf')
    cfg['grippers']['left']['max_opening_m']=.10;cfg['mounts']['right']['xyz']=[0.,-.30,.20]
    cfg['attachments']=[dict(name='head_camera',parent='world',xyz=[0.,.0875,.123],rpy=[0.,0.,0.],geometry={'box':[.08,.08,.08]},touch_links=[])]
    config=out/'robot.yaml';config.write_text(yaml.safe_dump(cfg));prepare(config,out)
    robot=ET.parse(out/'piper_moveit_guard/config/dual_piper_meshes.urdf').getroot()
    for mesh in robot.iter('mesh'):
        uri=mesh.get('filename')
        if uri.startswith('package://'):
            package,relative=uri[10:].split('/',1)
            path=(out if package=='piper_moveit_guard' else ROOT)/package/relative
            mesh.set('filename',path.resolve().as_uri())
    params={'robot_description':ET.tostring(robot,encoding='unicode'),'robot_description_semantic':(out/'piper_moveit_guard/config/dual_piper.srdf').read_text(),'convex_meshes':True,'demo':True}
    parameter_file=tmp_path/'params.yaml';parameter_file.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
    binary=Path(get_package_prefix('piper_moveit_guard'))/'lib/piper_moveit_guard/moveit_guard'
    with (tmp_path/'guard.log').open('w') as log:
        proc=subprocess.Popen([str(binary),'--ros-args','--params-file',str(parameter_file)],stdout=log,stderr=log,start_new_session=True)
        rclpy.init();node=Node('variant_custom_camera_test');client=node.create_client(GetStateValidity,'/moveit_guard/check_state_validity')
        try:
            assert client.wait_for_service(timeout_sec=15),(tmp_path/'guard.log').read_text()
            def check(opening):
                req=GetStateValidity.Request();req.robot_state.joint_state.name=[f'{s}_joint{i}' for s in ('left','right') for i in range(1,9)]
                q=[0.]*16;q[6]=opening/2;q[7]=-opening/2;req.robot_state.joint_state.position=q
                future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
                assert future.done();return future.result()
            result=check(.09)
            assert not result.valid and result.contacts,'90mm must pass 100mm bounds and reach collision check'
            assert any('head_camera' in (c.contact_body_1,c.contact_body_2) for c in result.contacts)
            result=check(.11)
            assert not result.valid and not result.contacts,'110mm exceeds the configured travel'
        finally: node.destroy_node();rclpy.shutdown();stop(proc)
