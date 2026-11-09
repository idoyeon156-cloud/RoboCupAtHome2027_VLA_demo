"""A small convex link fully inside another must still report a collision."""
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

pytestmark=pytest.mark.skipif(os.environ.get('RUN_MOVEIT_GUARD_TESTS')!='1',reason='isolated ROS tests')
ROOT=Path(__file__).resolve().parents[2]


def test_convex_mesh_backend_detects_solid_containment(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID')=='181'
    import rclpy
    from rclpy.node import Node
    from moveit_msgs.srv import GetStateValidity
    from ament_index_python.packages import get_package_prefix
    from piper_collision_guard.collision_model import load_model
    import sys
    sys.path.insert(0,str(ROOT/'tools'))
    from generate_collision_boxes import triangles
    description,_,_=load_model(ROOT/'piper_moveit_guard')
    root=ET.fromstring(description)
    poses={'world':np.eye(4)}
    pending=list(root.findall('joint'))
    while pending:
        for joint in list(pending):
            parent=joint.find('parent').get('link')
            if parent not in poses:continue
            origin=joint.find('origin');local=np.eye(4)
            local[:3,:3]=Rotation.from_euler('xyz',list(map(float,origin.get('rpy','0 0 0').split()))).as_matrix()
            local[:3,3]=list(map(float,origin.get('xyz','0 0 0').split()))
            poses[joint.find('child').get('link')]=poses[parent]@local
            pending.remove(joint)
    outer=root.find("link[@name='right_base_link']/collision")
    inner=root.find("link[@name='left_link6']/collision")
    destination=poses['right_base_link'].copy()
    destination[:3,3]+=destination[:3,:3]@np.array([-.009,0.,.043])
    local=np.linalg.inv(poses['left_link6'])@destination
    inner.find('origin').set('xyz',' '.join(map(str,local[:3,3])))
    inner.find('origin').set('rpy',' '.join(map(str,Rotation.from_matrix(local[:3,:3]).as_euler('xyz'))))
    def vertices(c):return np.array(triangles(Path(c.find('geometry/mesh').get('filename')[7:]))).reshape(-1,3)
    big=ConvexHull(vertices(outer))
    small=vertices(inner)+np.array([-.009,0.,.043])
    # Strictly inside: no triangle surfaces touch or cross.
    assert (small@big.equations[:,:3].T+big.equations[:,3]).max()<-.001
    for link in root.findall('link'):
        if link.get('name') not in ('right_base_link','left_link6'):
            for c in link.findall('collision'):link.remove(c)
    config=tmp_path/'contained.yaml'
    config.write_text(yaml.safe_dump({'/**':{'ros__parameters':{
        'robot_description':ET.tostring(root,encoding='unicode'),
        'robot_description_semantic':(ROOT/'piper_moveit_guard/config/dual_piper.srdf').read_text(),
        'convex_meshes':True}}}))
    log=(tmp_path/'contained.log').open('w')
    proc=subprocess.Popen([get_package_prefix('piper_moveit_guard')+'/lib/piper_moveit_guard/moveit_guard',
                           '--ros-args','--params-file',str(config)],stdout=log,stderr=log)
    rclpy.init();node=Node('test_solid_containment')
    client=node.create_client(GetStateValidity,'/moveit_guard/check_state_validity')
    try:
        assert client.wait_for_service(timeout_sec=8),(tmp_path/'contained.log').read_text()
        request=GetStateValidity.Request()
        request.robot_state.joint_state.name=[f'{side}_joint{i}' for side in ('left','right') for i in range(1,9)]
        request.robot_state.joint_state.position=[0.]*16
        future=client.call_async(request);end=time.monotonic()+3
        while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        assert future.done()
        result=future.result()
        assert not result.valid and result.contacts
        assert {result.contacts[0].contact_body_1,result.contacts[0].contact_body_2}=={'right_base_link','left_link6'}
        assert node.count_publishers('/joint_ctrl_cmd_left')==0
    finally:
        node.destroy_node();rclpy.shutdown();proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        log.close()
