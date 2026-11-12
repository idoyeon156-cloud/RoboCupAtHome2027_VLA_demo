"""Actual MoveIt backend: post collision is enabled for both arms."""
import os
from pathlib import Path
import signal
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml

pytestmark=pytest.mark.skipif(os.environ.get('RUN_SENSOR_MOUNT_ROS_TESTS')!='1',reason='isolated ROS tests')
ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('post_enabled',[False,True])
def test_actual_profile_collision_service(tmp_path,post_enabled):
    assert os.environ.get('ROS_DOMAIN_ID')=='181'
    import rclpy
    from rclpy.node import Node
    from ament_index_python.packages import get_package_prefix
    from moveit_msgs.srv import GetStateValidity
    from piper_collision_guard.collision_model import load_model
    description,_,_=load_model(ROOT/'piper_moveit_guard')
    if not post_enabled:
        robot=ET.fromstring(description);link=robot.find("link[@name='rear_profile_3030']")
        for collision in list(link.findall('collision')):link.remove(collision)
        description=ET.tostring(robot,encoding='unicode')
    params={'robot_description':description,'robot_description_semantic':(ROOT/'piper_moveit_guard/config/dual_piper.srdf').read_text(),'convex_meshes':True,'demo':True}
    path=tmp_path/'params.yaml';path.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
    with (tmp_path/'node.log').open('w') as log:
        proc=subprocess.Popen([get_package_prefix('piper_moveit_guard')+'/lib/piper_moveit_guard/moveit_guard','--ros-args','--params-file',str(path)],stdout=log,stderr=log)
        rclpy.init();node=Node('test_profile_no_driver')
        try:
            client=node.create_client(GetStateValidity,'/moveit_guard/check_state_validity')
            assert client.wait_for_service(timeout_sec=15),(tmp_path/'node.log').read_text()
            pose=np.deg2rad([70.37398553,66.28560393,-116.11908577,24.52879148,-15.16341032,-55.90519164])
            for side in ('zero','left','right'):
                q=np.zeros(16)
                if side=='left':q[:6]=pose
                if side=='right':
                    reflected=pose.copy();reflected[[0,3,5]]*=-1;q[8:14]=reflected
                req=GetStateValidity.Request();req.robot_state.joint_state.name=[f'{s}_joint{i}' for s in ('left','right') for i in range(1,9)];req.robot_state.joint_state.position=q.tolist()
                future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
                assert future.done();result=future.result()
                if side=='zero' or not post_enabled:
                    assert result.valid,[(c.contact_body_1,c.contact_body_2) for c in result.contacts]
                else:
                    assert not result.valid and any('rear_profile_3030' in (c.contact_body_1,c.contact_body_2) for c in result.contacts)
            assert node.count_publishers('/joint_ctrl_cmd_left')==0
        finally:
            node.destroy_node();rclpy.shutdown();proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
