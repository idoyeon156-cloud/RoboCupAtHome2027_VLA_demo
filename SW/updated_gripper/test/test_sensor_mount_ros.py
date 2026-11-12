"""Opt-in checks against the actual MoveIt convex-mesh service, no driver."""
import os
from pathlib import Path
import signal
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml

pytestmark = pytest.mark.skipif(os.environ.get('RUN_SENSOR_MOUNT_ROS_TESTS') != '1', reason='isolated ROS test')
ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('overlap', [False, True])
def test_actual_moveit_service_checks_mount(tmp_path, overlap):
    assert os.environ.get('ROS_DOMAIN_ID') == '181'
    import rclpy
    import pinocchio as pin
    from rclpy.node import Node
    from ament_index_python.packages import get_package_prefix
    from moveit_msgs.srv import GetStateValidity
    from piper_collision_guard.collision_model import load_model
    from prepare_model import mesh_triangles
    description, _, _ = load_model(ROOT/'piper_moveit_guard')
    if overlap:
        robot = ET.fromstring(description)
        fk = pin.buildModelFromXML(description); data = fk.createData()
        pin.framesForwardKinematics(fk, data, pin.neutral(fk))
        mount_pose = data.oMf[fk.getFrameId('left_gripper_flange')]
        target_pose = data.oMf[fk.getFrameId('right_link6')]
        mount_part = robot.find("link[@name='left_gripper_flange']").findall('collision')[1]
        from urllib.parse import unquote, urlparse
        points = mesh_triangles(Path(unquote(urlparse(mount_part.find('geometry/mesh').get('filename')).path))).reshape(-1,3)
        from prepare_model import rotation
        pose = mount_part.find('origin')
        rot = rotation(list(map(float,pose.get('rpy').split())))
        xyz = np.fromstring(pose.get('xyz'),sep=' ')
        centre = target_pose.actInv(mount_pose.act(rot @ np.unique(points,axis=0).mean(axis=0)+xyz))
        for link in robot.findall('link'):
            if link.get('name') == 'left_gripper_flange':
                for collision in list(link.findall('collision')):
                    if collision is not mount_part:
                        link.remove(collision)
                continue
            for collision in link.findall('collision'):
                link.remove(collision)
        link = robot.find("link[@name='right_link6']")
        collision = ET.SubElement(link,'collision')
        ET.SubElement(collision,'origin',xyz=' '.join(map(str,centre)),rpy='0 0 0')
        from generate_collision_meshes import enclosing_polyhedron, write_stl
        import itertools
        cube=np.asarray(list(itertools.product((-.003,.003),repeat=3)))
        faces,_=enclosing_polyhedron(cube,0.)
        obstacle=tmp_path/'obstacle.stl';write_stl(obstacle,faces)
        ET.SubElement(ET.SubElement(collision,'geometry'),'mesh',filename=obstacle.as_uri())
        description = ET.tostring(robot,encoding='unicode')
    params = {'robot_description':description,'robot_description_semantic':(ROOT/'piper_moveit_guard/config/dual_piper.srdf').read_text(),'convex_meshes':True,'demo':True}
    config = tmp_path/'params.yaml'; config.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
    with (tmp_path/'node.log').open('w') as log:
        proc = subprocess.Popen([get_package_prefix('piper_moveit_guard')+'/lib/piper_moveit_guard/moveit_guard','--ros-args','--params-file',str(config)],stdout=log,stderr=log)
        rclpy.init(); node = Node('test_sensor_mount_service')
        try:
            client=node.create_client(GetStateValidity,'/moveit_guard/check_state_validity')
            assert client.wait_for_service(timeout_sec=15), (tmp_path/'node.log').read_text()
            for angle in ([0] if overlap else [-120,-60,0,60,120]):
                request=GetStateValidity.Request()
                request.robot_state.joint_state.name=[f'{s}_joint{i}' for s in ('left','right') for i in range(1,9)]
                q=np.zeros(16); q[5]=np.deg2rad(angle); q[13]=-q[5]
                request.robot_state.joint_state.position=q.tolist()
                future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
                assert future.done()
                result=future.result()
                if overlap:
                    assert not result.valid
                    assert any({c.contact_body_1,c.contact_body_2} == {'left_gripper_flange','right_link6'} for c in result.contacts)
                elif angle in (60,120):
                    # The two outward envelopes approach each other at this inward wrist rotation.
                    assert not result.valid
                    assert any({c.contact_body_1,c.contact_body_2} == {'left_gripper_flange','right_gripper_flange'} for c in result.contacts)
                else:
                    assert result.valid, [(c.contact_body_1,c.contact_body_2) for c in result.contacts]
            assert node.count_publishers('/joint_ctrl_cmd_left') == 0
        finally:
            node.destroy_node();rclpy.shutdown();proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
