"""Opt-in headless launch test; no real robot and no GUI required."""
import json
import os
import signal
import subprocess
import tempfile
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('RUN_VISUAL_DEMO_TESTS') != '1',
                                reason='requires isolated visual demo ROS domain')


@pytest.mark.parametrize('gui', ['false', 'true'])
def test_visual_demo_launch_uses_updated_spacing_and_only_simulation_topics(gui):
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import MarkerArray

    assert os.environ.get('ROS_DOMAIN_ID') == os.environ.get('VISUAL_DEMO_TEST_DOMAIN_ID', '178')
    rclpy.init(args=[])
    observer = Node('visual_demo_test_observer')
    buffer = Buffer()
    listener = TransformListener(buffer, observer)
    statuses, joints, markers = [], [], []
    subscriptions = [
        observer.create_subscription(String, '/collision_preview/status', lambda m: statuses.append(json.loads(m.data)), 1),
        observer.create_subscription(JointState, '/collision_preview/slider_joint_states', lambda m: joints.append(m), 1),
        observer.create_subscription(MarkerArray, '/collision_preview/markers', lambda m: markers.append(m), 1),
    ]
    with tempfile.TemporaryFile(mode='w+') as log:
        environment = dict(os.environ)
        if gui == 'true':
            environment['QT_QPA_PLATFORM'] = 'offscreen'
        process = subprocess.Popen(
            ['/opt/ros/humble/bin/ros2', 'launch', 'piper_collision_guard',
             'collision_limit_visual_demo.launch.py', f'gui:={gui}', 'rviz:=false'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=environment)
        try:
            deadline = time.monotonic()+12
            tf_left = tf_ghost = None
            while time.monotonic() < deadline:
                rclpy.spin_once(observer, timeout_sec=.05)
                try:
                    tf_left = buffer.lookup_transform('world', 'left_base_link', Time())
                    tf_ghost = buffer.lookup_transform('world', 'ghost/left_base_link', Time())
                except Exception:
                    continue
                if statuses and joints and markers:
                    break
            log.seek(0)
            details = log.read()
            assert process.poll() is None, details
            assert statuses and joints and markers, details
            assert statuses[-1]['minimum_clearance_m'] == pytest.approx(.095)
            assert statuses[-1]['nearest_center_distance_m'] == pytest.approx(.175)
            assert not statuses[-1]['blocked']
            assert len(joints[-1].name) == 16
            assert tf_left.transform.translation.y == pytest.approx(.0875)
            assert tf_ghost.transform.translation.y == pytest.approx(.0875)
            assert len(markers[-1].markers) > 2
            assert observer.count_publishers('/joint_ctrl_cmd_left') == 0
            assert observer.count_publishers('/joint_ctrl_cmd_right') == 0
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            observer.destroy_node()
            rclpy.shutdown()
