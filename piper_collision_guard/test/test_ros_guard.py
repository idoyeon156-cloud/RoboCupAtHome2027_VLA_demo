"""Opt-in ROS middleware test. Uses fake states, no SDK/CAN/driver.

Run only in an isolated ROS domain with RUN_ROS_GUARD_TESTS=1.
"""
import json
import os
import time

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get("RUN_ROS_GUARD_TESTS") != "1",
                                reason="requires explicit isolated ROS test domain")


def test_ros_monitor_and_command_gate():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from piper_collision_guard.node import CollisionGuard

    # Refuse accidental test execution on the production/default domain.
    assert os.environ.get("ROS_DOMAIN_ID") == "177"
    rclpy.init(args=["--ros-args", "-p", "enforce:=false", "-p", "feedback_timeout_s:=0.5",
                      "-p", "resume_clearance_m:=0.021", "-p", "target_timeout_s:=0.5", "-p", "max_arm_skew_s:=0.15"])
    monitor = CollisionGuard()
    assert monitor.outputs == {}
    monitor.destroy_node()
    rclpy.shutdown()

    rclpy.init(args=["--ros-args", "-p", "enforce:=true", "-p", "geometry_confirmed:=true", "-p", "feedback_timeout_s:=0.5",
                      "-p", "resume_clearance_m:=0.021", "-p", "target_timeout_s:=0.5", "-p", "max_arm_skew_s:=0.15"])
    guard = CollisionGuard()
    fake = Node("fake_piper_no_hardware")
    executor = SingleThreadedExecutor()
    executor.add_node(guard)
    executor.add_node(fake)
    received = {"left": [], "right": []}
    states, targets, subscribers = {}, {}, []
    status = []
    for side in ("left", "right"):
        states[side] = fake.create_publisher(JointState, f"/joint_states_{side}", 1)
        targets[side] = fake.create_publisher(JointState, f"/teleop/{side}_target", 1)
        subscribers.append(fake.create_subscription(
            JointState, f"/joint_ctrl_cmd_{side}", lambda msg, s=side: received[s].append(msg), 10))
    subscribers.append(fake.create_subscription(String, "/collision_guard/status",
                                               lambda msg: status.append(json.loads(msg.data)), 10))
    client = fake.create_client(Trigger, "/collision_guard/reset")

    def publish(pub, q):
        msg = JointState()
        msg.header.stamp = fake.get_clock().now().to_msg()
        msg.name = [f"joint{i}" for i in range(1, 7)] + ["gripper"]
        msg.position = list(map(float, q)) + [0.02]
        pub.publish(msg)

    def pump(seconds, left=None, right=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if left is not None:
                publish(states["left"], np.zeros(6))
                publish(states["right"], np.zeros(6))
                publish(targets["left"], left)
                publish(targets["right"], right)
            executor.spin_once(timeout_sec=0.005)

    try:
        pump(.2, np.zeros(6), np.zeros(6))
        # Reset if discovery happened after the initial feedback timeout.
        if guard.lock.latched:
            response = guard.reset(Trigger.Request(), Trigger.Response())
            assert response.success
        pump(.2, np.zeros(6), np.zeros(6))
        assert received["left"] and received["right"]
        assert not guard.lock.latched
        assert received["left"][-1].velocity[6] == 10.

        near_left = np.array([np.deg2rad(15), 0., 0., 0., 0., 0.])
        near_right = np.zeros(6)
        pump(.2, near_left, near_right)
        assert guard.lock.latched and guard.lock.reason == "predicted_distance_stop"
        for side in received:
            np.testing.assert_allclose(received[side][-1].position[:6], np.zeros(6))
        pump(.1, np.zeros(6), np.zeros(6))
        assert guard.lock.latched  # clearance alone cannot auto-resume

        assert client.wait_for_service(timeout_sec=.5)
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic()+.3
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        assert future.done() and future.result().success
        assert not guard.targets  # old Quest targets must not be replayed
        pump(.1, np.zeros(6), np.zeros(6))
        assert not guard.lock.latched

        # Freshly stamped NaN is rejected before FK, then held at measured pose.
        pump(.1, np.array([float('nan'), 0, 0, 0, 0, 0]), np.zeros(6))
        assert guard.lock.latched and "invalid_target_left" in guard.lock.reason
        guard.reset(Trigger.Request(), Trigger.Response())
        pump(.1, np.zeros(6), np.zeros(6))
        pump(.7)  # no feedback at all: gate must stop publishing stale holds
        assert guard.lock.latched and guard.lock.reason == "feedback_timeout_or_arm_skew"
        before = {s: len(v) for s, v in received.items()}
        pump(.1)
        assert before == {s: len(v) for s, v in received.items()}
        assert status and status[-1]["blocked"]
    finally:
        executor.remove_node(guard)
        executor.remove_node(fake)
        guard.destroy_node()
        fake.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
