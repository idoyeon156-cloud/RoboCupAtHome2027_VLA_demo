"""ROS2 point-distance monitor and optional joint-position command gate."""
import json
import math
from pathlib import Path
import time
import yaml

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from .distance import DistanceModel, Interlock, arm_state


class CollisionGuard(Node):
    def __init__(self):
        super().__init__("piper_collision_guard")
        default_urdf = str(Path(get_package_share_directory("piper_dual_description")) / "urdf/dual_piper.urdf")
        default_protection = str(Path(get_package_share_directory("piper_collision_guard")) / "config/protection.yaml")
        defaults = dict(urdf_path=default_urdf, protection_path=default_protection,
                        enforce=False, geometry_confirmed=False, rate_hz=50.0,
                        stop_clearance_m=0.02, resume_clearance_m=0.04,
                        feedback_timeout_s=0.15, target_timeout_s=0.15,
                        max_arm_skew_s=0.05, path_step_rad=0.02,
                        max_path_samples=100, link_samples=2,
                        driver_speed_ratio=10, gripper_effort_nm=1.0)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        c = self.cfg
        if not math.isfinite(c["rate_hz"]) or not 1 <= c["rate_hz"] <= 500:
            raise ValueError("rate_hz must be finite and in [1,500]")
        if not 1 <= c["driver_speed_ratio"] <= 100 or not 0.5 <= c["gripper_effort_nm"] <= 3:
            raise ValueError("invalid driver speed ratio/gripper effort")
        if not math.isfinite(c["target_timeout_s"]) or c["target_timeout_s"] <= 0:
            raise ValueError("target_timeout_s must be positive")
        if not math.isfinite(c["path_step_rad"]) or c["path_step_rad"] <= 0 or c["max_path_samples"] < 1:
            raise ValueError("invalid path sampling parameters")
        if c["enforce"] and not c["geometry_confirmed"]:
            raise ValueError("enforce requires geometry_confirmed:=true after calibrating protective spheres and stopping clearance")
        with open(c["protection_path"], encoding="utf-8") as stream:
            protection = yaml.safe_load(stream)
        if not isinstance(protection, dict) or set(protection) != {"left", "right"}:
            raise ValueError("protection YAML must configure both left and right")
        self.fk = DistanceModel(c["urdf_path"], c["link_samples"], protection)
        if c["enforce"] and any(np.any(self.fk.radii[s] <= 0) for s in ("left", "right")):
            raise ValueError("enforce needs positive protective radii")
        self.lock = Interlock(c["stop_clearance_m"], c["resume_clearance_m"],
                              c["feedback_timeout_s"], c["max_arm_skew_s"])
        self.feedback, self.targets = {}, {}
        self.feedback_times, self.target_times = {}, {}
        self.hold = None
        self.last_clearance = None
        self.started = time.monotonic()
        self.subscriptions_held = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.outputs = {}
        for side in ("left", "right"):
            self.subscriptions_held.append(self.create_subscription(
                JointState, f"/joint_states_{side}",
                lambda msg, s=side: self.receive(msg, s, False), qos))
            self.subscriptions_held.append(self.create_subscription(
                JointState, f"/teleop/{side}_target",
                lambda msg, s=side: self.receive(msg, s, True), qos))
            # In monitor mode no hardware-command publisher exists.
            if c["enforce"]:
                self.outputs[side] = self.create_publisher(JointState, f"/joint_ctrl_cmd_{side}", qos)
        self.status_pub = self.create_publisher(String, "/collision_guard/status", latched)
        self.stop_pub = self.create_publisher(Bool, "/collision_guard/stop_requested", latched)
        self.marker_pub = self.create_publisher(MarkerArray, "/collision_guard/markers", 1)
        self.reset_srv = self.create_service(Trigger, "/collision_guard/reset", self.reset)
        self.timer = self.create_timer(1.0/c["rate_hz"], self.tick)
        self.get_logger().info("ENFORCE: position-command gate active" if c["enforce"] else "MONITOR: no robot commands will be published")

    def receive(self, msg, side, target):
        states = self.targets if target else self.feedback
        times = self.target_times if target else self.feedback_times
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            timeout = self.cfg["target_timeout_s" if target else "feedback_timeout_s"]
            if stamp <= 0 or not -0.05 <= age <= timeout:
                raise ValueError("missing, future or stale source timestamp")
            state = arm_state(msg.name, msg.position, side)
            # Validate this arm's model limits without inventing the other arm's state.
            indices = self.fk.slots[side]
            if np.any(state[0] < self.fk.model.lowerPositionLimit[indices]) or np.any(state[0] > self.fk.model.upperPositionLimit[indices]):
                raise ValueError("joint angle outside URDF bounds")
            # Map source time to the monotonic clock, so queued old samples
            # do not become fresh merely because their callbacks ran now.
            states[side], times[side] = state, time.monotonic() - max(age, 0.0)
        except (ValueError, TypeError, OverflowError) as error:
            states.pop(side, None)
            times.pop(side, None)
            self.lock.trip(f"invalid_{'target' if target else 'feedback'}_{side}: {error}")

    def target_fresh(self, now):
        if set(self.target_times) != {"left", "right"}:
            return False
        ts = list(self.target_times.values())
        return all(0 <= now-t <= self.cfg["target_timeout_s"] for t in ts) and abs(ts[0]-ts[1]) <= self.cfg["max_arm_skew_s"]

    def reset(self, request, response):
        now = time.monotonic()
        fresh = self.lock.fresh(self.feedback_times, now)
        distance = math.nan
        if fresh:
            distance = self.fk.clearance(self.feedback["left"][0], self.feedback["right"][0]).distance
        response.success = self.lock.reset(distance, fresh)
        response.message = "Reset; both arms need new target messages" if response.success else "Reset refused: feedback stale or arms below resume distance"
        if response.success:
            self.targets.clear()
            self.target_times.clear()
            self.hold = None
        return response

    def publish_commands(self, states):
        for side in ("left", "right"):
            q, gripper = states[side]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [f"joint{i}" for i in range(1, 7)] + ["gripper"]
            msg.position = [float(x) for x in q] + [float(gripper)]
            # Upstream Piper uses velocity[6] as the global 1..100 speed ratio,
            # NOT a conventional seven-joint velocity command.
            msg.velocity = [0.0] * 6 + [float(self.cfg["driver_speed_ratio"])]
            msg.effort = [0.0] * 6 + [float(self.cfg["gripper_effort_nm"])]
            self.outputs[side].publish(msg)

    def markers(self, left, right):
        points = self.fk.points(left, right)
        array = MarkerArray()
        for side in ("left", "right"):
            for index, radius in enumerate(np.unique(points[side][2])):
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns, marker.id = f"{side}_protection", index
                marker.type, marker.action = Marker.SPHERE_LIST, Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.points = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2]))
                                 for c in points[side][0][points[side][2] == radius]]
                marker.scale.x = marker.scale.y = marker.scale.z = max(0.002, 2*float(radius))
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = ((0.2, 0.6, 1., .35) if side == "left" else (1., 0.4, 0.2, .35))
                array.markers.append(marker)
        self.marker_pub.publish(array)

    def tick(self):
        now = time.monotonic()
        fresh = self.lock.fresh(self.feedback_times, now)
        can_forward = False
        clearance = None
        path_distance = None
        try:
            if not fresh:
                if now-self.started > self.cfg["feedback_timeout_s"]:
                    self.lock.trip("feedback_timeout_or_arm_skew")
            else:
                left, right = self.feedback["left"][0], self.feedback["right"][0]
                clearance = self.fk.clearance(left, right)
                self.last_clearance = clearance
                self.lock.observe(clearance.distance)
                self.markers(left, right)
                if not self.lock.latched and self.target_fresh(now):
                    target_clearance = self.fk.path_clearance(
                        left, right, self.targets["left"][0], self.targets["right"][0],
                        self.cfg["path_step_rad"], self.cfg["max_path_samples"])
                    path_distance = target_clearance.distance
                    if path_distance <= self.cfg["stop_clearance_m"]:
                        self.lock.trip("predicted_distance_stop")
                    else:
                        can_forward = True
                elif not self.lock.latched and self.targets:
                    self.lock.trip("target_timeout_or_arm_skew")
            # Other publishers can bypass this node; refuse our own forwarding
            # when detected. This check is diagnostic, not access control.
            if self.cfg["enforce"] and any(self.count_publishers(f"/joint_ctrl_cmd_{s}") > 1 for s in ("left", "right")):
                self.lock.trip("another_driver_command_publisher")
                can_forward = False
            if self.cfg["enforce"] and fresh:
                if can_forward and not self.lock.latched:
                    self.hold = None
                    self.publish_commands(self.targets)
                else:
                    if self.hold is None:
                        self.hold = {s: (q.copy(), g) for s, (q, g) in self.feedback.items()}
                    self.publish_commands(self.hold)
        except Exception as error:
            self.lock.trip(f"calculation_error: {error}")
            can_forward = False
        blocked = self.lock.latched or not fresh or not can_forward
        self.stop_pub.publish(Bool(data=blocked))
        payload = dict(mode="enforce" if self.cfg["enforce"] else "monitor", blocked=blocked,
                       latched=self.lock.latched,
                       reason=self.lock.reason if self.lock.latched else ("clear" if can_forward else "waiting_for_fresh_feedback_or_targets"),
                       minimum_clearance_m=clearance.distance if clearance else None,
                       nearest_center_distance_m=clearance.center_distance if clearance else None,
                       nearest_pair=[clearance.left_name, clearance.right_name] if clearance else None,
                       nearest_xyz_m=[clearance.left_xyz.tolist(), clearance.right_xyz.tolist()] if clearance else None,
                       path_minimum_clearance_m=path_distance)
        self.status_pub.publish(String(data=json.dumps(payload, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CollisionGuard()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.stop_pub.publish(Bool(data=True))
            node.destroy_node()
        rclpy.shutdown()
