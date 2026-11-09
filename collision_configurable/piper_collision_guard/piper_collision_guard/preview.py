"""Collision-limit VISUALISATION DEMO. No SDK, CAN, or hardware topics."""
import json
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

from .distance import DistanceModel
from .urdf_tools import placed_description


class PreviewState:
    """Requested ghost pose and accepted pose, using the same distance model.

    The demo auto-allows a subsequent safe pose. Unlike the real guard, it
    deliberately has no stop latch so users can explore with sliders.
    """
    def __init__(self, urdf, protection, stop_clearance=0.02, block_unsafe=True,
                 path_step_rad=0.02, max_samples=400, joint_center_limit_m=0.0):
        if not np.isfinite(stop_clearance) or stop_clearance < 0:
            raise ValueError("stop clearance must be finite and nonnegative")
        if not np.isfinite(joint_center_limit_m) or joint_center_limit_m < 0:
            raise ValueError("joint centre limit must be finite and nonnegative")
        self.joint_center_limit_m = float(joint_center_limit_m)
        if self.joint_center_limit_m:
            # Explicit comparison mode: equal sphere surface gap reaches its stop value when
            # joint-centre distance reaches the requested limit. No link samples.
            radius = min(.04, self.joint_center_limit_m / 2)
            protection = {side: {'radii_m': [radius] * 7}
                          for side in ('left', 'right')}
            stop_clearance = self.joint_center_limit_m - 2 * radius
        self.fk = DistanceModel(urdf, link_samples=0,
                                protection=protection)
        self.stop = float(stop_clearance)
        self.block_unsafe = block_unsafe
        self.path_step_rad, self.max_samples = path_step_rad, max_samples
        self.names = []
        self.indices = []
        for jid in range(1, self.fk.model.njoints):
            joint = self.fk.model.joints[jid]
            if joint.nq != 1:
                raise ValueError("preview expects single-coordinate Piper joints")
            self.names.append(self.fk.model.names[jid])
            self.indices.append(joint.idx_q)
        self.requested = pin.neutral(self.fk.model)
        self.accepted = self.requested.copy()
        self.blocked = False
        self.reason = "clear"
        self.path_gap = self.clearance().distance

    def arms(self, q):
        return q[self.fk.slots['left']], q[self.fk.slots['right']]

    def clearance(self):
        return self.fk.clearance(*self.arms(self.requested))

    def update(self, names, positions):
        if len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError("invalid slider joint message")
        mapping = dict(zip(names, positions))
        if not set(self.names) <= set(mapping):
            raise ValueError("slider message must contain all dual-arm joints")
        q = self.requested.copy()
        q[self.indices] = [mapping[name] for name in self.names]
        if not np.isfinite(q).all() or np.any(q < self.fk.model.lowerPositionLimit) or np.any(q > self.fk.model.upperPositionLimit):
            raise ValueError("slider state outside URDF limits")
        self.requested = q
        try:
            c = self.fk.path_clearance(*self.arms(self.accepted), *self.arms(q),
                                       self.path_step_rad, self.max_samples)
            self.path_gap = c.distance
            self.blocked = c.distance <= self.stop
            self.reason = "distance_limit" if self.blocked else "clear"
        except ValueError as error:
            self.blocked = True
            self.reason = 'path_sampling_budget' if 'sampling budget' in str(error) else f'invalid_path: {error}'
            self.path_gap = None
        if not self.block_unsafe or not self.blocked:
            self.accepted = q.copy()


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import Point
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray

    class VisualDemo(Node):
        def __init__(self):
            super().__init__('collision_limit_visual_demo')
            share = Path(get_package_share_directory('piper_collision_guard'))
            description = Path(get_package_share_directory('piper_dual_description'))
            self.declare_parameter('urdf_path', str(description/'urdf/dual_piper.urdf'))
            self.declare_parameter('robot_description', '')
            self.declare_parameter('protection_path', str(share/'config/protection_mesh_cover.yaml'))
            self.declare_parameter('stop_clearance_m', 0.02)
            self.declare_parameter('block_unsafe', True)
            self.declare_parameter('link_samples', 0)
            self.declare_parameter('path_step_rad', 0.02)
            self.declare_parameter('max_path_samples', 100)
            self.declare_parameter('joint_center_limit_m', 0.0)
            with open(self.get_parameter('protection_path').value, encoding='utf8') as stream:
                protection = yaml.safe_load(stream)
            robot_description = self.get_parameter('robot_description').value or self.get_parameter('urdf_path').value
            self.state = PreviewState(robot_description, protection,
                                      self.get_parameter('stop_clearance_m').value,
                                      self.get_parameter('block_unsafe').value,
                                      joint_center_limit_m=self.get_parameter('joint_center_limit_m').value,
                                      path_step_rad=self.get_parameter('path_step_rad').value,
                                      max_samples=self.get_parameter('max_path_samples').value)
            self.state.fk.link_samples = self.get_parameter('link_samples').value
            self.accepted_pub = self.create_publisher(JointState, '/collision_preview/accepted_joint_states', 1)
            self.requested_pub = self.create_publisher(JointState, '/collision_preview/ghost_joint_states', 1)
            self.markers_pub = self.create_publisher(MarkerArray, '/collision_preview/markers', 1)
            self.status_pub = self.create_publisher(String, '/collision_preview/status', 1)
            self.sub = self.create_subscription(JointState, '/collision_preview/slider_joint_states', self.receive, 1)
            self.timer = self.create_timer(0.05, self.publish)
            self.get_logger().info('COLLISION LIMIT VISUAL DEMO: slider + RViz only; no real robot commands')

        def receive(self, msg):
            try:
                self.state.update(msg.name, msg.position)
            except (ValueError, TypeError) as error:
                self.state.blocked, self.state.reason = True, f'invalid_demo_input: {error}'

        def marker(self, namespace, index, kind):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = namespace, index, kind, Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        def publish(self):
            s = self.state
            for publisher, q in ((self.accepted_pub, s.accepted), (self.requested_pub, s.requested)):
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = s.names
                msg.position = [float(v) for v in q[s.indices]]
                publisher.publish(msg)
            c = s.clearance()
            points = s.fk.points(*s.arms(s.requested))
            color = (1., .12, .12, .28) if s.blocked else (.1, .85, .3, .22)
            array = MarkerArray()
            radii_by_name = {}
            for side in ('left', 'right'):
                centers, names, radii = points[side]
                radii_by_name.update(zip(names, radii))
                for i, radius in enumerate(np.unique(radii)):
                    m = self.marker(f'{side}_requested_spheres', i, Marker.SPHERE_LIST)
                    m.points = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2]))
                                for c in centers[radii == radius]]
                    m.scale.x = m.scale.y = m.scale.z = max(.002, 2*float(radius))
                    m.color.r, m.color.g, m.color.b, m.color.a = color
                    array.markers.append(m)
            line = self.marker('nearest_surface_pair', 0, Marker.LINE_LIST)
            line.scale.x = .004
            line.color.r, line.color.g, line.color.b, line.color.a = (1., .2, .1, 1.) if s.blocked else (1., .9, .1, 1.)
            delta = c.right_xyz-c.left_xyz
            unit = delta / np.linalg.norm(delta) if np.linalg.norm(delta) > 1e-12 else np.array([1., 0., 0.])
            endpoints = [c.left_xyz+unit*radii_by_name[c.left_name], c.right_xyz-unit*radii_by_name[c.right_name]]
            line.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in endpoints]
            array.markers.append(line)
            label = self.marker('limit_status', 0, Marker.TEXT_VIEW_FACING)
            label.pose.position.z = .75
            label.scale.z = .035
            label.color.r, label.color.g, label.color.b, label.color.a = (1., .3, .3, 1.) if s.blocked else (.3, 1., .5, 1.)
            action = 'ACCEPTED POSE HELD' if s.block_unsafe and s.blocked else 'INSPECT ONLY' if not s.block_unsafe else 'FOLLOWING'
            path_text = 'unavailable' if s.path_gap is None else f'{s.path_gap*1000:.1f} mm'
            centre_text = (f'Joint centre limit: {s.joint_center_limit_m*1000:.0f} mm | Nearest centres: {c.center_distance*1000:.1f} mm\n'
                           if s.joint_center_limit_m else '')
            label.text = centre_text + (f"{'LIMIT REACHED' if s.blocked else 'CLEAR'} | {action}\n"
                          f"Requested gap: {c.distance*1000:.1f} mm | Stop: {s.stop*1000:.1f} mm\n"
                          f"Path gap: {path_text}\n{s.reason}")
            array.markers.append(label)
            self.markers_pub.publish(array)
            self.status_pub.publish(String(data=json.dumps(dict(
                visual_demo=True, blocked=s.blocked, reason=s.reason,
                distance_mode="joint_centres" if s.joint_center_limit_m else ("mesh_cover" if s.fk.covers else "joint_protection"),
                joint_center_limit_m=s.joint_center_limit_m,
                minimum_clearance_m=c.distance, nearest_center_distance_m=c.center_distance,
                stop_clearance_m=s.stop, path_minimum_clearance_m=s.path_gap,
                nearest_pair=[c.left_name, c.right_name], block_unsafe=s.block_unsafe))))

    rclpy.init(args=args)
    node = VisualDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
