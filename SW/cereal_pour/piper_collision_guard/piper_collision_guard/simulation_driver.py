"""Ideal feedback driver for the guarded RViz pipeline; never opens CAN.

All command endpoints must be remapped beneath /cereal_sim by the launcher.
This displays the final gate's accepted position stream, not robot dynamics.
"""
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from .cereal_pour import SIDES, JOINT_NAMES


class SimulationDriver(Node):
    def __init__(self):
        super().__init__('cereal_simulated_driver')
        self.declare_parameter('snapshot_receipt', '')
        self.receipt = self.get_parameter('snapshot_receipt').value
        self.q = np.zeros((2, 7))
        self.pending, self.statuses = {}, {}
        self.phase, self.future, self.service = 'boot', None, ''
        self.since, self.stable = time.monotonic(), None
        self.service_clients = {action: self.create_client(Trigger, name) for action, name in (
            ('distance_reset', '/collision_guard/reset'), ('mesh_reset', '/moveit_guard/reset'),
            ('run', '/cereal_pour/run'), ('stop', '/cereal_pour/stop'))}
        self.feedback = {s: self.create_publisher(JointState, '/joint_states_'+s, 1) for s in SIDES}
        self.full = self.create_publisher(JointState, '/cereal_sim/joint_states', 1)
        self.notice = self.create_publisher(Marker, '/cereal_sim/notice', 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for i, side in enumerate(SIDES):
            topic = self.resolve_topic_name('/joint_ctrl_cmd_'+side)
            if not topic.startswith('/cereal_sim/'):
                raise ValueError('Simulation requires /cereal_sim command remappings')
            self.create_subscription(JointState, '/joint_ctrl_cmd_'+side,
                                     lambda m, s=side: self.command(s, m), 1)
        for name in ('collision_guard', 'moveit_guard', 'cereal_pour'):
            self.create_subscription(String, '/'+name+'/status',
                                     lambda m, n=name: self.status(n, m), latched)
        self.create_timer(.02, self.tick)
        self.get_logger().info('IDEAL SIMULATION: final guarded commands only; no CAN, no hardware dynamics')

    def command(self, side, msg):
        values = dict(zip(msg.name, msg.position))
        if len(values) != len(msg.name) or not all(k in values for k in JOINT_NAMES):
            self.fail('malformed final driver command')
            return
        q = np.array([values[k] for k in JOINT_NAMES])
        if not np.isfinite(q).all():
            self.fail('nonfinite final driver command')
            return
        self.pending[side] = (q, time.monotonic())

    def status(self, name, msg):
        try:
            self.statuses[name] = (json.loads(msg.data), time.monotonic())
        except (ValueError, TypeError):
            self.fail('invalid '+name+' status')

    def fail(self, detail):
        if self.phase != 'failed':
            self.get_logger().error('SIMULATION STOPPED: '+detail)
            self.phase, self.error = 'failed', detail
            if self.service_clients['stop'].service_is_ready():
                self.service_clients['stop'].call_async(Trigger.Request())

    def request(self, action):
        if not self.service_clients[action].service_is_ready():
            self.fail('service unavailable: '+action)
            return
        self.service, self.since = action, time.monotonic()
        self.future = self.service_clients[action].call_async(Trigger.Request())
        self.phase = 'rpc'

    def tick(self):
        now = time.monotonic()
        fresh = all(n in self.statuses and now-self.statuses[n][1] < .3
                    for n in ('collision_guard', 'moveit_guard'))
        clear = fresh and all(not self.statuses[n][0]['blocked'] and not self.statuses[n][0]['latched']
                              for n in ('collision_guard', 'moveit_guard'))
        running = self.phase == 'running' or (self.phase == 'rpc' and self.service == 'run')
        if running and clear and all(s in self.pending and now-self.pending[s][1] < .15 for s in SIDES):
            if abs(self.pending['left'][1]-self.pending['right'][1]) <= .05:
                self.q = np.stack([self.pending[s][0] for s in SIDES])
        stamp = self.get_clock().now().to_msg()
        for side, q in zip(SIDES, self.q):
            msg = JointState();msg.header.stamp = stamp
            msg.name = JOINT_NAMES;msg.position = q.tolist()
            self.feedback[side].publish(msg)
        full = JointState();full.header.stamp = stamp
        full.name = [f'{s}_joint{i}' for s in SIDES for i in range(1, 9)]
        full.position = np.column_stack((self.q[:, :6], self.q[:, 6]/2, -self.q[:, 6]/2)).flatten().tolist()
        self.full.publish(full)
        if self.phase == 'boot':
            ready = fresh and 'cereal_pour' in self.statuses and all(c.service_is_ready() for c in self.service_clients.values())
            self.stable = (self.stable or now) if ready else None
            if self.stable is not None and now-self.stable >= 3.:
                self.request('distance_reset')
            elif now-self.since > 15.:
                self.fail('initial feedback/guard services not ready')
        elif self.phase == 'rpc':
            if self.future.done():
                try:
                    response = self.future.result()
                    if response is None or not response.success:
                        self.fail(self.service+': '+(response.message if response else 'no response'))
                    elif self.service == 'distance_reset':
                        self.request('mesh_reset')
                    elif self.service == 'mesh_reset':
                        self.phase, self.since, self.stable = 'unlocked', now, None
                    elif self.service == 'run':
                        self.phase = 'running'
                except Exception as error:
                    self.fail(str(error))
            elif now-self.since > 5.:
                self.fail(self.service+' response timeout')
        elif self.phase == 'unlocked':
            unlocked = fresh and all(not self.statuses[n][0]['latched'] for n in ('collision_guard', 'moveit_guard'))
            self.stable = (self.stable or now) if unlocked else None
            if self.stable is not None and now-self.stable >= 3.:
                self.request('run')
            elif now-self.since > 6.:
                self.fail('guards did not stay unlocked after initial reset')
        elif self.phase == 'running':
            state = self.statuses.get('cereal_pour', ({}, 0))[0]
            if not fresh:
                self.fail('guard status expired')
            elif any(self.statuses[n][0]['latched'] for n in ('collision_guard', 'moveit_guard')):
                self.fail(json.dumps({n: self.statuses[n][0] for n in ('collision_guard', 'moveit_guard')}))
            elif state.get('state') == 'stopped':
                self.fail(state.get('detail', 'sequence stopped'))
            elif state.get('state') == 'complete' and state.get('detail') == 'Finished; holding return_home':
                self.phase = 'complete'
                if self.receipt:
                    from .plan_snapshot import complete_snapshot
                    try:
                        complete_snapshot(self.receipt)
                    except (OSError, ValueError) as error:
                        self.fail('snapshot: '+str(error))
                if self.phase == 'complete':
                    self.get_logger().info('SIMULATION COMPLETE: guarded home-to-home sequence; not hardware validation')
        marker = Marker();marker.header.frame_id = 'world';marker.header.stamp = stamp
        marker.ns = 'cereal_simulation';marker.id = 0;marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD;marker.pose.orientation.w = 1.;marker.pose.position.z = .9
        marker.scale.z = .027;marker.color.a = 1.
        marker.color.r, marker.color.g = (1., .15) if self.phase == 'failed' else (.5, 1.)
        marker.text = 'IDEAL GUARDED SIM: '+self.phase
        if self.phase == 'failed':marker.text += '\n'+self.error[:180]
        self.notice.publish(marker)


def main():
    rclpy.init();node = None
    try:
        node = SimulationDriver();rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
