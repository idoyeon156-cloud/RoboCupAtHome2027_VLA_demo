"""Replay the empty-gripper pour via the existing two-stage command gate."""
import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from .distance import DistanceModel, arm_state


SIDES = ('left', 'right')
JOINT_NAMES = [f'joint{i}' for i in range(1, 7)] + ['gripper']
POSES = ('ready', 'align', 'tilt_25', 'tilt_45', 'tilt_65')
POUR_ORDER = POSES + tuple(reversed(POSES[:-1]))
TICK_S = .02
FEEDBACK_AGE_S = .15
PAIR_SKEW_S = .05
GUARD_AGE_S = .30
TRACK_TOL = np.array([np.deg2rad(1.5)] * 6 + [.004])
GOAL_TOL = np.array([np.deg2rad(.7)] * 6 + [.002])
SETTLE_S = .20


def read_sequence(config):
    left = np.r_[np.deg2rad(config['bowl_arm_deg']), config['bowl_opening_mm']/1000.]
    poses = {}
    for name in POSES:
        right = np.r_[np.deg2rad(config['right_waypoints'][name]),
                      config['box_opening_mm']/1000.]
        pose = np.stack([left, right])
        if pose.shape != (2, 7) or not np.isfinite(pose).all():
            raise ValueError(f'{name} needs six finite joint angles and one opening per arm')
        poses[name] = pose
    joint_rate = np.deg2rad(float(config['joint_speed_deg_s']))
    gripper_rate = float(config['opening_speed_mm_s'])/1000.
    rates = np.r_[[joint_rate]*6, gripper_rate]
    pour_hold = float(config['pour_hold_s'])
    if (not np.all(np.isfinite(rates)) or np.any(rates <= 0)
            or not np.isfinite(pour_hold) or pour_hold < 0):
        raise ValueError('Require positive joint/gripper speeds and a nonnegative pour hold')
    return poses, rates, pour_hold


class CerealPour(Node):
    def __init__(self):
        super().__init__('cereal_pour')
        share = Path(get_package_share_directory('piper_collision_guard'))
        model_share = Path(get_package_share_directory('piper_dual_description'))
        self.declare_parameter('sequence_path', str(share/'config/cereal_pour.yaml'))
        self.declare_parameter('enforce', False)
        self.mode = 'enforce' if self.get_parameter('enforce').value else 'monitor'
        config = yaml.safe_load(Path(self.get_parameter('sequence_path').value).read_text())
        self.model = DistanceModel(model_share/'urdf/dual_piper.urdf')
        self.lower = np.array([
            np.r_[self.model.model.lowerPositionLimit[self.model.slots[s]], 0.] for s in SIDES
        ])
        self.upper = np.array([
            np.r_[self.model.model.upperPositionLimit[self.model.slots[s]],
                  self.model.max_opening[s]] for s in SIDES
        ])
        poses, self.rates, self.pour_hold = read_sequence(config)
        self.poses = {name: self.bounded(values) for name, values in poses.items()}

        self.feedback = {}
        self.guard_status = {}
        self.command = None
        self.steps = []
        self.step_index = 0
        self.settled_at = None
        self.last_tick = 0.
        self.state = 'idle'
        self.detail = 'waiting for prepare or start'
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.targets = {}
        for side in SIDES:
            self.create_subscription(JointState, f'/joint_states_{side}',
                                     lambda msg, s=side: self.receive_feedback(s, msg), 1)
            self.targets[side] = self.create_publisher(JointState, f'/teleop/{side}_target', 1)
        for name in ('collision_guard', 'moveit_guard'):
            self.create_subscription(String, f'/{name}/status',
                                     lambda msg, n=name: self.receive_guard(n, msg), latched)
        self.status = self.create_publisher(String, '/cereal_pour/status', latched)
        self.create_service(Trigger, '/cereal_pour/prepare', self.prepare)
        self.create_service(Trigger, '/cereal_pour/start', self.start)
        self.create_service(Trigger, '/cereal_pour/stop', self.stop)
        self.create_timer(TICK_S, self.tick)
        self.report('idle', self.detail)

    def bounded(self, values):
        if (values.shape != (2, 7) or not np.isfinite(values).all()
                or np.any(values < self.lower) or np.any(values > self.upper)):
            raise ValueError('Joint target outside the loaded dual PiPER URDF limits')
        return values

    def receive_feedback(self, side, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        age = self.get_clock().now().nanoseconds*1e-9 - stamp
        index = SIDES.index(side)
        try:
            if stamp <= 0 or not -.05 <= age <= FEEDBACK_AGE_S:
                raise ValueError('stale feedback')
            q, opening = arm_state(msg.name, msg.position, side, self.model.max_opening[side])
            values = np.r_[q, opening]
            if np.any(values < self.lower[index]) or np.any(values > self.upper[index]):
                raise ValueError('joint feedback outside URDF limits')
            self.feedback[side] = (values, time.monotonic()-max(0., age))
        except (ValueError, TypeError, OverflowError):
            self.feedback.pop(side, None)

    def receive_guard(self, name, msg):
        try:
            status = json.loads(msg.data)
            if (status['mode'] != self.mode or type(status['blocked']) is not bool
                    or type(status['latched']) is not bool):
                raise ValueError('unexpected guard mode or status')
            self.guard_status[name] = (status, time.monotonic())
        except (KeyError, ValueError, TypeError):
            self.guard_status.pop(name, None)

    def inputs(self, now):
        if len(self.feedback) != 2:
            raise ValueError('Both arms need fresh joint feedback')
        states, times = zip(*(self.feedback[side] for side in SIDES))
        if now-min(times) > FEEDBACK_AGE_S or abs(times[0]-times[1]) > PAIR_SKEW_S:
            raise ValueError('Joint feedback expired or left/right timestamps differ')
        if len(self.guard_status) != 2 or any(now-stamp > GUARD_AGE_S for _, stamp in self.guard_status.values()):
            raise ValueError('Both collision guards must report fresh status in the selected mode')
        for name, (status, _) in self.guard_status.items():
            if status['latched']:
                raise ValueError(f"{name}: {status.get('reason', 'latched')}")
        if any(self.count_publishers(f'/teleop/{side}_target') > 1 for side in SIDES):
            raise ValueError('Another publisher is sending arm targets')
        return np.stack(states)

    def report(self, state, detail):
        self.state, self.detail = state, detail
        self.status.publish(String(data=json.dumps({'state': state, 'detail': detail, 'mode': self.mode})))
        self.get_logger().info(f'{state}: {detail}')

    def begin(self, prepare, response):
        try:
            if self.state == 'running':
                raise ValueError('Sequence is already running')
            now = time.monotonic()
            measured = self.inputs(now)
            ready = self.poses['ready']
            if prepare:
                if np.any(np.abs(measured[:, :6]) > np.deg2rad(1.5)):
                    raise ValueError('Prepare expects both arms at the neutral joint pose')
                left_first = np.stack([ready[0], measured[1]])
                self.steps = [('raise_bowl_arm', left_first, 0.), ('ready', ready, 0.)]
            else:
                if np.any(np.abs(measured-ready) > TRACK_TOL):
                    raise ValueError('Move both arms to the ready pose or call /cereal_pour/prepare from neutral')
                self.steps = [(name, self.poses[name], self.pour_hold if name == 'tilt_65' else 0.)
                              for name in POUR_ORDER]
            self.command = measured
            self.last_tick = now
            self.step_index = 0
            self.begin_step(now)
            response.success, response.message = True, 'Sequence started'
        except ValueError as error:
            response.success, response.message = False, str(error)
        return response

    def prepare(self, _request, response):
        return self.begin(True, response)

    def start(self, _request, response):
        return self.begin(False, response)

    def stop(self, _request, response):
        self.abort('Operator stopped target streaming')
        response.success, response.message = True, 'Stopped target streaming'
        return response

    def abort(self, message):
        self.command = None
        self.report('stopped', message)

    def begin_step(self, now):
        self.settled_at = None
        name, goal, hold = self.steps[self.step_index]
        nominal_s = np.max(np.abs(goal-self.command)/self.rates)
        self.deadline = now + max(5., 3.*nominal_s + hold + 2.)
        self.report('running', name)

    def publish_targets(self):
        stamp = self.get_clock().now().to_msg()
        for side, values in zip(SIDES, self.command):
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = JOINT_NAMES
            msg.position = values.tolist()
            self.targets[side].publish(msg)

    def tick(self):
        if self.command is None:
            return
        now = time.monotonic()
        try:
            measured = self.inputs(now)
            if self.state == 'running':
                _, goal, hold = self.steps[self.step_index]
                if now > self.deadline:
                    raise ValueError(f'Timed out reaching {self.detail}')
                guards_clear = all(not status['blocked'] for status, _ in self.guard_status.values())
                if guards_clear and np.all(np.abs(measured-self.command) <= TRACK_TOL):
                    delta = goal-self.command
                    remaining = np.max(np.abs(delta)/self.rates)
                    dt = min(now-self.last_tick, TICK_S)
                    self.command = goal.copy() if remaining <= dt else self.command + delta*(dt/remaining)
                if guards_clear and np.array_equal(self.command, goal) and np.all(np.abs(measured-goal) <= GOAL_TOL):
                    if self.settled_at is None:
                        self.settled_at = now
                    elif now-self.settled_at >= SETTLE_S + hold:
                        self.step_index += 1
                        if self.step_index == len(self.steps):
                            self.report('complete', 'Both arms holding the ready pose')
                        else:
                            self.begin_step(now)
                else:
                    self.settled_at = None
            self.publish_targets()
            self.last_tick = now
        except ValueError as error:
            self.abort(str(error))


def main():
    rclpy.init()
    node = None
    try:
        node = CerealPour()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
