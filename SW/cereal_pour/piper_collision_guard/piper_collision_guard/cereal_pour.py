"""Replay planned object-dependent paths via the two-stage command gate."""
import json
import hashlib
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from .distance import DistanceModel, arm_state, normalize_feedback_joints
from .diagnostics import Recorder
from .segment_motion import SegmentMotion, SmoothSegmentMotion, TrackingPacer, arm_only_steps, is_alpha_pose


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


def step_rates(name, command, goal, rates, *, alpha=False):
    """Slow the boundary alpha approach without changing its target or limits."""
    result = rates.copy()
    if alpha or name in ('alpha', 'return_alpha'):
        # Keep configured speed away from the boundary. Only J5 near its
        # alpha endpoint limits the synchronized interpolation in the tail.
        remaining_j5_deg = np.rad2deg(np.max(np.abs(goal[:, 4]-command[:, 4])))
        if remaining_j5_deg <= 5.:
            result[4] = min(result[4], np.deg2rad(1.))
    return result


def read_sequence(config):
    if config.get('version') == 2:
        raise ValueError('Use the planned v2 replay branch; read_sequence only reads legacy fixed-angle sequences')
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
        self.recorder = Recorder(self)
        self.arm_status = {}
        self.driver_observations = {}
        self.last_diagnostic_sample = 0.
        share = Path(get_package_share_directory('piper_collision_guard'))
        model_share = Path(get_package_share_directory('piper_dual_description'))
        self.declare_parameter('sequence_path', str(share/'config/cereal_pour.yaml'))
        self.declare_parameter('enforce', False)
        self.declare_parameter('arms_only', False)
        self.arms_only = bool(self.get_parameter('arms_only').value)
        self.declare_parameter('joint_speed_scale', 1.0)
        self.declare_parameter('joint_target_acceleration_deg_s2', 20.)
        acceleration = float(self.get_parameter('joint_target_acceleration_deg_s2').value)
        if not np.isfinite(acceleration) or acceleration <= 0:
            raise ValueError('joint_target_acceleration_deg_s2 must be positive and finite')
        self.acceleration = np.r_[[np.deg2rad(acceleration)]*6, .02]
        joint_speed_scale = float(self.get_parameter('joint_speed_scale').value)
        if not np.isfinite(joint_speed_scale) or not 0 < joint_speed_scale <= 3.:
            raise ValueError('joint_speed_scale must be finite and in (0, 3]')
        self.mode = 'enforce' if self.get_parameter('enforce').value else 'monitor'
        config = yaml.safe_load(Path(self.get_parameter('sequence_path').value).read_text())
        if bool(config.get('settings', {}).get('execution', {}).get('arms_only', False)) != self.arms_only:
            raise ValueError('Executor arms_only mode differs from the planned sequence')
        if self.arms_only:
            if config.get('version') != 2 or any(
                    wp.get('opening_mm') != [0., 0.] for wp in config['waypoints']):
                raise ValueError('Arms-only mode requires a newly checked closed-gripper sequence')
            self.get_logger().warn('ARMS ONLY: closed empty grippers; no physical grasp/pour. Use --arms-only driver.')
        self.model = DistanceModel(model_share/'urdf/dual_piper.urdf')
        self.lower = np.array([
            np.r_[self.model.model.lowerPositionLimit[self.model.slots[s]], 0.] for s in SIDES
        ])
        self.upper = np.array([
            np.r_[self.model.model.upperPositionLimit[self.model.slots[s]],
                  self.model.max_opening[s]] for s in SIDES
        ])
        self.planned = config.get('version') == 2
        if self.planned:
            from .cereal_timeline import Timeline
            Timeline(config)  # Validate speeds, finite values and return to home.
            self.rates = np.r_[[np.deg2rad(config['settings']['motion']['joint_speed_deg_s'])]*6,
                               config['settings']['motion']['opening_speed_mm_s']/1000.]
            self.planned_steps = []
            for wp in config['waypoints']:
                values = np.array([np.r_[np.deg2rad(wp[side+'_deg']), wp['opening_mm'][i]/1000.]
                                   for i, side in enumerate(SIDES)])
                self.planned_steps.append((wp['name'], self.bounded(values), float(wp.get('hold_s', 0))))
            self.alpha_index = next(i for i, step in enumerate(self.planned_steps) if step[0] == 'alpha')
            expected = np.tile(np.deg2rad(config['settings']['alpha']['joint_deg']), (2, 1))
            if not np.allclose(self.planned_steps[self.alpha_index][1][:, :6], expected, atol=1e-8):
                raise ValueError('Planned alpha disagrees with task settings')
            self.poses = {'ready': self.planned_steps[self.alpha_index][1]}
        else:
            poses, self.rates, self.pour_hold = read_sequence(config)
            self.poses = {name: self.bounded(values) for name, values in poses.items()}

        self.rates[:6] *= joint_speed_scale
        self.feedback = {}
        self.active_rates = self.rates.copy()
        self.feedback_errors = {}
        self.guard_status = {}
        self.command = None
        self.steps = []
        self.step_index = 0
        self.settled_at = None
        self.last_tick = 0.
        self.state = 'idle'
        self.detail = 'waiting for run, home, prepare or start'
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Measurements are replaceable: never wait for retransmission of an old
        # encoder sample. Source-age and paired-arm freshness checks still apply.
        feedback_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.targets = {}
        for side in SIDES:
            self.create_subscription(JointState, f'/joint_states_{side}',
                                     lambda msg, s=side: self.receive_feedback(s, msg), feedback_qos)
            self.targets[side] = self.create_publisher(JointState, f'/teleop/{side}_target', 1)
            # Observe final gate output and the driver's SDK command echo.
            # These subscriptions never send anything to the hardware.
            for kind, topic in (('final_command', '/joint_ctrl_cmd_'),
                                ('sdk_command_echo', '/joint_states_ctrl_')):
                self.create_subscription(JointState, topic+side,
                    lambda m, s=side, k=kind: self.observe_driver(s, k, m), feedback_qos)
        for name in ('collision_guard', 'moveit_guard'):
            self.create_subscription(String, f'/{name}/status',
                                     lambda msg, n=name: self.receive_guard(n, msg), latched)
        self.status = self.create_publisher(String, '/cereal_pour/status', latched)
        self.create_service(Trigger, '/cereal_pour/prepare', self.prepare)
        self.create_service(Trigger, '/cereal_pour/run', self.run)
        self.create_service(Trigger, '/cereal_pour/home', self.home)
        self.create_service(Trigger, '/cereal_pour/start', self.start)
        self.create_service(Trigger, '/cereal_pour/stop', self.stop)
        self.create_timer(TICK_S, self.tick)
        self.create_timer(.5, self.publish_status)
        self.recorder.event('startup', {
            'sequence_path': self.get_parameter('sequence_path').value,
            'sequence_sha256': hashlib.sha256(Path(self.get_parameter('sequence_path').value).read_bytes()).hexdigest(),
            'mode': self.mode, 'joint_speed_scale': joint_speed_scale, 'arms_only': self.arms_only,
            'joint_target_acceleration_deg_s2': acceleration,
            'joint_names': JOINT_NAMES, 'logical_arm_order': list(SIDES)})
        try:
            from piper_msgs.msg import PiperStatusMsg
            from rosidl_runtime_py.convert import message_to_ordereddict
            def receive_status(side, msg):
                value = dict(message_to_ordereddict(msg))
                if self.arm_status.get(side) != value:
                    self.recorder.event('arm_status', {'side': side, 'status': value})
                self.arm_status[side] = value
            for side in SIDES:
                self.create_subscription(PiperStatusMsg, '/arm_status_'+side,
                                         lambda m, s=side: receive_status(s, m), feedback_qos)
        except ImportError:
            self.recorder.event('arm_status_unavailable', {'reason': 'piper_msgs is not in this ROS environment'})
        self.report('idle', self.detail)

    def observe_driver(self, side, kind, msg):
        self.driver_observations.setdefault(side, {})[kind] = {
            'received_monotonic_s': time.monotonic(),
            'source_stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
            'names': list(msg.name), 'position_rad_m': list(msg.position),
            'velocity': list(msg.velocity), 'effort': list(msg.effort)}

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
            q, opening = arm_state(msg.name, msg.position, side, self.model.max_opening[side], feedback=True)
            q = normalize_feedback_joints(q, self.lower[index, :6], self.upper[index, :6])
            values = np.r_[q, opening]
            if np.any(values < self.lower[index]) or np.any(values > self.upper[index]):
                raise ValueError('joint feedback outside URDF limits')
            self.feedback[side] = (values, time.monotonic()-max(0., age))
            self.feedback_errors.pop(side, None)
        except (ValueError, TypeError, OverflowError) as error:
            self.feedback.pop(side, None)
            self.feedback_errors[side] = str(error)

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
            missing = [f'{side}: {self.feedback_errors.get(side, "no message received")}'
                       for side in SIDES if side not in self.feedback]
            raise ValueError('Both arms need fresh joint feedback; '+ '; '.join(missing))
        states, times = zip(*(self.feedback[side] for side in SIDES))
        if self.arms_only and any(abs(state[6]) > .001 for state in states):
            raise ValueError('Arms-only path was checked with closed jaws; measured opening exceeds 1mm')
        if now-min(times) > FEEDBACK_AGE_S or abs(times[0]-times[1]) > PAIR_SKEW_S:
            raise ValueError(f'Joint feedback expired or left/right timestamps differ: '
                             f'left_age={now-times[0]:.3f}s, right_age={now-times[1]:.3f}s, '
                             f'skew={abs(times[0]-times[1]):.3f}s')
        if len(self.guard_status) != 2 or any(now-stamp > GUARD_AGE_S for _, stamp in self.guard_status.values()):
            raise ValueError('Both collision guards must report fresh status in the selected mode')
        for name, (status, _) in self.guard_status.items():
            if status['latched']:
                raise ValueError(f"{name}: {status.get('reason', 'latched')}")
        if any(self.count_publishers(self.targets[side].topic_name) > 1 for side in SIDES):
            raise ValueError('Another publisher is sending arm targets')
        return np.stack(states)

    def report(self, state, detail):
        self.state, self.detail = state, detail
        self.publish_status()
        self.get_logger().info(f'{state}: {detail}')
        self.recorder.event('sequence_transition', {'state': state, 'detail': detail,
            'guards': {name: value for name, (value, _) in self.guard_status.items()},
            'feedback_errors': self.feedback_errors, 'arm_status': self.arm_status,
            'driver_observations': self.driver_observations})

    def publish_status(self):
        self.status.publish(String(data=json.dumps({'state': self.state, 'detail': self.detail,
                                                   'mode': self.mode, 'arms_only': self.arms_only})))

    def begin(self, prepare, response, full=False):
        try:
            if self.state == 'running':
                raise ValueError('Sequence is already running')
            now = time.monotonic()
            measured = self.inputs(now)
            ready = self.poses['ready']
            if full and not self.planned:
                raise ValueError('Full run requires a planned v2 sequence')
            if self.planned:
                if prepare:
                    home = self.planned_steps[0][1]
                    if np.any(np.abs(measured-home) > GOAL_TOL):
                        action = 'Run' if full else 'Prepare'
                        raise ValueError(action+' requires both arms at planned home, including gripper opening')
                    self.steps = self.planned_steps[1:] if full else self.planned_steps[1:self.alpha_index+1]
                else:
                    if np.any(np.abs(measured-ready) > GOAL_TOL):
                        raise ValueError('Start requires the planned alpha pose; call prepare from home first')
                    self.steps = self.planned_steps[self.alpha_index+1:]
            elif prepare:
                if np.any(np.abs(measured[:, :6]) > np.deg2rad(1.5)):
                    raise ValueError('Prepare expects both arms at the neutral joint pose')
                left_first = np.stack([ready[0], measured[1]])
                self.steps = [('raise_bowl_arm', left_first, 0.), ('ready', ready, 0.)]
            else:
                if np.any(np.abs(measured-ready) > TRACK_TOL):
                    raise ValueError('Move both arms to the ready pose or call /cereal_pour/prepare from neutral')
                self.steps = [(name, self.poses[name], self.pour_hold if name == 'tilt_65' else 0.)
                              for name in POUR_ORDER]
            self.active_rates = self.rates.copy()
            if self.arms_only:
                before = len(self.steps)
                self.steps = arm_only_steps(self.steps, measured)
                self.recorder.event('execution_path', {'original_steps': before,
                    'execution_steps': len(self.steps), 'profile': 'quintic_straight_edges',
                    'corner_policy': 'stop; no blending', 'speed_limits_changed': False})
            self.command = measured
            self.last_tick = now
            self.step_index = 0
            self.begin_step(now, measured)
            response.success, response.message = True, 'Sequence started'
        except ValueError as error:
            response.success, response.message = False, str(error)
        return response

    def prepare(self, _request, response):
        return self.begin(True, response)

    def run(self, _request, response):
        return self.begin(True, response, full=True)

    def home(self, _request, response):
        """Explicit measured-pose return through both gates; no zero calibration.

        This interpolates joint space and stops on a blocked route. It does not
        plan an obstacle detour or recover a pose with an object held.
        """
        try:
            if self.state == 'running':
                raise ValueError('Sequence is already running')
            if not self.planned:
                raise ValueError('Home requires a planned v2 sequence')
            now = time.monotonic()
            measured = self.inputs(now)
            goal = self.planned_steps[0][1].copy()
            # Move joints first while preserving the measured gripper opening.
            joints_home = goal.copy()
            joints_home[:, 6] = measured[:, 6]
            self.steps = [('home_joints', joints_home, 0.), ('home', goal, 0.)]
            self.active_rates = np.minimum(self.rates, np.r_[[np.deg2rad(5.)]*6, .005])
            self.command = measured.copy()
            self.last_tick = now
            self.step_index = 0
            self.begin_step(now, measured)
            response.success, response.message = True, 'Home started through both collision guards'
        except ValueError as error:
            response.success, response.message = False, str(error)
        return response

    def start(self, _request, response):
        return self.begin(False, response)

    def stop(self, _request, response):
        self.abort('Operator stopped target streaming')
        response.success, response.message = True, 'Stopped target streaming'
        return response

    def abort(self, message):
        self.command = None
        self.report('stopped', message)

    def begin_step(self, now, measured):
        self.settled_at = None
        name, goal, hold = self.steps[self.step_index]
        alpha = name in ('alpha', 'return_alpha') or (
            self.planned and is_alpha_pose(goal, self.poses['ready']))
        profile_acceleration = self.acceleration*.5 if self.arms_only else self.acceleration
        self.pacer = TrackingPacer(self.active_rates, self.acceleration) if self.arms_only else None
        self.time_scale = 1.
        self.tracking_ratio = 0.
        self.tracking_wait = False
        if self.arms_only and not alpha:
            self.segment = SmoothSegmentMotion(self.command, goal, self.active_rates, profile_acceleration)
        else:
            self.segment = SegmentMotion(self.command, goal, self.active_rates, profile_acceleration, alpha=alpha)
        # Exact corners still have zero target velocity. Dwell is needed at
        # task stops, not at every route interpolation/corner waypoint.
        self.settle_duration = SETTLE_S
        if self.arms_only and not alpha and hold == 0 and name not in ('home', 'home_joints', 'return_home'):
            self.settle_duration = 0.
        self.pause_counts = {'guard': 0, 'tracking': 0}
        self.pause_reason = ''
        rates = step_rates(name, self.command, goal, self.active_rates, alpha=alpha)
        nominal_s = np.max(np.abs(goal-self.command)/rates)
        if alpha:
            nominal_s += 5.  # Allow the slower last five degrees.
        self.deadline = now + max(5., 3.*nominal_s + hold + 2.)
        self.grip_best_error = np.abs(goal[:, 6]-measured[:, 6])
        self.grip_active = self.grip_best_error > GOAL_TOL[6]
        self.grip_progress_at = np.full(2, now)
        if np.any(self.grip_active):
            # The gripper has no speed command in this driver interface.
            # Allow slower measured progress, but never wait indefinitely.
            self.deadline = max(self.deadline, now+20.+hold)
        self.last_progress_log = now-1.
        self.report('running', name)

    def progress_detail(self, measured, goal):
        result = {'step': self.detail}
        result['motion_profile'] = type(self.segment).__name__
        result['alpha_destination'] = is_alpha_pose(goal, self.poses['ready'])
        result['pause_counts'] = self.pause_counts.copy()
        result['pause_reason'] = self.pause_reason
        result['tracking_ratio'] = round(self.tracking_ratio, 4)
        result['reference_time_scale'] = round(self.time_scale, 4)
        for i, side in enumerate(SIDES):
            result[side] = {
                'goal_deg': np.rad2deg(goal[i, :6]).round(3).tolist(),
                'command_deg': np.rad2deg(self.command[i, :6]).round(3).tolist(),
                'command_rate_deg_s': np.rad2deg(self.segment.delta[i, :6]*self.segment.speed*self.time_scale).round(3).tolist(),
                'measured_deg': np.rad2deg(measured[i, :6]).round(3).tolist(),
                'gripper_goal_mm': round(float(goal[i, 6])*1000, 3),
                'gripper_command_mm': round(float(self.command[i, 6])*1000, 3),
                'gripper_measured_mm': round(float(measured[i, 6])*1000, 3),
            }
        result['guards'] = {name: status for name, (status, _) in self.guard_status.items()}
        result['driver_observations'] = self.driver_observations
        return json.dumps(result)

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
                name, goal, hold = self.steps[self.step_index]
                if now-self.last_progress_log >= 1.:
                    self.get_logger().info('motion_progress: '+self.progress_detail(measured, goal))
                    self.last_progress_log = now
                if now-self.last_diagnostic_sample >= .05:
                    self.recorder.event('sample', json.loads(self.progress_detail(measured, goal)))
                    self.last_diagnostic_sample = now
                if now > self.deadline:
                    raise ValueError(f'Timed out reaching {self.detail}; '+self.progress_detail(measured, goal))
                grip_error = np.abs(goal[:, 6]-measured[:, 6])
                for index, side in enumerate(SIDES):
                    if not self.grip_active[index] or grip_error[index] <= GOAL_TOL[6]:
                        self.grip_progress_at[index] = now
                    elif self.grip_best_error[index]-grip_error[index] >= .00025:
                        self.grip_best_error[index] = grip_error[index]
                        self.grip_progress_at[index] = now
                    elif now-self.grip_progress_at[index] > 3.:
                        raise ValueError(f'No gripper progress for 3s: {side}; '
                                         +self.progress_detail(measured, goal))
                guards_clear = all(not status['blocked'] for status, _ in self.guard_status.values())
                self.tracking_ratio = float(np.max(np.abs(measured-self.command)/TRACK_TOL))
                if self.arms_only:
                    if self.tracking_ratio > 1.:
                        self.tracking_wait = True
                    elif self.tracking_ratio <= .65:
                        self.tracking_wait = False
                    tracking = not self.tracking_wait
                else:
                    tracking = self.tracking_ratio <= 1.
                reason = '' if guards_clear and tracking else ('guard' if not guards_clear else 'tracking')
                if reason and reason != self.pause_reason:
                    self.pause_counts[reason] += 1
                self.pause_reason = reason
                if not reason:
                    # Account for moderate timer jitter without allowing a
                    # long scheduling delay to jump far along the path.
                    dt = min(max(0., now-self.last_tick), 2*TICK_S if self.arms_only else TICK_S)
                    self.time_scale = self.pacer.advance(self.tracking_ratio, dt) if self.pacer else 1.
                    self.command = self.segment.advance(dt*self.time_scale)
                else:
                    self.segment.pause()
                    if self.pacer:
                        self.pacer.pause()
                    self.time_scale = 0.
                if guards_clear and np.array_equal(self.command, goal) and np.all(np.abs(measured-goal) <= GOAL_TOL):
                    if self.settled_at is None:
                        self.settled_at = now
                    if now-self.settled_at >= self.settle_duration + hold:
                        self.step_index += 1
                        if self.step_index == len(self.steps):
                            self.report('complete', 'Finished; holding '+self.steps[-1][0])
                        else:
                            self.begin_step(now, measured)
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
