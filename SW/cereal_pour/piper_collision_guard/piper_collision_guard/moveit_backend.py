"""Planning-only MoveIt process, isolated from every arm command endpoint."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid

import numpy as np
import yaml
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetMotionPlan, GetStateValidity

from .collision_model import load_model, load_semantic

SIDES = ('left', 'right')
NAMES = [f'{s}_joint{j}' for s in SIDES for j in range(1, 9)]


class MoveItBackend:
    def __init__(self, scene):
        self.scene, self.held = scene, ()
        self.process = self.node = self.executor = self.context = self.log = None
        self.params_path = None

    def __enter__(self):
        try:
            self.start()
        except Exception:
            self.close()
            raise
        return self

    def start(self):
        share = Path(get_package_share_directory('piper_moveit_guard'))
        executable = Path(get_package_prefix('piper_moveit_guard'))/'lib/piper_moveit_guard/cereal_moveit_planner'
        if not executable.is_file():
            raise ValueError('MoveIt planner executable missing: build the cereal workspace first')
        description, _, _ = load_model(share, 'meshes', '')
        semantic = load_semantic(share, description)
        token = uuid.uuid4().hex
        prefix = '/cereal_planner_'+token
        log_path = self.scene.root/'log/planning'/f'{token}.log'
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log = log_path.open('w')
        self.log_path = log_path
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as stream:
            yaml.safe_dump({'/**': {'ros__parameters': {
                'robot_description': description, 'robot_description_semantic': semantic,
                'service_prefix': prefix}}}, stream)
            self.params_path = Path(stream.name)
        # Separate DDS domain and unique services; this process has no command publishers.
        self.context = Context()
        self.context.init(args=[], domain_id=191)
        self.node = Node('cereal_planning_client_'+token, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.service = self.node.create_service(GetStateValidity, prefix+'/constraints', self.constraints)
        self.planner = self.node.create_client(GetMotionPlan, prefix+'/plan')
        self.validity = self.node.create_client(GetStateValidity, prefix+'/check_state')
        env = dict(os.environ, ROS_DOMAIN_ID='191', ROS_LOCALHOST_ONLY='1')
        self.process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(self.params_path)],
                                        env=env, stdout=self.log, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        deadline = time.monotonic()+20.
        while not (self.planner.service_is_ready() and self.validity.service_is_ready()):
            self.executor.spin_once(timeout_sec=.1)
            if self.process.poll() is not None or time.monotonic() > deadline:
                raise ValueError(f'MoveIt planning service did not start; see {log_path}')
        print(f'MoveIt OMPL/Bullet planner ready; log: {log_path}', flush=True)

    @staticmethod
    def message(state):
        msg = RobotState()
        msg.joint_state.name = NAMES
        msg.joint_state.position = np.column_stack((state[:, :6], state[:, 6]/2, -state[:, 6]/2)).ravel().tolist()
        msg.is_diff = False
        return msg

    def constraints(self, request, response):
        try:
            values = dict(zip(request.robot_state.joint_state.name, request.robot_state.joint_state.position))
            if not all(n in values for n in NAMES):
                response.valid = False
                return response
            state = np.array([[values[f'{s}_joint{j}'] for j in range(1, 7)]+
                              [2*values[f'{s}_joint7']] for s in SIDES])
            response.valid = bool(np.isfinite(state).all() and
                                  all(abs(values[f'{s}_joint7']+values[f'{s}_joint8']) < 1e-8 for s in SIDES) and
                                  self.scene.valid(state, self.held))
        except (ValueError, KeyError, TypeError):
            response.valid = False
        return response

    def call(self, client, request, timeout):
        future = client.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=timeout)
        if not future.done():
            client.remove_pending_request(future)
            raise ValueError(f'MoveIt service timeout; see {self.log_path}')
        try:
            response = future.result()
        except Exception as error:
            raise ValueError(f'MoveIt service error: {error}; see {self.log_path}') from error
        if response is None:
            raise ValueError('MoveIt returned no response')
        return response

    def check_state(self, state, held):
        self.held = tuple(held)
        request = GetStateValidity.Request()
        request.robot_state = self.message(state)
        response = self.call(self.validity, request, 8.)
        if not response.valid:
            pairs = ', '.join(c.contact_body_1+'/'+c.contact_body_2 for c in response.contacts)
            self.scene.last_error = 'MoveIt Bullet/feasibility rejected state: '+(pairs or self.scene.last_error)
        return response.valid

    def plan(self, start, goal, active, held):
        if not np.allclose(start[:, 6], goal[:, 6], atol=1e-9):
            raise ValueError('MoveIt arm routes require fixed gripper opening')
        self.held = tuple(held)
        for role, state in (('start', start), ('goal', goal)):
            if not self.check_state(state, held):
                raise ValueError(f'MoveIt {role} state rejected: {self.scene.last_error}')
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = 'both_arms' if len(active) == 2 else active[0]+'_arm'
        motion.planner_id = 'RRTConnectkConfigDefault'
        motion.num_planning_attempts = 1
        motion.allowed_planning_time = float(self.scene.spec['planning']['rrt_timeout_s'])
        motion.start_state = self.message(start)
        constraints = Constraints()
        for side in active:
            for j in range(6):
                item = JointConstraint()
                item.joint_name = f'{side}_joint{j+1}'
                item.position = float(goal[SIDES.index(side), j])
                item.tolerance_above = item.tolerance_below = 1e-6
                item.weight = 1.
                constraints.joint_constraints.append(item)
        motion.goal_constraints = [constraints]
        # Keep the wrist on the local branch between the start and selected
        # goal. Do not let a collision detour add an unrelated wrist sweep.
        margin = float(self.scene.spec['planning'].get('wrist_path_margin_deg', 15.))
        if not np.isfinite(margin) or margin < 0.:
            raise ValueError('wrist_path_margin_deg must be finite and nonnegative')
        path_constraints = Constraints()
        for side in active:
            arm = SIDES.index(side)
            for j in (3, 5):
                item = JointConstraint()
                item.joint_name = f'{side}_joint{j+1}'
                item.position = float((start[arm, j]+goal[arm, j])/2)
                item.tolerance_above = item.tolerance_below = float(abs(goal[arm, j]-start[arm, j])/2+np.deg2rad(margin))
                item.weight = 1.
                path_constraints.joint_constraints.append(item)
        motion.path_constraints = path_constraints
        response = self.call(self.planner, request, motion.allowed_planning_time+15.).motion_plan_response
        if response.error_code.val != 1:
            raise ValueError(f'MoveIt could not find a collision-free path (code {response.error_code.val}); '
                             f'{self.scene.last_error}; see {self.log_path}')
        trajectory = response.trajectory.joint_trajectory
        if not trajectory.points:
            raise ValueError('MoveIt returned an empty trajectory')
        allowed = {f'{s}_joint{j}': (SIDES.index(s), j-1) for s in active for j in range(1, 7)}
        if set(trajectory.joint_names) != set(allowed):
            raise ValueError('MoveIt trajectory joints differ from requested arm group')
        path = [start.copy()]
        for point in trajectory.points:
            if len(point.positions) != len(trajectory.joint_names):
                raise ValueError('Malformed MoveIt trajectory')
            state = start.copy()
            for name, value in zip(trajectory.joint_names, point.positions):
                state[allowed[name]] = value
            if not np.isfinite(state).all():
                raise ValueError('Nonfinite MoveIt trajectory')
            if np.max(np.abs(state-path[-1])) > 1e-10:
                path.append(state)
        if np.max(np.abs(path[-1]-goal)) > 1e-4:
            raise ValueError('MoveIt trajectory did not reach the requested goal')
        path[-1] = goal.copy()
        if len(path) == 1:
            path.append(goal.copy())
        return path

    def close(self):
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                try:
                    self.process.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait()
        if self.executor is not None:
            self.executor.shutdown()
        if self.node is not None:
            self.node.destroy_node()
        if self.context is not None and self.context.ok():
            self.context.shutdown()
        if self.params_path is not None:
            self.params_path.unlink(missing_ok=True)
        if self.log is not None:
            self.log.close()

    def __exit__(self, *_):
        self.close()
