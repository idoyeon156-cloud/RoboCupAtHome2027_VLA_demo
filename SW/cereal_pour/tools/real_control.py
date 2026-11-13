"""Bounded real-session checks and explicit service calls; no joint publisher.

check/status only observe. ready explicitly resets both guards; run resets once
before requesting the whole home-to-home sequence. home/prepare/start explicitly
request motion. There is no retry or reset during motion.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from piper_collision_guard.distance import DistanceModel, arm_state, normalize_feedback_joints


class Session(Node):
    def __init__(self):
        super().__init__('cereal_real_control')
        self.samples, self.errors, self.statuses = {}, {}, {}
        share = Path(get_package_share_directory('piper_dual_description'))
        self.model = DistanceModel(share/'urdf/dual_piper.urdf')
        feedback_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subs = []
        for side in ('left', 'right'):
            self.subs.append(self.create_subscription(JointState, f'/joint_states_{side}',
                             lambda m, s=side: self.feedback(s, m), feedback_qos))
        for name in ('collision_guard', 'moveit_guard', 'cereal_pour'):
            self.subs.append(self.create_subscription(String, f'/{name}/status',
                             lambda m, n=name: self.status(n, m), status_qos))

    def feedback(self, side, msg):
        try:
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            if stamp <= 0 or not -.05 <= age <= .15:
                raise ValueError(f'source age={age:.3f}s (limit 0.15s)')
            q, opening = arm_state(msg.name, msg.position, side, self.model.max_opening[side], feedback=True)
            slots = self.model.slots[side]
            normalize_feedback_joints(q, self.model.model.lowerPositionLimit[slots],
                                      self.model.model.upperPositionLimit[slots])
            self.samples[side] = (time.monotonic()-max(age, 0.), q, opening)
            self.errors.pop(side, None)
        except (ValueError, TypeError, OverflowError) as error:
            self.samples.pop(side, None)
            self.errors[side] = str(error)

    def status(self, name, msg):
        try:
            self.statuses[name] = (time.monotonic(), json.loads(msg.data))
        except (ValueError, TypeError):
            self.statuses.pop(name, None)

    def health_error(self):
        now = time.monotonic()
        for side in ('left', 'right'):
            if side not in self.samples:
                return f'{side}: {self.errors.get(side, "no joint feedback; check driver and CAN")}'
            age = now-self.samples[side][0]
            if age > .15:
                return f'{side}: feedback age={age:.3f}s > 0.15s'
        skew = abs(self.samples['left'][0]-self.samples['right'][0])
        if skew > .05:
            return f'left/right timestamp skew={skew:.3f}s > 0.05s'
        for name in ('collision_guard', 'moveit_guard', 'cereal_pour'):
            if name not in self.statuses or now-self.statuses[name][0] > (.30 if name != 'cereal_pour' else 1.5):
                return f'{name}: no fresh status; check run_cereal_real.sh terminal'
            if self.statuses[name][1].get('mode') != 'enforce':
                return f'{name}: expected enforce mode'
        return ''

    def check(self):
        print('양팔 피드백·시각 차이·패키지 상태를 3초 연속 검사합니다. 이동 명령 없음.', flush=True)
        deadline, stable, last_error = time.monotonic()+12., None, ''
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.01)
            error = self.health_error()
            if error:
                stable = None
                if error != last_error:
                    print('대기:', error, flush=True)
                last_error = error
            else:
                stable = stable or time.monotonic()
                if time.monotonic()-stable >= 3.:
                    self.check_driver_gripper_mode()
                    for side, (_, q, opening) in self.samples.items():
                        print(side, 'deg=', np.rad2deg(q).round(2).tolist(), 'opening_mm=', round(opening*1000, 2))
                    print('PASS: 양팔 피드백·시각·실행 모드 정상 (안전장치 잠금 해제 여부는 별도)', flush=True)
                    return
        raise RuntimeError('검사 실패: '+(self.health_error() or last_error)+'. 이동을 요청하지 않았습니다.')

    def check_driver_gripper_mode(self):
        if not self.statuses['cereal_pour'][1].get('arms_only', False):
            return
        for side in ('left', 'right'):
            service = f'/piper_{side}_ctrl_node/get_parameters'
            client = self.create_client(GetParameters, service)
            try:
                if not client.wait_for_service(timeout_sec=3.):
                    raise RuntimeError(service+': cannot verify arms-only driver')
                request = GetParameters.Request(names=['cereal_gripper_commands_enabled'])
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=3.)
                response = future.result() if future.done() else None
                if (response is None or len(response.values) != 1 or
                        response.values[0].type != ParameterType.PARAMETER_BOOL or
                        response.values[0].bool_value):
                    raise RuntimeError(side+': restart driver with --arms-only; gripper suppression not confirmed')
            finally:
                self.destroy_client(client)
        print('ARMS ONLY: 양팔 드라이버의 그리퍼 명령 비활성화 확인', flush=True)

    def call(self, service):
        client = self.create_client(Trigger, service)
        try:
            if not client.wait_for_service(timeout_sec=3.):
                raise RuntimeError(service+': service unavailable; check package terminal')
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.)
            if not future.done() or future.result() is None:
                raise RuntimeError(service+': response timeout; outcome unknown. Do not repeat motion blindly.')
            result = future.result()
            print(service, result.success, result.message, flush=True)
            if not result.success:
                raise RuntimeError(result.message)
        finally:
            self.destroy_client(client)

    def verify_reset(self):
        """Observe an explicit reset; never reset again automatically."""
        start = time.monotonic()
        stable = None
        while time.monotonic()-start < 6.:
            rclpy.spin_once(self, timeout_sec=.01)
            if time.monotonic()-start < .3:
                continue  # Drain statuses queued before the service replies.
            error = self.health_error()
            if error:
                raise RuntimeError('리셋 후 상태 검사 실패: '+error)
            for name in ('collision_guard', 'moveit_guard'):
                status = self.statuses[name][1]
                if status.get('latched') is not False:
                    raise RuntimeError(name+': 리셋 후 다시 잠김: '+json.dumps(status, ensure_ascii=False))
            stable = stable or time.monotonic()
            if time.monotonic()-stable >= 3.:
                print('READY: 두 안전장치 잠금 해제 후 3초 연속 유지 확인', flush=True)
                return
        raise RuntimeError('리셋 후 잠금 해제 상태 확인 시간 초과')

    def wait_complete(self):
        deadline, last = time.monotonic()+180., None
        # Discard the latched result of the previous run before observing this run.
        self.statuses.pop('cereal_pour', None)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            item = self.statuses.get('cereal_pour')
            if item:
                status = item[1]
                if status != last:
                    print(status, flush=True)
                    last = status
                if status.get('state') == 'complete':
                    return
                if status.get('state') == 'stopped':
                    raise RuntimeError(status.get('detail', 'sequence stopped'))
        self.call('/cereal_pour/stop')
        raise RuntimeError('180s completion timeout; stopped target streaming')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['check', 'ready', 'run', 'home', 'prepare', 'start', 'stop', 'status'])
    args = parser.parse_args()
    hold = Path(__file__).resolve().parents[1]/'REAL_MOTION_HOLD_KO.md'
    if args.action in ('ready', 'run', 'home', 'prepare', 'start') and hold.exists():
        print('ERROR: 실물 자동 동작 중단 상태입니다. '+str(hold), flush=True)
        return 1
    rclpy.init()
    node = Session()
    try:
        if args.action == 'stop':
            node.call('/cereal_pour/stop')
        elif args.action == 'status':
            deadline = time.monotonic()+3.
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            print(json.dumps({n: s for n, (_, s) in node.statuses.items()}, ensure_ascii=False, indent=2))
        else:
            node.check()
            if args.action in ('ready', 'run'):
                node.call('/collision_guard/reset')
                node.call('/moveit_guard/reset')
                node.verify_reset()
            if args.action in ('run', 'home', 'prepare', 'start'):
                for name in ('collision_guard', 'moveit_guard'):
                    status = node.statuses[name][1]
                    if status.get('latched') is not False:
                        raise RuntimeError(name+': latched; first_stop_reason='
                                           +str(status.get('first_stop_reason'))+'; reason='
                                           +str(status.get('reason'))+'; feedback_errors='
                                           +str(status.get('last_feedback_errors', {})))
                node.call('/cereal_pour/'+args.action)
                node.wait_complete()
    except KeyboardInterrupt:
        print('상태 확인을 중단했습니다. 로봇의 실제 정지를 뜻하지 않습니다. '
              '별도 창에서 bash run_real_control.sh stop 을 실행하세요.', flush=True)
        return 1
    except RuntimeError as error:
        print('ERROR:', error, flush=True)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
