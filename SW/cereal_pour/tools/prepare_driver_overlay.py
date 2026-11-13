"""Make a local driver copy with exact degree/radian conversion.

Does not import the control node, connect CAN, or edit the upstream workspace.
The launch wrapper uses this copy only on the next driver start.
"""
import argparse
import importlib.util
from pathlib import Path


def corrected_source(source, disable_gripper=False):
    feedback_count = source.count('0.017444')
    command_count = source.count('57324.840764')
    if feedback_count != 12 or command_count != 1:
        raise ValueError('Unexpected upstream driver revision; conversion overlay requires manual review '
                         f'(feedback={feedback_count}, command={command_count})')
    result = source.replace('0.017444', '(math.pi / 180.0)')
    result = result.replace('57324.840764', '(1000.0 * 180.0 / math.pi)')
    if disable_gripper:
        # Cover startup, enable services and BOTH position callbacks, including
        # upstream calls that do not consult gripper_exist.
        if 'self.piper.GripperCtrl(' not in result:
            raise ValueError('Cannot locate upstream gripper command calls')
        result = result.replace('self.piper.GripperCtrl(', 'self._cereal_ignore_gripper(')
        result = result.replace('    def GetEnableFlag(self):',
            '    def _cereal_ignore_gripper(self, *args, **kwargs):\n'
            '        """Arms-only mode: do not send gripper CAN commands."""\n'
            '        return None\n\n'
            '    def GetEnableFlag(self):', 1)
    init_anchor = '        self.__enable_flag = False'
    method_anchor = '    def GetEnableFlag(self):'
    if init_anchor not in result or method_anchor not in result:
        raise ValueError('Unexpected driver control-state layout')
    result = result.replace(init_anchor,
        '        self.declare_parameter("cereal_gripper_commands_enabled", '
        +str(not disable_gripper)+')\n'+init_anchor, 1)
    result = result.replace(init_anchor, init_anchor+'\n        self._cereal_inhibit = ""\n        self._cereal_seen_can = False', 1)
    control_check = '''    def _cereal_command_allowed(self):
        # Never reacquire CAN control after teach/control loss by continuing
        # to send MotionCtrl_2(CAN, MOVE_J). A restart is required after latch.
        if self._cereal_inhibit:
            return False
        try:
            packet = self.piper.GetArmStatus()
            status = packet.arm_status
            mode = int(status.ctrl_mode)
            teach = int(status.teach_status)
            fault = int(status.arm_status)
            error = int(status.err_code)
            age = time.time()-float(packet.time_stamp)
            if not 0 <= age <= 0.5:
                reason = "missing/stale arm status"
            elif teach != 0 or mode not in (0, 1):
                reason = "manual/teach/other control mode"
            elif self._cereal_seen_can and mode != 1:
                reason = "CAN control was lost"
            elif fault != 0 or error != 0:
                reason = "arm fault or firmware error"
            else:
                self._cereal_seen_can = self._cereal_seen_can or mode == 1
                return True
            self._cereal_inhibit = (reason+f"; can={self.can_port} ctrl={mode} teach={teach} "
                                    f"status={fault} err={error} age={age:.3f}s")
        except Exception as error:
            self._cereal_inhibit = "cannot read arm control state: "+str(error)
        self.get_logger().error("CEREAL_CONTROL_INHIBITED: "+self._cereal_inhibit+
                                "; position commands ignored; no automatic reacquisition")
        return False

'''
    result = result.replace(method_anchor, control_check+method_anchor, 1)
    for method in ('joint_callback', 'pos_callback'):
        marker = '    def '+method+'('
        start = result.index(marker)
        following = result.find('\n    def ', start+len(marker))
        if following < 0:
            following = len(result)
        block = result[start:following]
        checks = ('if self.GetEnableFlag():', 'if(self.GetEnableFlag()):')
        replaced = False
        for check in checks:
            if check in block:
                block = block.replace(check, 'if self.GetEnableFlag() and self._cereal_command_allowed():', 1)
                replaced = True
                break
        if not replaced:
            raise ValueError('Cannot attach control-state interlock to '+method)
        result = result[:start]+block+result[following:]
    compile(result, '<corrected_piper_driver>', 'exec')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--disable-gripper', action='store_true')
    args = parser.parse_args()
    spec = importlib.util.find_spec('piper.piper_ctrl_single_node')
    if spec is None or not spec.origin:
        raise RuntimeError('Cannot locate installed piper driver')
    original = Path(spec.origin).resolve()
    result = corrected_source(original.read_text(), disable_gripper=args.disable_gripper)
    package = Path(args.output)/'piper'
    package.mkdir(parents=True, exist_ok=True)
    (package/'__init__.py').write_text('from pkgutil import extend_path\n__path__ = extend_path(__path__, __name__)\n')
    (package/'piper_ctrl_single_node.py').write_text(result)
    print(f'Driver overlay: {original} -> {package}; exact conversion and latched control-state interlock', flush=True)


if __name__ == '__main__':
    main()
