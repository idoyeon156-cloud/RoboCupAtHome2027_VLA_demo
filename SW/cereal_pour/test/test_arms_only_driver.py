"""No ROS/CAN: ensure suppression covers startup and every command entry."""
import ast
import importlib.util
from pathlib import Path


def test_disabled_driver_never_calls_gripper_but_keeps_joint_commands():
    path = Path(__file__).resolve().parents[1]/'tools/prepare_driver_overlay.py'
    spec = importlib.util.spec_from_file_location('overlay', path)
    overlay = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(overlay)
    source = '''class Driver:
    def __init__(self, piper):
        self.piper = piper
        self.parameters = {}
        self.__enable_flag = False
        self.piper.GripperCtrl(0, 1000, 1, 0)
    def declare_parameter(self, name, value):
        self.parameters[name] = value
    def GetEnableFlag(self):
        return True
    def joint_callback(self):
        if self.GetEnableFlag():
            self.piper.JointCtrl(1, 2, 3, 4, 5, 6)
            self.piper.GripperCtrl(30000, 1000, 1, 0)
    def pos_callback(self):
        if(self.GetEnableFlag()):
            self.piper.GripperCtrl(10000, 1000, 1, 0)
    def enable_callback(self):
        self.piper.GripperCtrl(0, 1000, 2, 0)
'''
    source += '\n'+''.join(f'f{i} = 0.017444\n' for i in range(12))+'cmd = 57324.840764\n'

    class SDK:
        def __init__(self):
            self.joints, self.grippers = [], []
        def JointCtrl(self, *args):
            self.joints.append(args)
        def GripperCtrl(self, *args):
            self.grippers.append(args)

    for disabled in (False, True):
        result = overlay.corrected_source(source, disable_gripper=disabled)
        # Extract only the class: no real SDK imports or control node startup.
        tree = ast.parse(result)
        scope = {}
        exec(compile(ast.Module(body=[tree.body[0]], type_ignores=[]), '<fake-driver>', 'exec'), scope)
        sdk = SDK()
        driver = scope['Driver'](sdk)
        driver._cereal_command_allowed = lambda: True
        driver.joint_callback()
        driver.pos_callback()
        driver.enable_callback()
        assert sdk.joints == [(1, 2, 3, 4, 5, 6)]
        assert len(sdk.grippers) == (0 if disabled else 4)
        assert driver.parameters['cereal_gripper_commands_enabled'] is (not disabled)


def test_actual_local_driver_has_no_direct_gripper_calls_when_disabled():
    path = Path(__file__).resolve().parents[1]/'tools/prepare_driver_overlay.py'
    spec = importlib.util.spec_from_file_location('overlay', path)
    overlay = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(overlay)
    upstream = Path('/home/doyeonlee/piper_ros/src/piper/piper/piper_ctrl_single_node.py')
    if not upstream.exists():
        import pytest
        pytest.skip('Local upstream driver not installed')
    result = overlay.corrected_source(upstream.read_text(), disable_gripper=True)
    calls = [n for n in ast.walk(ast.parse(result)) if isinstance(n, ast.Call)]
    assert not any(isinstance(n.func, ast.Attribute) and n.func.attr == 'GripperCtrl' for n in calls)
