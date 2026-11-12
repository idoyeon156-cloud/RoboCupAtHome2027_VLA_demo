"""Exercise user-entered units without starting a robot driver."""
import math
import os
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest


def test_degree_input_preserves_sdk_pose_and_publishes_ros_units():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    from python_qt_binding.QtWidgets import QApplication
    from piper_collision_guard.degree_gui import DegreeControls

    app = QApplication.instance() or QApplication([])
    description = (Path(__file__).resolve().parents[2] /
                   'piper_dual_description/urdf/dual_piper.urdf').read_text()
    joints = {}
    for joint in ET.fromstring(description).findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        limit = joint.find('limit')
        joints[joint.get('name')] = {'min': float(limit.get('lower')),
                                   'max': float(limit.get('upper')), 'position': 0.0}
    publisher = SimpleNamespace(joint_list=list(joints), free_joints=joints,
                                set_robot_description_update_cb=lambda cb: None)
    window = DegreeControls(publisher, description)
    try:
        degrees = [-.50, 171.73, -168.33, -.85, 18.54, -75.98]
        for i, angle in enumerate(degrees, 1):
            name = f'left_joint{i}'
            window.controls[name][0].setValue(angle)
            assert joints[name]['position'] == pytest.approx(math.radians(angle))
            assert window.controls[name][0].value() == pytest.approx(angle)
        window.controls['left_joint7'][0].setValue(12.5)
        assert joints['left_joint7']['position'] == pytest.approx(.0125)
        assert all(joints[f'right_joint{i}']['position'] == 0 for i in range(1, 7))
        window.zero()
        assert all(j['position'] == 0 for j in joints.values())
    finally:
        window.close()
        app.processEvents()
