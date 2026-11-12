"""Demo-only controls: degrees/mm on screen, radians/metres in JointState."""
import math
import signal
import sys
import xml.etree.ElementTree as ET

import rclpy
from joint_state_publisher.joint_state_publisher import JointStatePublisher
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication, QDoubleSpinBox, QGridLayout, QLabel, QPushButton,
    QScrollArea, QSlider, QVBoxLayout, QWidget,
)


class DegreeControls(QWidget):
    def __init__(self, jsp, description):
        super().__init__()
        self.jsp = jsp
        self.types = {j.get('name'): j.get('type') for j in ET.fromstring(description).findall('joint')}
        self.controls = {}
        self.setWindowTitle('Dual Piper — joint angles (°) / fingers (mm)')
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel('Arm joints: degrees (°)   |   Finger displacement: mm'))
        reset = QPushButton('Zero all joints (0° / 0 mm)')
        reset.clicked.connect(self.zero)
        layout.addWidget(reset)
        self.grid = QGridLayout()
        body = QWidget()
        body.setLayout(self.grid)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(body)
        layout.addWidget(scroll)
        self.resize(1080, 510)
        self.setMinimumSize(920, 460)
        jsp.set_robot_description_update_cb(self.initialize)
        self.initialize()

    def initialize(self):
        self.controls.clear()
        while self.grid.count():
            item = self.grid.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self.grid.setHorizontalSpacing(12)
        self.grid.setVerticalSpacing(12)
        for side, title in enumerate(('왼팔 (LEFT)', '오른팔 (RIGHT)')):
            header = QLabel(title)
            header.setStyleSheet('font-weight: bold; font-size: 16px; padding: 6px;')
            self.grid.addWidget(header, 0, side * 4, 1, 3)
            self.grid.setColumnStretch(side * 4 + 1, 1)
        self.grid.setColumnMinimumWidth(3, 28)
        for name in self.jsp.joint_list:
            if name not in self.jsp.free_joints:
                continue
            joint = self.jsp.free_joints[name]
            factor = 1000.0 if self.types[name] == 'prismatic' else 180.0 / math.pi
            unit = 'mm' if self.types[name] == 'prismatic' else '°'
            side = 0 if name.startswith('left_') else 1
            row = sum(n.startswith('left_' if side == 0 else 'right_') for n in self.controls)
            box = QDoubleSpinBox()
            box.setDecimals(3)
            box.setRange(joint['min'] * factor, joint['max'] * factor)
            box.setSingleStep(0.1)
            box.setSuffix(' ' + unit)
            box.setKeyboardTracking(False)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 100000)
            label = QLabel(name.split('_', 1)[-1])
            label.setToolTip(name)
            box.setMinimumWidth(110)
            slider.setMinimumWidth(210)
            self.grid.addWidget(label, row + 1, side * 4)
            self.grid.addWidget(slider, row + 1, side * 4 + 1)
            self.grid.addWidget(box, row + 1, side * 4 + 2)
            self.controls[name] = (box, slider, factor)
            box.valueChanged.connect(lambda value, n=name: self.set_position(n, value / self.controls[n][2]))
            slider.valueChanged.connect(lambda value, n=name: self.from_slider(n, value))
            self.set_position(name, joint['position'])

    def from_slider(self, name, tick):
        joint = self.jsp.free_joints[name]
        self.set_position(name, joint['min'] + (joint['max'] - joint['min']) * tick / 100000)

    def set_position(self, name, value):
        joint = self.jsp.free_joints[name]
        value = min(joint['max'], max(joint['min'], value))
        joint['position'] = value
        box, slider, factor = self.controls[name]
        box.blockSignals(True)
        slider.blockSignals(True)
        box.setValue(value * factor)
        slider.setValue(round((value - joint['min']) / (joint['max'] - joint['min']) * 100000))
        box.blockSignals(False)
        slider.blockSignals(False)

    def zero(self):
        for name in self.controls:
            self.set_position(name, 0.0)


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    jsp = JointStatePublisher(None)
    description = jsp.get_parameter('robot_description').value
    window = DegreeControls(jsp, description)
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(jsp, timeout_sec=0))
    timer.start(10)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    window.show()
    try:
        app.exec_()
    finally:
        jsp.destroy_node()
        rclpy.shutdown()
