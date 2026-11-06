"""Simulation-only box editor. Saved profiles can be loaded by the robot guard."""
import copy
from pathlib import Path
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QComboBox, QDoubleSpinBox, QPushButton, QFileDialog, QMessageBox)
from .box_tuning import BoxProfile


class BoxEditor(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.profile = BoxProfile(node.get_parameter('base_description').value)
        initial = node.get_parameter('profile_path').value
        if initial: self.profile.load(initial)
        self.save_path = Path(initial).expanduser() if initial else Path.cwd()/'collision_tuning.json'
        self.pending = ''
        self.loading = False
        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = node.create_publisher(String,'/moveit_guard/tuned_description',qos)
        self.selection = node.create_publisher(String,'/moveit_guard/selected_link',qos)
        self.ack = node.create_subscription(String,'/moveit_guard/applied_description',self.acknowledged,qos)
        self.setWindowTitle('MoveIt — 링크별 충돌 박스 편집')
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel('시뮬에서 즉시 반영 · 실물에는 저장 후 재시작하여 적용'))
        selectors = QHBoxLayout()
        self.link = QComboBox();self.link.addItems(list(self.profile.links))
        self.box = QComboBox()
        selectors.addWidget(QLabel('링크'));selectors.addWidget(self.link)
        selectors.addWidget(QLabel('박스'));selectors.addWidget(self.box)
        layout.addLayout(selectors)
        grid = QGridLayout();layout.addLayout(grid);self.fields = {}
        for col,axis in enumerate(('X','Y','Z'),1):grid.addWidget(QLabel(axis),0,col)
        for row,(key,label,low,high) in enumerate([
            ('size_mm','박스 본체 크기 (mm)',1,2000),('center_mm','중심 위치 (mm)',-2000,2000),
            ('rotation_deg','회전 R/P/Y (°)',-360,360)],1):
            grid.addWidget(QLabel(label),row,0)
            values=[]
            for col in range(1,4):
                field=QDoubleSpinBox();field.setRange(low,high);field.setDecimals(2);field.setSingleStep(1.)
                grid.addWidget(field,row,col);values.append(field);field.valueChanged.connect(self.changed)
            self.fields[key]=values
        grid.addWidget(QLabel('각 면 추가 여유 (mm)'),4,0)
        self.margin=QDoubleSpinBox();self.margin.setRange(0,100);self.margin.setDecimals(2);self.margin.setSingleStep(.5)
        grid.addWidget(self.margin,4,1);self.margin.valueChanged.connect(self.changed)
        self.effective=QLabel();layout.addWidget(self.effective)
        for actions in [ [('박스 추가',self.add_box),('선택 박스 삭제',self.remove_box),('이 링크 기본값',self.reset_link)],
                         [('반대 팔 같은 링크에 복사',self.copy_other)],
                         [('설정 열기',self.open_profile),('저장',self.save_profile),('다른 이름으로 저장',self.save_as)] ]:
            buttons=QHBoxLayout();layout.addLayout(buttons)
            for title,callback in actions:
                button=QPushButton(title);button.clicked.connect(callback);buttons.addWidget(button)
        self.status=QLabel();self.status.setWordWrap(True);layout.addWidget(self.status)
        self.file_label=QLabel(str(self.save_path));self.file_label.setWordWrap(True);layout.addWidget(self.file_label)
        layout.addWidget(QLabel('같은 팔 내부 검사·Pinocchio 보호 구 설정은 변경되지 않습니다.'))
        self.debounce=QTimer(self);self.debounce.setSingleShot(True);self.debounce.timeout.connect(self.apply)
        self.link.currentTextChanged.connect(self.select_link);self.box.currentIndexChanged.connect(self.select_box)
        self.select_link();QTimer.singleShot(0,self.apply);self.resize(650,460)

    def current(self): return self.profile.links[self.link.currentText()][max(0,self.box.currentIndex())]

    def select_link(self,*_):
        self.selection.publish(String(data=self.link.currentText()))
        self.loading=True;self.box.clear()
        self.box.addItems([f'{i+1}' for i in range(len(self.profile.links[self.link.currentText()]))])
        self.loading=False;self.select_box()

    def select_box(self,*_):
        if self.loading:return
        self.loading=True;entry=self.current()
        for key,fields in self.fields.items():
            for field,value in zip(fields,entry[key]):field.setValue(value)
        self.margin.setValue(entry['margin_mm']);self.loading=False;self.update_effective()

    def update_effective(self):
        entry=self.current()
        self.effective.setText('충돌 검사 전체 크기: '+' × '.join(f"{x+2*entry['margin_mm']:.2f}" for x in entry['size_mm'])+' mm')

    def changed(self,*_):
        if self.loading:return
        entry=self.current()
        for key,fields in self.fields.items():entry[key]=[f.value() for f in fields]
        entry['margin_mm']=self.margin.value();self.update_effective()
        self.status.setText('수정됨 · MoveIt 반영 대기');self.debounce.start(250)

    def apply(self):
        try:
            self.pending=self.profile.description()
            self.publisher.publish(String(data=self.pending));self.status.setText('MoveIt에 적용 요청 중…')
        except Exception as error:self.status.setText(f'설정 오류: {error}')

    def acknowledged(self,message):
        if message.data==self.pending:
            self.status.setText('MoveIt 반영 완료 · RViz 박스와 충돌 판정에 적용됨')
        elif message.data.startswith('ERROR:'):self.status.setText(message.data)

    def add_box(self):
        entries=self.profile.links[self.link.currentText()]
        if len(entries)>=8:return
        entries.append(copy.deepcopy(self.current()));self.select_link();self.box.setCurrentIndex(len(entries)-1);self.apply()

    def remove_box(self):
        entries=self.profile.links[self.link.currentText()]
        if len(entries)==1:
            self.status.setText('링크당 박스는 최소 1개를 유지합니다. 크기·위치를 조정하세요.');return
        entries.pop(self.box.currentIndex());self.select_link();self.apply()

    def reset_link(self):
        name=self.link.currentText();self.profile.links[name]=copy.deepcopy(self.profile.defaults[name])
        self.select_link();self.apply()

    def copy_other(self):
        name=self.link.currentText();other=('right_' if name.startswith('left_') else 'left_')+name.split('_',1)[1]
        self.profile.links[other]=copy.deepcopy(self.profile.links[name]);self.apply()

    def open_profile(self):
        filename,_=QFileDialog.getOpenFileName(self,'설정 열기',str(self.save_path),'JSON (*.json)')
        if filename:
            try:
                self.profile.load(filename);self.save_path=Path(filename);self.file_label.setText(filename)
                self.select_link();self.apply()
            except Exception as error:QMessageBox.warning(self,'설정 오류',str(error))

    def save_profile(self):
        try:
            self.profile.save(self.save_path);self.file_label.setText('저장됨: '+str(self.save_path))
        except Exception as error:QMessageBox.warning(self,'저장 실패',str(error))

    def save_as(self):
        filename,_=QFileDialog.getSaveFileName(self,'설정 저장',str(self.save_path),'JSON (*.json)')
        if filename:self.save_path=Path(filename);self.save_profile()


def main():
    rclpy.init();app=QApplication(sys.argv);node=Node('moveit_box_editor')
    node.declare_parameter('base_description','');node.declare_parameter('profile_path','')
    window=BoxEditor(node);timer=QTimer();timer.timeout.connect(lambda:rclpy.spin_once(node,timeout_sec=0));timer.start(10)
    signal.signal(signal.SIGINT,lambda *_:app.quit());window.show()
    try:app.exec_()
    finally:node.destroy_node();rclpy.shutdown()
