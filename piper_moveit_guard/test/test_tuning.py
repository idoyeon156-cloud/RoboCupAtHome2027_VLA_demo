from pathlib import Path
import copy
import math
import xml.etree.ElementTree as ET
import pytest
from piper_collision_guard.box_tuning import BoxProfile, tuned_description

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'piper_moveit_guard/config/dual_piper_boxes.urdf'


def test_saved_profile_preserves_kinematics_and_edits_only_named_collision(tmp_path):
    profile=BoxProfile(BASE.read_text())
    original=ET.fromstring(profile.description())
    entry=profile.links['left_link2'][0]
    entry.update(size_mm=[200.,40.,50.],center_mm=[80.,-5.,0.],rotation_deg=[0.,0.,30.],margin_mm=2.)
    profile.links['left_link2'].append(copy.deepcopy(entry))
    profile.links['left_link2'][1]['center_mm'][0]=180.
    path=profile.save(tmp_path/'collision_tuning.json')
    tuned=ET.fromstring(tuned_description(BASE,str(path)))
    assert [ET.tostring(x).strip() for x in original.findall('joint')]==[ET.tostring(x).strip() for x in tuned.findall('joint')]
    for side in ['left_link1','right_link2']:
        assert ET.tostring(original.find(f"link[@name='{side}']")).strip()==ET.tostring(tuned.find(f"link[@name='{side}']")).strip()
    collisions=tuned.find("link[@name='left_link2']").findall('collision')
    assert len(collisions)==2
    assert list(map(float,collisions[0].find('geometry/box').get('size').split()))==pytest.approx([.204,.044,.054])
    assert float(collisions[0].find('origin').get('rpy').split()[2])==pytest.approx(math.pi/6)


@pytest.mark.parametrize('bad',[0.,-1.,float('nan'),float('inf')])
def test_invalid_sizes_cannot_be_saved_or_loaded(tmp_path,bad):
    profile=BoxProfile(BASE.read_text());profile.links['left_link2'][0]['size_mm'][0]=bad
    with pytest.raises(ValueError):profile.save(tmp_path/'invalid.json')
    assert not (tmp_path/'invalid.json').exists()


def test_gui_edits_copies_and_saves_without_robot(tmp_path,monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen')
    from types import SimpleNamespace
    from python_qt_binding.QtWidgets import QApplication
    from piper_collision_guard.box_editor import BoxEditor
    class FakeNode:
        def get_parameter(self,name):return SimpleNamespace(value=BASE.read_text() if name=='base_description' else '')
        def create_publisher(self,*_):return SimpleNamespace(publish=lambda m:None)
        def create_subscription(self,*_):return None
    app=QApplication.instance() or QApplication([])
    window=BoxEditor(FakeNode());window.show();app.processEvents()
    try:
        window.link.setCurrentText('left_link2')
        window.fields['size_mm'][0].setValue(200.)
        window.fields['center_mm'][0].setValue(80.)
        window.margin.setValue(2.)
        window.copy_other()
        assert window.profile.links['right_link2']==window.profile.links['left_link2']
        window.add_box();assert len(window.profile.links['left_link2'])==2
        window.remove_box();assert len(window.profile.links['left_link2'])==1
        window.save_path=tmp_path/'profile.json';window.save_profile()
        loaded=BoxProfile(BASE.read_text());loaded.load(window.save_path)
        assert loaded.links['left_link2'][0]['size_mm'][0]==200.
        window.grab().save('/tmp/piper_box_editor.png')
    finally:window.close();app.processEvents()
