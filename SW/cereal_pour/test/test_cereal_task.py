"""Task coordinate semantics, IK, fail-closed alpha and loop interpolation."""
from pathlib import Path
import copy
import sys
import numpy as np
import pytest
import yaml
import pinocchio as pin

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'piper_collision_guard'))
from piper_collision_guard.cereal_planning import object_definitions,Scene,Planner
from piper_collision_guard.cereal_timeline import Timeline


def settings():return yaml.safe_load((ROOT/'config/cereal_task.yaml').read_text())


def test_bottom_centre_offsets_and_position_updates_are_used():
    spec=settings();obj=object_definitions(spec)
    np.testing.assert_allclose(obj['bowl']['grasp_world_m'],np.array(spec['bowl']['bottom_center_world_m'])+[0,.085,.035])
    np.testing.assert_allclose(obj['box']['grasp_world_m'],np.array(spec['box']['bottom_center_world_m'])+[0,0,.08])
    spec['bowl']['bottom_center_world_m'][0]+=.05
    np.testing.assert_allclose(object_definitions(spec)['bowl']['grasp_world_m']-obj['bowl']['grasp_world_m'],[.05,0,0])


def test_ik_recomputes_joint_solution_for_a_changed_target():
    scene=Scene(ROOT,settings());planner=Planner(scene);seed=np.deg2rad([21.2,91.1,-33.3,47.7,-62.9,63.5])
    state=np.zeros((2,7));state[0,:6]=seed;scene.update(state)
    frame=scene.data.oMf[scene.frames['left']];position=frame.act(scene.tcp);rotation=frame.rotation.copy()
    solutions=planner.ik('left',position+[.005,0,0],rotation,seed)
    assert solutions
    state[0,:6]=solutions[0];scene.update(state);target=scene.data.oMf[scene.frames['left']]
    np.testing.assert_allclose(target.act(scene.tcp),position+[.005,0,0],atol=1e-5)
    assert np.linalg.norm(solutions[0]-seed)>.001


def test_positive_j5_alpha_is_rejected_even_with_simultaneous_joint_motion():
    scene=Scene(ROOT,settings());state=np.zeros((2,7));state[:,:6]=np.deg2rad([0,90,0,0,69,0])
    assert not scene.valid(state)
    assert 'rear_profile_3030' in scene.last_error
    assert not scene.edge(np.zeros((2,7)),state)


def test_timeline_changes_j2_and_j5_together_and_returns_without_a_jump():
    spec=settings()
    def wp(name,q,held=(),hold=0):return dict(name=name,left_deg=q,right_deg=q,opening_mm=[0,0],held_objects=list(held),hold_s=hold)
    config=dict(version=2,settings=spec,waypoints=[wp('home',[0]*6),wp('alpha',[0,90,0,0,-70,0]),wp('home',[0]*6)])
    timeline=Timeline(config);half=timeline.segments[0][2]/2
    state,_,_=timeline.sample(half)
    np.testing.assert_allclose(np.rad2deg(state[:,[1,4]]),[[45,-35],[45,-35]])
    np.testing.assert_allclose(timeline.sample(0)[0],timeline.sample(timeline.duration)[0])
    bad=copy.deepcopy(config);bad['waypoints'].pop()
    with pytest.raises(ValueError,match='return'):Timeline(bad)


def test_negative_seventy_alpha_has_a_clear_simultaneous_entry():
    scene=Scene(ROOT,settings());state=np.zeros((2,7));state[:,:6]=np.deg2rad([0,90,0,0,-70,0])
    assert scene.edge(np.zeros((2,7)),state),scene.last_error


def test_object_size_changes_affect_collision_validation():
    spec=settings();scene=Scene(ROOT,spec)
    assert scene.valid(np.zeros((2,7))),scene.last_error
    spec['bowl']['diameter_m']=2.0;scene=Scene(ROOT,spec)
    assert not scene.valid(np.zeros((2,7)))
    assert 'bowl' in scene.last_error


def test_cli_replans_changed_yaml_and_never_replaces_output_on_failure(tmp_path):
    import subprocess
    default=yaml.safe_load((ROOT/'model/cereal_task_sequence.yaml').read_text())
    spec=settings();spec['bowl']['bottom_center_world_m'][0]-=.005
    config=tmp_path/'task.yaml';output=tmp_path/'sequence.yaml'
    config.write_text(yaml.safe_dump(spec))
    command=[sys.executable,str(ROOT/'tools/plan_cereal_task.py'),'--config',str(config),'--output',str(output)]
    result=subprocess.run(command,capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr
    planned=yaml.safe_load(output.read_text())
    original=next(w for w in default['waypoints'] if w['name']=='bowl_close')
    changed=next(w for w in planned['waypoints'] if w['name']=='bowl_close')
    assert np.linalg.norm(np.array(original['left_deg'])-changed['left_deg'])>.01
    assert subprocess.run(command+['--check'],capture_output=True).returncode==0
    saved=output.read_bytes();spec['bowl']['bottom_center_world_m'][0]=3
    config.write_text(yaml.safe_dump(spec))
    assert subprocess.run(command+['--check'],capture_output=True).returncode!=0
    assert subprocess.run(command,capture_output=True).returncode!=0
    assert output.read_bytes()==saved
