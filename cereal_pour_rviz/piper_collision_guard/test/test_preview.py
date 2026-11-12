from pathlib import Path
import numpy as np
import pytest
import yaml
from piper_collision_guard.preview import PreviewState, placed_description

ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT/'piper_dual_description/urdf/dual_piper.urdf'
PROTECTION = yaml.safe_load((ROOT/'piper_collision_guard/config/protection_mesh_cover.yaml').read_text())
NEAR_LEFT = [.2472277, .97563748, -.60250523, .19306071, -.55266473, .28215612]
NEAR_RIGHT = [.63469996, .72964917, -.62731257, .34529583, -.00143087, -.37091546]


def request(state, near=False):
    values = dict(zip(state.names, [0.]*len(state.names)))
    if near:
        for side, q in [('left',NEAR_LEFT), ('right',NEAR_RIGHT)]:
            for i, v in enumerate(q, 1):
                values[f'{side}_joint{i}'] = v
    state.update(list(values), list(values.values()))


def test_visual_preview_holds_accepted_but_shows_unsafe_requested_pose():
    state = PreviewState(placed_description(URDF, .20), PROTECTION)
    request(state)
    assert state.clearance().distance > state.stop
    accepted = state.accepted.copy()
    request(state, True)
    assert state.blocked and state.clearance().distance < .02
    np.testing.assert_equal(state.accepted, accepted)
    assert np.any(state.requested != state.accepted)
    request(state)
    assert not state.blocked  # demo recovery is deliberately not a real stop latch


def test_inspect_mode_shows_even_the_unsafe_pose():
    state = PreviewState(URDF, PROTECTION, block_unsafe=False)
    request(state, True)
    assert state.blocked
    np.testing.assert_equal(state.accepted, state.requested)


def test_preview_rejects_nan_missing_and_out_of_limit_joints():
    state = PreviewState(URDF, PROTECTION)
    original = state.accepted.copy()
    for names, positions in [([],[]), (state.names, [float('nan')]*len(state.names)),
                             (state.names, [100.]*len(state.names))]:
        with pytest.raises(ValueError):
            state.update(names, positions)
    np.testing.assert_equal(state.accepted, original)


def test_spacing_override_updates_fk_and_rejects_bad_values():
    state = PreviewState(placed_description(URDF, .18), PROTECTION)
    default = PreviewState(URDF, PROTECTION)
    assert state.clearance().distance - default.clearance().distance == pytest.approx(.005)
    for invalid in [0, -1, float('nan')]:
        with pytest.raises(ValueError):
            placed_description(URDF, invalid)


def test_rviz_mesh_urls_are_encoded_and_files_exist():
    import xml.etree.ElementTree as ET
    from urllib.parse import unquote, urlparse
    description = placed_description(URDF, .175, mesh_root=URDF.parents[1])
    meshes = ET.fromstring(description).findall('.//mesh')
    assert len(meshes) == 44
    for mesh in meshes:
        uri = mesh.get('filename')
        assert uri.startswith('file:///') and ' ' not in uri
        assert uri.isascii()
        assert Path(unquote(urlparse(uri).path)).is_file()


def test_joint_centre_eight_cm_comparison_allows_fifteen_degrees_and_blocks_below_limit():
    state = PreviewState(URDF, PROTECTION, joint_center_limit_m=.08)
    assert state.fk.link_samples == 0
    assert state.clearance().center_distance == pytest.approx(.175)
    for degrees, blocked in [(15, False), (20, False), (20.1, True)]:
        values = dict.fromkeys(state.names, 0.)
        values['left_joint1'] = np.deg2rad(degrees)
        state.update(list(values), list(values.values()))
        assert state.blocked == blocked
        assert state.clearance().distance == pytest.approx(state.clearance().center_distance-.08)
    for invalid in [-.08, float('nan')]:
        with pytest.raises(ValueError):
            PreviewState(URDF, PROTECTION, joint_center_limit_m=invalid)


def test_nine_cm_demo_and_real_guard_profile_use_same_spheres_and_gap():
    from piper_collision_guard.distance import DistanceModel
    real = yaml.safe_load((ROOT/'piper_collision_guard/config/protection.yaml').read_text())
    config = yaml.safe_load((ROOT/'piper_collision_guard/config/guard.yaml').read_text())['piper_collision_guard']['ros__parameters']
    guard = DistanceModel(URDF, link_samples=config['link_samples'], protection=real)
    state = PreviewState(URDF, PROTECTION, joint_center_limit_m=.09)
    assert state.stop == pytest.approx(.01)
    assert config['stop_clearance_m'] == pytest.approx(.01)
    assert not config['enforce']
    for degrees, blocked in [(15,False),(17,False),(18,True)]:
        values = dict.fromkeys(state.names,0.)
        values['left_joint1'] = np.deg2rad(degrees)
        state.update(list(values),list(values.values()))
        assert state.blocked == blocked
        left = np.array([np.deg2rad(degrees),0,0,0,0,0])
        c = guard.clearance(left,np.zeros(6))
        assert (c.distance <= config['stop_clearance_m']) == blocked
        assert c.distance == pytest.approx(state.clearance().distance)
