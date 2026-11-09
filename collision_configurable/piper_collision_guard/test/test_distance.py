from pathlib import Path
import numpy as np
import pytest
import yaml
from piper_collision_guard.distance import DistanceModel, Interlock, arm_state, closest

URDF = Path(__file__).resolve().parents[2] / "piper_dual_description/urdf/dual_piper.urdf"


def test_shipped_protection_configuration():
    path = Path(__file__).resolve().parents[1] / "config/protection.yaml"
    fk = DistanceModel(URDF, protection=yaml.safe_load(path.read_text()))
    assert fk.clearance(np.zeros(6), np.zeros(6)).distance == pytest.approx(.095)


def test_fk_base_offset_and_identical_orientation():
    fk = DistanceModel(URDF)
    q = np.array([0.1, 0.8, -1.1, 0.2, 0.3, 0.1])
    points = fk.points(q, q)
    np.testing.assert_allclose(points['left'][0]-points['right'][0],
                               np.tile([0, 0.175, 0], (len(points['left'][0]), 1)), atol=1e-12)


def test_named_messages_are_reordered_and_never_fill_missing_joints_with_zero():
    q, opening = arm_state(['gripper', 'joint6', 'joint5', 'joint4', 'joint3', 'joint2', 'joint1'],
                            [0.04, 6, 5, 4, 3, 2, 1], 'left')
    np.testing.assert_equal(q, [1, 2, 3, 4, 5, 6])
    assert opening == 0.04
    for names, values in [(['joint1'], [0]), (['joint1']*7, [0]*7),
                          ([f'joint{i}' for i in range(1, 8)], [float('nan')]*7)]:
        with pytest.raises(ValueError):
            arm_state(names, values, 'left')


def test_nearest_pair_search_includes_cross_indices():
    c = closest(np.array([[0.,0.,0.], [10.,0.,0.]]),
                np.array([[10.01,0.,0.], [30.,0.,0.]]), ['L0','L1'], ['R0','R1'])
    assert c.distance == pytest.approx(0.01)
    assert (c.left_name, c.right_name) == ('L1', 'R0')


def test_volume_uses_surface_gap_and_can_change_the_limiting_pair():
    c = closest(np.array([[0.,0.,0.]]), np.array([[.18,0.,0.], [.25,0.,0.]]),
                ['L'], ['thin', 'thick'], [.04], [.04, .20])
    assert c.right_name == 'thick'
    assert c.center_distance == pytest.approx(.25)
    assert c.distance == pytest.approx(.01)


def test_protective_spheres_in_world_and_invalid_radius_rejected():
    protection = {s: {'radii_m': [.04]*7} for s in ('left','right')}
    fk = DistanceModel(URDF, protection=protection)
    c = fk.clearance(np.zeros(6), np.zeros(6))
    assert c.center_distance == pytest.approx(.175)
    assert c.distance == pytest.approx(.095)
    protection['left']['radii_m'][0] = -.01
    with pytest.raises(ValueError):
        DistanceModel(URDF, protection=protection)


def test_interlock_does_not_auto_resume_and_rejects_stale_skewed_state():
    lock = Interlock(.1, .13, .15, .05)
    lock.observe(.09)
    lock.observe(.3)
    assert lock.latched
    assert not lock.reset(.12, True)
    assert not lock.reset(.3, False)
    assert lock.reset(.3, True)
    assert not lock.fresh({'left': 1.0, 'right': 1.1}, 1.12)
    assert not lock.fresh({'left': 1.0, 'right': 1.01}, 1.3)
    assert lock.fresh({'left': 1.0, 'right': 1.01}, 1.05)


def test_invalid_model_configuration_and_sampling_budget_fail_closed():
    fk = DistanceModel(URDF)
    with pytest.raises(ValueError):
        fk.clearance(np.array([0, -1, 0, 0, 0, 0]), np.zeros(6))
    with pytest.raises(ValueError):
        fk.path_clearance(np.zeros(6), np.zeros(6), np.array([1,0,0,0,0,0]), np.zeros(6), .01, 10)


def test_limit_interpolation_accepts_roundoff_but_not_real_limit_violation():
    fk = DistanceModel(URDF)
    q = np.zeros(6)
    q[5] = np.nextafter(-2.0944, -np.inf)
    fk.clearance(q, np.zeros(6))
    q[5] = -2.094401
    with pytest.raises(ValueError):
        fk.clearance(q, np.zeros(6))


def test_path_checks_midpoints_even_when_endpoints_are_clear():
    # A robot-independent crossing trajectory tests the actual interpolator:
    # left goes -1 -> +1, right remains at 0. Both endpoints have distance 1.
    fk = DistanceModel(URDF, 0)
    fk.configuration = lambda left, right: np.r_[left, right]
    fk.clearance = lambda left, right: closest(np.array([[left[0],0.,0.]]),
                                              np.array([[right[0],0.,0.]]), ['L'], ['R'])
    first = np.array([-1.,0,0,0,0,0]); last = -first; right = np.zeros(6)
    assert fk.clearance(first, right).distance == 1
    assert fk.clearance(last, right).distance == 1
    assert fk.path_clearance(first, right, last, right, .1, 30).distance == pytest.approx(0.)
