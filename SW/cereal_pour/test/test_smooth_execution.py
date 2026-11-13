"""Pure numerical tests; never imports ROS or connects to a controller."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

path = Path(__file__).resolve().parents[1]/'piper_collision_guard/piper_collision_guard/segment_motion.py'
spec = importlib.util.spec_from_file_location('motion', path)
motion = importlib.util.module_from_spec(spec)
spec.loader.exec_module(motion)


def test_smooth_profile_stays_on_edge_and_respects_velocity_acceleration():
    start = np.zeros((2, 7))
    goal = np.array([[1., -.7, .2, 0, .1, -.3, 0], [.3, .5, -.6, 0, 0, .2, 0]])
    rates = np.r_[np.full(6, .4), .02]
    acceleration = np.r_[np.full(6, .35), .02]
    segment = motion.SmoothSegmentMotion(start, goal, rates, acceleration)
    dt = segment.duration/10000
    points = [start]
    for _ in range(10001):
        points.append(segment.advance(dt))
        assert 0 <= segment.progress <= 1
        assert np.allclose(points[-1], start+(goal-start)*segment.progress)
    points = np.array(points)
    velocity = np.diff(points, axis=0)/dt
    accel = np.diff(velocity, axis=0)/dt
    assert np.all(np.abs(velocity) <= rates+1e-7)
    assert np.all(np.abs(accel) <= acceleration+1e-6)
    assert np.array_equal(points[-1], goal)
    assert segment.speed == 0


def test_pause_restarts_from_last_command_on_same_edge_without_jump():
    start, goal = np.zeros((2, 7)), np.ones((2, 7))
    segment = motion.SmoothSegmentMotion(start, goal, np.ones(7), np.ones(7))
    current = segment.advance(.7)
    segment.pause()
    assert segment.speed == 0
    assert np.array_equal(segment.advance(0.), current)
    resumed = segment.advance(.01)
    assert np.all(resumed >= current)
    assert np.all(resumed <= goal)
    assert np.max(resumed-current) < 1e-5


def test_zero_distance_and_invalid_timestep():
    q = np.zeros((2, 7))
    segment = motion.SmoothSegmentMotion(q, q, np.ones(7), np.ones(7))
    assert np.array_equal(segment.advance(.02), q)
    with pytest.raises(ValueError):
        segment.advance(float('nan'))


def test_reversed_task_label_cannot_hide_alpha_destination():
    alpha = np.zeros((2, 7))
    alpha[:, :6] = np.deg2rad([0, 90, 0, 0, -70, 0])
    reversed_open = alpha.copy()
    reversed_open[:, 6] = [.03, .08]
    assert motion.is_alpha_pose(reversed_open, alpha)
    shifted = alpha.copy()
    shifted[1, 4] += np.deg2rad(1.)
    assert not motion.is_alpha_pose(shifted, alpha)


def test_executor_uses_destination_for_boundary_speed_and_deadline():
    # Extract the actual methods without importing ROS or starting any node.
    import ast
    from types import SimpleNamespace
    source = ast.parse((path.parent/'cereal_pour.py').read_text())
    rates_method = next(n for n in source.body if isinstance(n, ast.FunctionDef) and n.name == 'step_rates')
    cls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'CerealPour')
    begin = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'begin_step')
    scope = dict(np=np, SegmentMotion=motion.SegmentMotion, SmoothSegmentMotion=motion.SmoothSegmentMotion,
                 TrackingPacer=motion.TrackingPacer,
                 is_alpha_pose=motion.is_alpha_pose, SETTLE_S=.2, GOAL_TOL=np.ones(7)*.002)
    exec(compile(ast.Module(body=[rates_method, begin], type_ignores=[]), '<executor-test>', 'exec'), scope)
    goal = np.zeros((2, 7)); goal[:, :6] = np.deg2rad([0, 90, 0, 0, -70, 0])
    start = goal.copy(); start[:, 4] += np.deg2rad(10.)
    dummy = SimpleNamespace(steps=[('return_bowl_open', goal, 0.)], step_index=0,
        command=start, active_rates=np.r_[np.full(6, np.deg2rad(22.5)), .02],
        acceleration=np.r_[np.full(6, np.deg2rad(20)), .02], arms_only=True,
        planned=True, poses={'ready': goal}, report=lambda *args: None)
    scope['begin_step'](dummy, 100., start)
    assert isinstance(dummy.segment, motion.SegmentMotion)
    assert dummy.segment.tail_start < 1
    assert dummy.segment.tail_rate <= np.deg2rad(1.)/np.deg2rad(10.)+1e-12
    assert dummy.settle_duration == .2
    assert dummy.deadline > 110.


def test_pacing_preserves_velocity_and_acceleration_limits_during_slowdowns():
    rates = np.full(7, .4)
    acceleration = np.full(7, .35)
    segment = motion.SmoothSegmentMotion(np.zeros((2, 7)), np.ones((2, 7)), rates, acceleration*.5)
    pacer = motion.TrackingPacer(rates, acceleration)
    dt = .002
    points = [segment.position.copy()]
    factors = [pacer.factor]
    for i in range(18000):
        ratio = .8 if (i//750) % 2 else .1
        scale = pacer.advance(ratio, dt)
        points.append(segment.advance(dt*scale))
        factors.append(pacer.factor)
    velocity = np.diff(points, axis=0)/dt
    accel = np.diff(velocity, axis=0)/dt
    assert np.max(np.abs(np.diff(factors))) <= pacer.slew*dt+1e-12
    assert np.all(np.abs(velocity) <= rates+1e-6)
    assert np.all(np.abs(accel) <= acceleration+1e-4)
    assert np.array_equal(points[-1], np.ones((2, 7)))


def test_pacer_recovers_gradually_after_hard_stop_and_rejects_invalid_inputs():
    pacer = motion.TrackingPacer(np.ones(7), np.ones(7))
    pacer.pause()
    scale = pacer.advance(0., .02)
    assert scale < .02
    assert 0 < pacer.factor <= pacer.slew*.02
    with pytest.raises(ValueError):
        pacer.advance(float('nan'), .02)


def test_near_collinear_planner_shortcut_requires_collision_revalidation():
    import ast
    import time
    from types import SimpleNamespace
    source = ast.parse((path.parent/'cereal_planning.py').read_text())
    planner = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'Planner')
    method = next(n for n in planner.body if isinstance(n, ast.FunctionDef) and n.name == 'move')
    scope = dict(np=np, time=time)
    exec(compile(ast.Module(body=[method], type_ignores=[]), '<planner-test>', 'exec'), scope)
    start = np.zeros((2, 7)); end = start.copy(); end[:, 1] = 1.
    middle = .9*end; middle[0, 0] = np.deg2rad(.00005)
    for valid in (True, False):
        calls, added = [], []
        def edge(a, b, held):
            calls.append((a.copy(), b.copy()))
            return valid
        dummy = SimpleNamespace(route=lambda *args: [start, middle, end],
            scene=SimpleNamespace(edge=edge), spec={'execution': {'arms_only': True}}, held=[], log=[],
            add=lambda name, q: added.append((name, q.copy())))
        scope['move'](dummy, 'alpha', end, ('left', 'right'))
        assert len(calls) == 1
        assert np.array_equal(calls[0][0], start)
        assert np.array_equal(calls[0][1], end)
        assert len(added) == (1 if valid else 2)
        assert np.array_equal(added[-1][1], end)
    calls, added = [], []
    dummy.spec = {'execution': {'arms_only': False}}
    scope['move'](dummy, 'alpha', end, ('left', 'right'))
    assert calls == []  # Normal grasp mode keeps its original compression policy.
    assert len(added) == 2


def test_reference_pacing_reduces_restart_chatter_with_lagging_mock_feedback():
    # Numerical unit fixture, NOT a PiPER/ROS/physics simulation.
    def exercise(adaptive):
        dt = .02
        rates = np.full(7, np.deg2rad(22.5))
        acc = np.full(7, np.deg2rad(20.))
        goal = np.zeros((2, 7)); goal[0, 1] = np.deg2rad(90.)
        measured = np.zeros((2, 7)); command = measured.copy()
        segment = motion.SmoothSegmentMotion(command, goal, rates, acc*(.5 if adaptive else 1.))
        pacer = motion.TrackingPacer(rates, acc)
        waiting = False; stops = 0
        for _ in range(5000):
            ratio = float(np.max(abs(command-measured))/np.deg2rad(1.5))
            previous = waiting
            if not adaptive:
                waiting = ratio > 1.
            elif ratio > 1.:
                waiting = True
            elif ratio <= .65:
                waiting = False
            if waiting:
                stops += int(not previous)
                segment.pause(); pacer.pause()
            else:
                scale = pacer.advance(ratio, dt) if adaptive else 1.
                command = segment.advance(dt*scale)
            measured += (command-measured)*(1.-np.exp(-dt/.1))
            if np.max(abs(goal-measured)) < 1e-6 and segment.progress == 1.:
                return stops
        raise AssertionError('Mock reference failed to finish')
    assert exercise(False) > 0
    assert exercise(True) == 0


def test_compaction_keeps_corners_reversals_and_task_stops():
    def q(x, y=0):
        result = np.zeros((2, 7)); result[0, :2] = x, y
        return result
    steps = [('a', q(1), 0), ('b', q(2), 0), ('corner', q(2, 1), 0),
             ('back', q(2), 0), ('alpha', q(3), 1),
             ('bowl_open', q(3), 0), ('bowl_attached', q(3), .5),
             ('hold', q(4), 1), ('return_home', q(0), 0)]
    compact = motion.arm_only_steps(steps, q(0))
    assert [x[0] for x in compact] == ['b', 'corner', 'back', 'alpha', 'hold', 'return_home']
    assert compact[3][2] == 1
    assert compact[4][2] == 1
    # Every output edge is a union of collinear input edges or an original edge.
    assert np.array_equal(compact[0][1], q(2))
    assert np.array_equal(compact[1][1], q(2, 1))
