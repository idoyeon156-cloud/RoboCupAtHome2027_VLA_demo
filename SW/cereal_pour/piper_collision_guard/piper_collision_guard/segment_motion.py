"""Bound target acceleration along an unchanged, collision-checked joint edge.

This shapes position targets only. It is not a controller/torque/dynamics model.
All joints share one scalar progress so smoothing never cuts a planned corner.
"""
import numpy as np


def is_alpha_pose(goal, reference):
    """Task labels can describe the source action on a reversed path.

    Recognize the destination by both arms' six joint angles; jaw width is
    irrelevant to the boundary approach speed.
    """
    return bool(np.allclose(np.asarray(goal)[:, :6], np.asarray(reference)[:, :6],
                            rtol=0., atol=1e-8))


class TrackingPacer:
    """Slow reference time BEFORE the unchanged hard tracking threshold.

    Profiles use half the acceleration budget. With reference velocity <= V,
    limiting |pace'| <= A/(2V) leaves the other half for time scaling:
    |q''| <= |q_ref''|*pace**2 + |q_ref'|*|pace'| <= A.
    Hard guard/tracking stops are deliberately outside this normal-motion rule.
    """
    def __init__(self, rates, acceleration):
        rates, acceleration = np.broadcast_arrays(rates, acceleration)
        if (not np.isfinite(rates).all() or np.any(rates <= 0) or
                not np.isfinite(acceleration).all() or np.any(acceleration <= 0)):
            raise ValueError('Positive finite pacing limits required')
        self.slew = float(np.min(acceleration/(2.*rates)))
        self.factor = 1.

    def pause(self):
        self.factor = 0.

    def advance(self, error_ratio, dt):
        if not np.isfinite(error_ratio) or error_ratio < 0 or not np.isfinite(dt) or dt < 0:
            raise ValueError('Finite nonnegative pacing input required')
        # Full speed below 35% of hard tolerance; minimum pace by 75%.
        desired = float(np.clip(1.-.85*max(0., error_ratio-.35)/.4, .15, 1.))
        previous = self.factor
        self.factor += float(np.clip(desired-self.factor, -self.slew*dt, self.slew*dt))
        return .5*(previous+self.factor)


class SmoothSegmentMotion:
    """Quintic time scaling on ONE checked straight edge; no corner cutting.

    Velocity and acceleration are zero at both ends. Duration enforces the
    analytic peak velocity (15/8) and acceleration (10/sqrt(3)) bounds.
    Guard/tracking pauses still stop target progress immediately, then restart
    on the remaining suffix; this is not a physical braking controller.
    """
    def __init__(self, start, goal, rates, acceleration):
        self.goal = np.asarray(goal, dtype=float).copy()
        self.rates = np.asarray(rates, dtype=float)
        self.acceleration_limits = np.asarray(acceleration, dtype=float)
        if (not np.isfinite(self.rates).all() or np.any(self.rates <= 0) or
                not np.isfinite(self.acceleration_limits).all() or np.any(self.acceleration_limits <= 0)):
            raise ValueError('Positive finite motion limits required')
        self.position = np.asarray(start, dtype=float).copy()
        self._restart()

    def _restart(self):
        self.start = self.position.copy()
        self.delta = self.goal-self.start
        if not np.isfinite(self.delta).all():
            raise ValueError('Nonfinite motion endpoint')
        magnitude = np.abs(self.delta)
        self.duration = max(float(np.max(1.875*magnitude/self.rates)),
                            float(np.sqrt(np.max((10./np.sqrt(3.))*magnitude/self.acceleration_limits))))
        self.elapsed = self.progress = self.speed = 0.

    def pause(self):
        self._restart()

    def advance(self, dt):
        if not np.isfinite(dt) or dt < 0:
            raise ValueError('Finite nonnegative timestep required')
        self.elapsed = min(self.duration, self.elapsed+float(dt))
        if self.duration <= 1e-12 or self.elapsed >= self.duration:
            self.progress, self.speed = 1., 0.
            self.position = self.goal.copy()
        else:
            u = self.elapsed/self.duration
            self.progress = min(1., max(0., u**3*(10.-15.*u+6.*u*u)))
            self.speed = 30.*u*u*(1.-u)**2/self.duration
            self.position = self.start+self.delta*self.progress
        return self.position.copy()


def arm_only_steps(steps, initial):
    """Remove empty grasp actions and EXACT collinear points, preserving corners.

    The resulting edges are unions of the checked original edges. No joint
    angle, held-object geometry, collision policy or speed limit is relaxed.
    """
    result = []
    protected = {'alpha', 'return_alpha', 'return_home', 'home', 'home_joints'}
    for name, goal, hold in steps:
        if name.endswith(('_attached', '_open', '_close')):
            hold = 0.  # No physical grasp/release or grasp dwell in this mode.
        previous = result[-1][1] if result else initial
        if name not in protected and hold == 0 and np.array_equal(previous, goal):
            continue
        result.append((name, goal.copy(), hold))
        while len(result) >= 2:
            middle, end = result[-2:]
            if middle[0] in protected or middle[2] != 0:
                break
            start = result[-3][1] if len(result) >= 3 else initial
            full, part = (end[1]-start).ravel(), (middle[1]-start).ravel()
            length = float(full@full)
            if length <= 1e-20:
                break
            fraction = float(part@full/length)
            if not 0. < fraction < 1. or np.max(np.abs(part-fraction*full)) > 1e-10:
                break
            result.pop(-2)
    return result


class SegmentMotion:
    def __init__(self, start, goal, rates, acceleration, *, alpha=False):
        self.start = np.asarray(start).copy()
        self.goal = np.asarray(goal).copy()
        self.delta = self.goal-self.start
        self.progress = 0.
        self.speed = 0.
        magnitude = np.abs(self.delta)
        moving = magnitude > 1e-12
        self.rate_limit = float(np.min(np.broadcast_to(rates, magnitude.shape)[moving]/magnitude[moving])) if moving.any() else 0.
        self.acceleration = float(np.min(np.broadcast_to(acceleration, magnitude.shape)[moving]/magnitude[moving])) if moving.any() else 1.
        self.tail_start = 1.
        self.tail_rate = self.rate_limit
        if alpha and np.max(magnitude[:, 4]) > 1e-12:
            j5_travel = float(np.max(magnitude[:, 4]))
            self.tail_start = max(0., 1.-np.deg2rad(5.)/j5_travel)
            self.tail_rate = min(self.rate_limit, np.deg2rad(1.)/j5_travel)

    def pause(self):
        self.speed = 0.

    def advance(self, dt):
        if self.rate_limit == 0. or self.progress >= 1.:
            self.progress = 1.
            self.speed = 0.
            return self.goal.copy()
        dt = max(0., float(dt))
        remaining = 1.-self.progress
        # Brake before the endpoint and before the slower alpha tail begins.
        cap = min(self.rate_limit, np.sqrt(self.acceleration*remaining))
        if self.tail_rate < self.rate_limit:
            cap = min(cap, np.sqrt(self.tail_rate**2+self.acceleration*max(0., self.tail_start-self.progress)))
        self.speed = min(cap, self.speed+self.acceleration*dt)
        self.progress = min(1., self.progress+self.speed*dt)
        # Finite position resolution prevents an asymptotic endpoint wait.
        if self.progress >= 1.-1e-10:
            self.progress = 1.
            self.speed = 0.
            return self.goal.copy()
        return self.start+self.delta*self.progress
