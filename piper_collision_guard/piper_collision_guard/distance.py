"""Pure calculation layer: radians in, world XYZ and point gaps in metres out."""
from dataclasses import dataclass
import math

import numpy as np
import pinocchio as pin


def arm_state(names, positions, side):
    """Read six named arm joints and Piper's full gripper opening in metres.

    Do not assume JointState array order. Both arms use the same unprefixed
    names in the upstream driver, but arrive on separate topics.
    """
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError("joint names/positions mismatch or duplicate name")
    values = dict(zip(names, positions))
    if not all(math.isfinite(float(v)) for v in positions):
        raise ValueError("non-finite joint state")
    q = []
    for index in range(1, 7):
        matches = [k for k in (f"joint{index}", f"{side}_joint{index}") if k in values]
        if len(matches) != 1:
            raise ValueError(f"missing/ambiguous {side} joint{index}")
        q.append(float(values[matches[0]]))
    matches = [k for k in ("gripper", "joint7", f"{side}_joint7") if k in values]
    if len(matches) != 1:
        raise ValueError("one full-opening gripper value is required (Piper driver convention)")
    opening = float(values[matches[0]])
    if not 0 <= opening <= 0.08:
        raise ValueError("gripper opening must be in [0, 0.08] metres")
    return np.asarray(q), opening


@dataclass
class Clearance:
    distance: float
    left_name: str
    right_name: str
    left_xyz: np.ndarray
    right_xyz: np.ndarray
    center_distance: float = 0.0


def closest(left, right, left_names, right_names, left_radii=None, right_radii=None):
    distances = np.linalg.norm(left[:, None, :] - right[None, :, :], axis=-1)
    if not np.isfinite(distances).all():
        raise ValueError("invalid point distances")
    ra = np.zeros(len(left)) if left_radii is None else np.asarray(left_radii)
    rb = np.zeros(len(right)) if right_radii is None else np.asarray(right_radii)
    if ra.shape != (len(left),) or rb.shape != (len(right),) or not np.isfinite(ra).all() or not np.isfinite(rb).all() or np.any(ra < 0) or np.any(rb < 0):
        raise ValueError("invalid protection radii")
    gaps = distances - ra[:, None] - rb[None, :]
    i, j = np.unravel_index(np.argmin(gaps), gaps.shape)
    return Clearance(float(gaps[i, j]), left_names[i], right_names[j], left[i].copy(), right[j].copy(), float(distances[i, j]))


class DistanceModel:
    """FK of both arms in one URDF. No mesh or MoveIt collision checking.

    Point model: link1..link6 origins, a gripper centre 0.1358 m along
    gripper_base +Z, and optional samples between successive arm origins.
    Optional local offsets and protective radii approximate volume. Their
    coverage must be calibrated; finger opening and payloads are not modelled.
    """
    def __init__(self, urdf, link_samples=2, protection=None):
        if not isinstance(link_samples, int) or not 0 <= link_samples <= 20:
            raise ValueError("link_samples must be an integer in [0,20]")
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        self.slots, self.frames = {}, {}
        for side in ("left", "right"):
            slots = []
            for i in range(1, 7):
                name = f"{side}_joint{i}"
                if not self.model.existJointName(name):
                    raise ValueError(f"missing {name}")
                joint = self.model.joints[self.model.getJointId(name)]
                if joint.nq != 1:
                    raise ValueError(f"{name} must have one position coordinate")
                slots.append(joint.idx_q)
            self.slots[side] = slots
            names = [f"{side}_link{i}" for i in range(1, 7)] + [f"{side}_gripper_base"]
            if not all(self.model.existFrame(n) for n in names):
                raise ValueError("missing arm/gripper frames")
            self.frames[side] = [self.model.getFrameId(n) for n in names]
        self.link_samples = link_samples
        default_offsets = np.zeros((7, 3))
        default_offsets[-1, 2] = 0.1358
        self.offsets = {}
        self.radii = {}
        for side in ("left", "right"):
            config = (protection or {}).get(side, {})
            offsets = np.asarray(config.get("offsets_m", default_offsets), dtype=float)
            radii = np.asarray(config.get("radii_m", np.zeros(7)), dtype=float)
            if offsets.shape != (7, 3) or radii.shape != (7,) or not np.isfinite(offsets).all() or not np.isfinite(radii).all() or np.any(radii < 0):
                raise ValueError("protection needs seven finite XYZ offsets and seven non-negative radii per arm")
            self.offsets[side], self.radii[side] = offsets, radii

    def configuration(self, left, right):
        q = pin.neutral(self.model)
        for side, values in (("left", left), ("right", right)):
            values = np.asarray(values, dtype=float)
            if values.shape != (6,) or not np.isfinite(values).all():
                raise ValueError("each arm needs six finite joint angles in radians")
            indices = self.slots[side]
            if np.any(values < self.model.lowerPositionLimit[indices]) or np.any(values > self.model.upperPositionLimit[indices]):
                raise ValueError(f"{side} joint angle outside URDF bounds")
            q[indices] = values
        return q

    def points(self, left, right):
        pin.framesForwardKinematics(self.model, self.data, self.configuration(left, right))
        result = {}
        for side in ("left", "right"):
            placements = [self.data.oMf[f] for f in self.frames[side]]
            points = [p.act(offset) for p, offset in zip(placements, self.offsets[side])]
            names = [f"{side}_joint{i}_protection" for i in range(1, 7)]
            names.append(f"{side}_gripper_protection")
            radii = self.radii[side].tolist()
            original = list(points)
            for i in range(len(original) - 1):
                for k in range(1, self.link_samples + 1):
                    t = k / (self.link_samples + 1)
                    points.append((1-t) * original[i] + t * original[i+1])
                    names.append(f"{side}_segment{i+1}_sample{k}")
                    radii.append(max(self.radii[side][i], self.radii[side][i+1]))
            result[side] = (np.asarray(points), names, np.asarray(radii))
        return result

    def clearance(self, left, right):
        p = self.points(left, right)
        return closest(p["left"][0], p["right"][0], p["left"][1], p["right"][1], p["left"][2], p["right"][2])

    def path_clearance(self, left, right, left_target, right_target, step_rad=0.02, max_samples=100):
        if not math.isfinite(step_rad) or step_rad <= 0 or max_samples < 1:
            raise ValueError("invalid path sampling settings")
        start = self.configuration(left, right)
        end = self.configuration(left_target, right_target)
        steps = max(1, math.ceil(float(np.max(np.abs(end-start))) / step_rad))
        if steps > max_samples:
            raise ValueError("target jump exceeds path sampling budget")
        best = None
        for t in np.linspace(0., 1., steps + 1):
            c = self.clearance((1-t)*left+t*left_target, (1-t)*right+t*right_target)
            if best is None or c.distance < best.distance:
                best = c
        return best


class Interlock:
    """Latched stop, explicit reset, and per-arm freshness/skew checks."""
    def __init__(self, stop, resume, timeout, max_skew):
        if not all(math.isfinite(x) for x in (stop, resume, timeout, max_skew)) or not 0 < stop < resume or timeout <= 0 or max_skew < 0:
            raise ValueError("require 0 < stop < resume, timeout > 0, max_skew >= 0")
        self.stop, self.resume, self.timeout, self.max_skew = stop, resume, timeout, max_skew
        self.latched = False
        self.reason = "waiting_for_feedback"

    def trip(self, reason):
        self.latched, self.reason = True, reason

    def fresh(self, timestamps, now):
        if set(timestamps) != {"left", "right"}:
            return False
        values = list(timestamps.values())
        return all(0 <= now-t <= self.timeout for t in values) and abs(values[0]-values[1]) <= self.max_skew

    def observe(self, distance):
        if not math.isfinite(distance) or distance <= self.stop:
            self.trip("distance_stop")

    def reset(self, distance, fresh):
        if not fresh or not math.isfinite(distance) or distance <= self.resume:
            return False
        self.latched, self.reason = False, "ready"
        return True
