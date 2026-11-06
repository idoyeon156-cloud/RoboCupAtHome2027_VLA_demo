"""Pure calculation layer: radians in, world XYZ and point gaps in metres out."""
from dataclasses import dataclass
import math

import numpy as np
import pinocchio as pin
from scipy.spatial import cKDTree


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
    left, right = np.asarray(left), np.asarray(right)
    if left.ndim != 2 or right.ndim != 2 or left.shape[1] != 3 or right.shape[1] != 3 or not len(left) or not len(right) or not np.isfinite(left).all() or not np.isfinite(right).all():
        raise ValueError("invalid point distances")
    ra = np.zeros(len(left)) if left_radii is None else np.asarray(left_radii)
    rb = np.zeros(len(right)) if right_radii is None else np.asarray(right_radii)
    if ra.shape != (len(left),) or rb.shape != (len(right),) or not np.isfinite(ra).all() or not np.isfinite(rb).all() or np.any(ra < 0) or np.any(rb < 0):
        raise ValueError("invalid protection radii")
    # Every mesh cell group has one radius. Query each right-radius group
    # exactly, then subtract both radii; ordinary nearest-centre search alone
    # would miss a larger but more distant sphere.
    best = None
    for radius in np.unique(rb):
        indices = np.flatnonzero(rb == radius)
        distances, nearest = cKDTree(right[indices]).query(left)
        gaps = distances - ra - radius
        i = int(np.argmin(gaps))
        j = int(indices[nearest[i]])
        if best is None or gaps[i] < best.distance:
            best = Clearance(float(gaps[i]), left_names[i], right_names[j], left[i].copy(), right[j].copy(), float(distances[i]))
    return best


class DistanceModel:
    """FK of both arms in one URDF. No mesh or MoveIt collision checking.

    Configured mesh covers enclose full source collision AABBs using cell
    circumspheres. Finger covers include all opening positions. Legacy seven
    point settings remain supported for explicitly supplied custom models.
    No payload volume or measured braking model is included.
    """
    def __init__(self, urdf, link_samples=2, protection=None):
        if not isinstance(link_samples, int) or not 0 <= link_samples <= 20:
            raise ValueError("link_samples must be an integer in [0,20]")
        description = str(urdf)
        self.model = pin.buildModelFromXML(description) if description.lstrip().startswith('<') else pin.buildModelFromUrdf(description)
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
        self.covers = {}
        self.offsets = {}
        self.radii = {}
        for side in ("left", "right"):
            config = (protection or {}).get(side, {})
            if 'mesh_covers' in config:
                self.covers[side] = self.make_covers(side, config['mesh_covers'])
                self.radii[side] = np.concatenate([item[2] for item in self.covers[side]])
                continue
            offsets = np.asarray(config.get("offsets_m", default_offsets), dtype=float)
            radii = np.asarray(config.get("radii_m", np.zeros(7)), dtype=float)
            if offsets.shape != (7, 3) or radii.shape != (7,) or not np.isfinite(offsets).all() or not np.isfinite(radii).all() or np.any(radii < 0):
                raise ValueError("protection needs seven finite XYZ offsets and seven non-negative radii per arm")
            self.offsets[side], self.radii[side] = offsets, radii

    def make_covers(self, side, specs):
        if not isinstance(specs, list) or not specs:
            raise ValueError("mesh_covers must be a nonempty list")
        result = []
        for spec in specs:
            frame = f"{side}_{spec['frame']}"
            lo = np.asarray(spec['bounds_min_m'], dtype=float)
            hi = np.asarray(spec['bounds_max_m'], dtype=float)
            cell, padding = float(spec['cell_size_m']), float(spec['padding_m'])
            if not self.model.existFrame(frame) or lo.shape != (3,) or hi.shape != (3,) or not np.isfinite(np.r_[lo, hi, cell, padding]).all() or np.any(hi <= lo) or cell <= 0 or padding < 0:
                raise ValueError("invalid mesh cover bounds, frame, cell size or padding")
            counts = np.ceil((hi - lo) / cell).astype(int)
            if np.prod(counts.astype(float)) > 20000:
                raise ValueError("mesh cover exceeds sphere budget")
            step = (hi - lo) / counts
            axes = [lo[i] + (np.arange(counts[i]) + .5) * step[i] for i in range(3)]
            centres = np.stack(np.meshgrid(*axes, indexing='ij'), axis=-1).reshape(-1, 3)
            radii = np.full(len(centres), np.linalg.norm(step) / 2 + padding)
            result.append((self.model.getFrameId(frame), centres, radii,
                           [f"{frame}_cover_{i}" for i in range(len(centres))]))
        return result

    def configuration(self, left, right):
        q = pin.neutral(self.model)
        for side, values in (("left", left), ("right", right)):
            values = np.asarray(values, dtype=float)
            if values.shape != (6,) or not np.isfinite(values).all():
                raise ValueError("each arm needs six finite joint angles in radians")
            indices = self.slots[side]
            lower = self.model.lowerPositionLimit[indices]
            upper = self.model.upperPositionLimit[indices]
            # Linear interpolation at a limit can differ by one floating-point
            # ULP. Accept only numerical roundoff, never a real out-of-range pose.
            if np.any(values < lower - 1e-10) or np.any(values > upper + 1e-10):
                raise ValueError(f"{side} joint angle outside URDF bounds")
            q[indices] = np.clip(values, lower, upper)
        return q

    def points(self, left, right):
        pin.framesForwardKinematics(self.model, self.data, self.configuration(left, right))
        result = {}
        for side in ("left", "right"):
            if side in self.covers:
                points, names, radii = [], [], []
                for frame, local, radius, labels in self.covers[side]:
                    pose = self.data.oMf[frame]
                    points.append(local @ pose.rotation.T + pose.translation)
                    names.extend(labels)
                    radii.append(radius)
                result[side] = (np.concatenate(points), names, np.concatenate(radii))
                continue
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
