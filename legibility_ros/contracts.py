"""Testable ROS message protocols; these helpers do not import ROS."""
from copy import deepcopy
from functools import lru_cache
import xml.etree.ElementTree as ET

import numpy as np

from experimental.legibility_optimizer.ros1_adapter import extract_path
from experimental.legibility_optimizer.optimizer_core import segment_velocities, interior_accelerations


def positive(value, label):
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise ValueError(label + " must be finite and positive")
    return value


def vector(value, size, label):
    value = np.asarray(value, dtype=float)
    if value.shape != (size,) or not np.all(np.isfinite(value)):
        raise ValueError(label + " has invalid shape or nonfinite entries")
    return value


def read_experiment(cfg):
    """Use the group's measured layout, with tool0 hover goals, not cube centers."""
    motion = cfg["motion"]
    for key, value in (("frame_id", cfg["frame_id"]),
                       ("planning_group", motion["planning_group"]),
                       ("ee_link", motion["ee_link"])):
        if not isinstance(value, str) or not value.strip():
            raise ValueError(key + " must be explicit")
    blocks = cfg["blocks"]
    ids = [b["id"] for b in blocks]
    if len(ids) < 2 or len(set(ids)) != len(ids) or any(type(i) is not int for i in ids):
        raise ValueError("blocks require at least two unique integer ids")
    z = float(motion["touch_z"]) + positive(motion["hover_height"], "hover_height")
    goals = np.array([[*vector(b["position"], 3, "block position")[:2], z] for b in blocks])
    if not np.all(np.isfinite(goals)) or len(np.unique(goals, axis=0)) != len(goals):
        raise ValueError("hover goals must be finite and distinct")
    positive(motion["duration"], "duration")
    safety = cfg["safety"]
    lo, hi = (vector(safety[k], 3, k) for k in ("workspace_min", "workspace_max"))
    if np.any(lo >= hi) or np.any(goals < lo) or np.any(goals > hi):
        raise ValueError("goals or workspace bounds are invalid")
    for key in ("max_tcp_speed", "max_joint_speed_deg", "max_joint_step_deg"):
        positive(safety[key], key)
    return ids, goals


def state_with_positions(seed, names, positions):
    """Keep non-arm joints, attachments and multi-DOF state from a full snapshot."""
    state = deepcopy(seed)
    js = state.joint_state
    if seed.is_diff or len(js.name) != len(set(js.name)) or len(js.name) != len(js.position):
        raise ValueError("a complete, unique RobotState snapshot is required")
    vector(js.position, len(js.name), "seed positions")
    if len(set(names)) != len(names) or not set(names).issubset(js.name):
        raise ValueError("seed state is missing trajectory joints or names are duplicated")
    q = vector(positions, len(names), "joint positions")
    values = dict(zip(names, q))
    js.position = [float(values.get(n, p)) for n, p in zip(js.name, js.position)]
    js.velocity, js.effort = [], []
    state.is_diff = False
    return state


class FK:
    """Exact per-request cache, no quantization, no hardcoded robot kinematics."""
    def __init__(self, service, request_factory, seed, names, frame, link, max_calls=30000):
        if not frame or not link or max_calls < 1:
            raise ValueError("FK needs explicit frame/link and a positive call budget")
        self.service, self.request_factory = service, request_factory
        self.seed, self.names = deepcopy(seed), tuple(names)
        self.frame, self.link, self.max_calls = frame, link, max_calls
        self.calls = 0
        self._cached = lru_cache(maxsize=8192)(self._query)

    def _query(self, key):
        if self.calls >= self.max_calls:
            raise RuntimeError("FK service call budget exceeded; no candidate accepted")
        req = self.request_factory()
        req.header.frame_id, req.fk_link_names = self.frame, [self.link]
        req.robot_state = state_with_positions(self.seed, self.names, key)
        self.calls += 1
        response = self.service(req)
        if response.error_code.val != 1:
            raise RuntimeError("MoveIt FK failed with code " + str(response.error_code.val))
        if list(response.fk_link_names) != [self.link] or len(response.pose_stamped) != 1:
            raise ValueError("FK response does not contain exactly the requested link")
        stamped = response.pose_stamped[0]
        if stamped.header.frame_id != self.frame:
            raise ValueError("FK response frame differs from goal frame")
        p, r = stamped.pose.position, stamped.pose.orientation
        xyz = vector([p.x, p.y, p.z], 3, "FK position")
        quat = vector([r.x, r.y, r.z, r.w], 4, "FK quaternion")
        if not np.isclose(np.linalg.norm(quat), 1, atol=1e-5, rtol=0):
            raise ValueError("FK quaternion is not unit length")
        return tuple(xyz), tuple(quat)

    def pose(self, q):
        xyz, quat = self._cached(tuple(vector(q, len(self.names), "FK joints")))
        return np.array(xyz), np.array(quat)

    def __call__(self, q):
        return self.pose(q)[0]


def loaded_limits(robot, names, urdf, overrides, speed_cap_deg):
    """Position bounds from MoveIt, velocity from URDF, tighter overrides honored.

    Acceleration must be configured explicitly. No guessed UR5/UR5e limits.
    """
    joints = {j.attrib["name"]: j for j in ET.fromstring(urdf).findall("joint")}
    bounds, velocities, accelerations = [], [], []
    for name in names:
        pair = vector(robot.get_joint(name).bounds(), 2, name + " bounds")
        if pair[0] >= pair[1]:
            raise ValueError("finite unwrapped position bounds required for " + name)
        bounds.append(pair)
        joint = joints[name]
        if joint.attrib.get("type") != "revolute":
            raise ValueError("this lab adapter requires bounded revolute joints: " + name)
        vmax = positive(joint.find("limit").attrib["velocity"], name + " URDF velocity")
        override = overrides.get(name, {})
        if override.get("has_velocity_limits", False):
            vmax = min(vmax, positive(override["max_velocity"], name + " velocity"))
        velocities.append(min(vmax, np.deg2rad(positive(speed_cap_deg, "speed cap"))))
        if override.get("has_acceleration_limits") is not True:
            raise ValueError("missing acceleration limit for " + name)
        accelerations.append(positive(override["max_acceleration"], name + " acceleration"))
    return np.asarray(bounds), np.asarray(velocities), np.asarray(accelerations)


def replace_path(template, q, t, duration_factory):
    names, old, _ = extract_path(template)
    if np.shape(q) != old.shape or len(t) != len(old):
        raise ValueError("replacement must preserve the original trajectory grid")
    out = deepcopy(template)
    for point, values, time in zip(out.joint_trajectory.points, q, t):
        point.positions = vector(values, len(names), "replacement positions").tolist()
        point.velocities, point.accelerations, point.effort = [], [], []
        point.time_from_start = duration_factory(float(time))
    extract_path(out)
    return out


def retime_checked(group, seed, trajectory, scale):
    scale = positive(scale, "retiming scale")
    if scale > 1:
        raise ValueError("retiming scale must be <= 1")
    names, q, _ = extract_path(trajectory)
    result = group.retime_trajectory(deepcopy(seed), deepcopy(trajectory),
                                     velocity_scaling_factor=scale,
                                     acceleration_scaling_factor=scale,
                                     algorithm="iterative_time_parameterization")
    new_names, new_q, _ = extract_path(result)
    if new_names != names or new_q.shape != q.shape or not np.array_equal(new_q, q):
        raise ValueError("retimer changed joint order, grid or positions; reoptimization required")
    return result


def sampled_checks(trajectory, fk, state_valid, bounds, vmax, amax, safety,
                   orientation_tolerance, *, sample_step=0.02, max_samples=20000):
    """Discrete checks only. Never certifies controller interpolation or execution."""
    names, q, t = extract_path(trajectory)
    tol = 1e-7
    orientation_tolerance = positive(orientation_tolerance, "orientation tolerance")
    sample_step = positive(sample_step, "sample step")
    if np.any(q < bounds[:, 0] - tol) or np.any(q > bounds[:, 1] + tol):
        raise ValueError("sampled joint position limit violation")
    if np.any(np.abs(segment_velocities(q, t)) > vmax + tol):
        raise ValueError("sampled joint velocity limit violation")
    if np.any(np.abs(interior_accelerations(q, t)) > amax + tol):
        raise ValueError("sampled joint acceleration limit violation")
    for field, limit in (("velocities", vmax), ("accelerations", amax)):
        for point in trajectory.joint_trajectory.points:
            if np.any(np.abs(vector(getattr(point, field), len(names), field)) > limit + tol):
                raise ValueError("retimed " + field + " limit violation")
    lo = vector(safety["workspace_min"], 3, "workspace minimum")
    hi = vector(safety["workspace_max"], 3, "workspace maximum")
    tcp_speed = positive(safety["max_tcp_speed"], "TCP speed")
    orientation = fk.pose(q[0])[1]
    counts = np.maximum(1, np.ceil(np.max(np.abs(np.diff(q, axis=0)), axis=1) / sample_step)).astype(int)
    if int(np.sum(counts)) + 1 > max_samples:
        raise ValueError("sample validation budget exceeded")
    previous_xyz, previous_time, checked = None, None, 0
    # Include all original knots and bounded joint-space interpolation samples.
    samples = [(q[0], t[0])]
    for i, count in enumerate(counts):
        for k in range(1, int(count) + 1):
            alpha = k / count
            samples.append((q[i] + alpha * (q[i + 1] - q[i]), t[i] + alpha * (t[i + 1] - t[i])))
    for values, time in samples:
        xyz, quat = fk.pose(values)
        if np.any(xyz < lo) or np.any(xyz > hi):
            raise ValueError("sampled tool workspace violation")
        angle = 2 * np.arccos(np.clip(abs(np.dot(quat, orientation)), 0, 1))
        if angle > orientation_tolerance:
            raise ValueError("sampled tool orientation drift exceeds tolerance")
        if previous_xyz is not None and np.linalg.norm(xyz - previous_xyz) / (time - previous_time) > tcp_speed + tol:
            raise ValueError("sampled TCP speed violation")
        if not state_valid(values):
            raise ValueError("MoveIt state validity rejected sample " + str(checked))
        checked += 1
        previous_xyz, previous_time = xyz, time
    return {"sample_checks_passed": True, "checked_states": checked,
            "sampling_max_joint_step_rad": sample_step,
            "physical_validity": "unassessed", "execution_ready": False}
