"""Retiming and controller-curve checks, independent of ROS transport.

Joint extrema are checked on quintic polynomials. Collision, workspace, tool
orientation and TCP speed are sampled: this is not continuous collision proof.
The polynomial is derived by solving its six boundary conditions; no controller
source is copied. Only position/velocity/acceleration trajectories are supported.
"""
from copy import deepcopy
import numpy as np
from numpy.polynomial import Polynomial

from experimental.legibility_optimizer.ros1_adapter import extract_path
from legibility_ros.contracts import positive, retime_checked


def derivatives(path):
    names, q, t = extract_path(path)
    try:
        v = np.asarray([p.velocities for p in path.joint_trajectory.points], dtype=float)
        a = np.asarray([p.accelerations for p in path.joint_trajectory.points], dtype=float)
    except (TypeError, ValueError):
        raise ValueError('retimed path requires finite velocities and accelerations at every knot')
    if v.shape != q.shape or a.shape != q.shape or not np.isfinite([v, a]).all():
        raise ValueError('retimed path requires finite velocities and accelerations at every knot')
    if abs(t[0]) > 1e-9:
        raise ValueError('trajectory must start at zero')
    if np.max(np.abs(v[[0, -1]])) > 1e-7:
        raise ValueError('trajectory must start and finish at rest')
    return names, q, t, v, a


def fit_duration(group, seed, path, required_duration, duration_factory):
    """IPTP first, then uniform time scaling; all resulting limits are rechecked."""
    required_duration = positive(required_duration, 'required duration')
    result = retime_checked(group, seed, deepcopy(path), 1.0)
    _, _, t, _, _ = derivatives(result)
    factor = required_duration / t[-1]
    for point in result.joint_trajectory.points:
        point.time_from_start = duration_factory(point.time_from_start.to_sec() * factor)
        point.velocities = (np.asarray(point.velocities) / factor).tolist()
        point.accelerations = (np.asarray(point.accelerations) / factor**2).tolist()
        point.effort = []
    return result


def reversed_path(path, duration, duration_factory):
    """Reverse the last completed reach only. Caller must validate it again."""
    _, _, t, _, _ = derivatives(path)
    factor = positive(duration, 'home duration') / t[-1]
    result = deepcopy(path)
    result.joint_trajectory.points = list(reversed(result.joint_trajectory.points))
    for point, old_time in zip(result.joint_trajectory.points, t[::-1]):
        point.time_from_start = duration_factory((t[-1] - old_time) * factor)
        point.velocities = (-np.asarray(point.velocities) / factor).tolist()
        point.accelerations = (np.asarray(point.accelerations) / factor**2).tolist()
    return result


def segment(q0, q1, v0, v1, a0, a1, dt):
    # Solve in unit time for numerical conditioning. Physical derivatives are
    # obtained by dividing by dt**order when evaluating.
    leading = np.array([q0, v0 * dt, a0 * dt**2 / 2.0])
    rhs = np.array([q1 - leading.sum(), v1 * dt - leading[1] - 2 * leading[2],
                    a1 * dt**2 - 2 * leading[2]])
    tail = np.linalg.solve(np.array([[1., 1., 1.], [3., 4., 5.], [6., 12., 20.]]), rhs)
    return Polynomial(np.r_[leading, tail])


def extrema(poly):
    points = [0., 1.]
    for root in poly.deriv().roots():
        if abs(root.imag) < 1e-8 and 0 < root.real < 1:
            points.append(float(root.real))
    values = poly(points)
    if not np.isfinite(values).all():
        raise ValueError('nonfinite spline extrema')
    return float(min(values)), float(max(values))


def check_curve(path, fk, state_valid, jacobian, bounds, vmax, amax, safety,
                orientation_tolerance=0.05, max_dt=0.02, max_step=0.01, max_samples=20000):
    """Validate the actual P/V/A quintic interpolation, including overshoot.

    state_valid must check the whole robot including attached geometry. jacobian
    must return the tool's translational Jacobian in an orthonormal frame, with
    columns ordered like the trajectory. Missing callbacks fail, never pass.
    """
    names, q, t, v, a = derivatives(path)
    bounds, vmax, amax = map(np.asarray, (bounds, vmax, amax))
    if (bounds.shape != (len(names), 2) or vmax.shape != (len(names),)
            or amax.shape != vmax.shape or not np.isfinite(bounds).all()
            or not np.isfinite([vmax, amax]).all() or np.any(vmax <= 0) or np.any(amax <= 0)
            or np.any(bounds[:, 0] >= bounds[:, 1])):
        raise ValueError('invalid loaded joint limits')
    max_dt, max_step = positive(max_dt, 'sample interval'), positive(max_step, 'joint sample step')
    orientation_tolerance = positive(orientation_tolerance, 'orientation tolerance')
    lo, hi = np.asarray(safety['workspace_min']), np.asarray(safety['workspace_max'])
    if lo.shape != (3,) or hi.shape != (3,) or not np.isfinite([lo, hi]).all() or np.any(lo >= hi):
        raise ValueError('invalid workspace')
    tcp_limit = positive(safety['max_tcp_speed'], 'TCP speed')
    # This limit describes adjacent checked samples, not sparse RRT knots.
    max_step = min(max_step, np.deg2rad(positive(safety['max_joint_step_deg'], 'joint step')))
    reference = np.asarray(fk.pose(q[0])[1])
    count, peak_tcp = 0, 0.
    for index, dt in enumerate(np.diff(t)):
        curves = [segment(q[index, j], q[index+1, j], v[index, j], v[index+1, j],
                          a[index, j], a[index+1, j], dt) for j in range(len(names))]
        peak_v = 0.
        for j, curve in enumerate(curves):
            low, high = extrema(curve)
            speed = max(abs(x) for x in extrema(curve.deriv() / dt))
            acceleration = max(abs(x) for x in extrema(curve.deriv(2) / dt**2))
            if low < bounds[j, 0] - 1e-9 or high > bounds[j, 1] + 1e-9:
                raise ValueError('quintic interpolation violates joint position limit')
            if speed > vmax[j] + 1e-9 or acceleration > amax[j] + 1e-9:
                raise ValueError('quintic interpolation violates velocity/acceleration limit')
            peak_v = max(peak_v, speed)
        intervals = max(1, int(np.ceil(dt / max_dt)), int(np.ceil(peak_v * dt / max_step)))
        count += intervals + 1
        if count > max_samples:
            raise ValueError('validation sample budget exceeded')
        for u in np.linspace(0, 1, intervals + 1):
            position = np.array([p(u) for p in curves])
            velocity = np.array([p.deriv()(u) / dt for p in curves])
            xyz, quat = map(np.asarray, fk.pose(position))
            if (xyz.shape != (3,) or quat.shape != (4,) or not np.isfinite(xyz).all()
                    or not np.isfinite(quat).all() or abs(np.linalg.norm(quat) - 1) > 1e-6):
                raise ValueError('invalid FK response')
            if np.any(xyz < lo) or np.any(xyz > hi):
                raise ValueError('sampled tool position leaves workspace')
            angle = 2 * np.arccos(np.clip(abs(np.dot(reference, quat)), 0, 1))
            if angle > orientation_tolerance:
                raise ValueError('sampled tool orientation exceeds tolerance')
            jac = np.asarray(jacobian(position), dtype=float)
            if jac.shape != (3, len(names)) or not np.isfinite(jac).all():
                raise ValueError('invalid translational Jacobian')
            speed = np.linalg.norm(jac @ velocity)
            if speed > tcp_limit:
                raise ValueError('sampled TCP speed exceeds limit')
            peak_tcp = max(peak_tcp, float(speed))
            if state_valid(position) is not True:
                raise ValueError('whole-robot state validity check failed')
    return dict(checked_states=count, peak_sampled_tcp_speed=peak_tcp,
                interpolation='quintic_pva', collision_check='discrete',
                max_sample_dt=max_dt, max_sample_joint_step=max_step,
                continuous_collision_certified=False)
