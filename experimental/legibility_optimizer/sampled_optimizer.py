"""Optimize a small support grid, then check the candidate on the ORIGINAL grid.

Useful with service-based FK and dense (e.g. 100 Hz) baseline trajectories.
This is a surrogate optimization, not an equivalent full-dimensional solve.
"""
from dataclasses import replace
from numbers import Integral

import numpy as np

from .optimizer_core import (OptimizerConfig, optimize_joint_path, legibility_score,
                             joint_motion_cost, validate_trajectory_arrays,
                             segment_velocities, interior_accelerations)


def optimize_sampled_path(positions, times, goals, intended, fk, *, support_points=11,
                          config=None, **constraints):
    q, t = validate_trajectory_arrays(positions, times, minimum_points=3)
    if (isinstance(support_points, bool) or not isinstance(support_points, Integral)
            or support_points < 3):
        raise ValueError("support_points must be an integer >= 3")
    config = OptimizerConfig() if config is None else config
    # Check the dense baseline too: support selection must not hide violations.
    lower = np.asarray(constraints["joint_lower"], dtype=float)
    upper = np.asarray(constraints["joint_upper"], dtype=float)
    if np.any(q < lower) or np.any(q > upper):
        raise ValueError("dense baseline violates supplied joint limits")
    for key, derivative in (("velocity_limits", segment_velocities),
                            ("acceleration_limits", interior_accelerations)):
        limit = constraints.get(key)
        if limit is not None and np.any(np.abs(derivative(q, t)) > limit):
            raise ValueError("dense baseline violates " + key)
    indices = np.unique(np.linspace(0, len(q) - 1, min(support_points, len(q))).astype(int))
    result = optimize_joint_path(q[indices], t[indices], goals, intended, fk,
                                 config=config, **constraints)
    offsets = result.positions - q[indices]
    candidate = q + np.column_stack([
        np.interp(t, t[indices], offsets[:, j]) for j in range(q.shape[1])])
    candidate[[0, -1]] = q[[0, -1]]  # Exact, including floating point representation.
    options = dict(beta=config.beta, early_weight_power=config.early_weight_power,
                   priors=constraints.get("priors"))
    before = legibility_score(q, t, goals, intended, fk, **options)
    after = legibility_score(candidate, t, goals, intended, fk, **options)
    cost0, cost1 = joint_motion_cost(q, t), joint_motion_cost(candidate, t)
    tol = config.constraint_tolerance
    joint_ok = bool(np.all(candidate >= lower - tol) and np.all(candidate <= upper + tol))
    local_ok = bool(np.all(np.abs(candidate - q) <=
                           np.asarray(constraints["max_joint_deviation"]) + tol))
    cost_ok = cost1 <= config.cost_ratio * cost0 + tol * max(1, config.cost_ratio * cost0)
    dynamic = []
    for key, derivative in (("velocity_limits", segment_velocities),
                            ("acceleration_limits", interior_accelerations)):
        limit = constraints.get(key)
        dynamic.append(None if limit is None else bool(
            np.all(np.abs(derivative(candidate, t)) <= np.asarray(limit) + tol)))
    improvement = after >= before + config.min_legibility_improvement
    checks = (joint_ok, local_ok, cost_ok, improvement,
              dynamic[0] is not False, dynamic[1] is not False)
    accepted = result.numerical_accepted and all(checks)
    return replace(result, positions=candidate, times=t, numerical_accepted=accepted,
                   baseline_legibility=before, optimized_legibility=after,
                   baseline_joint_cost=cost0, optimized_joint_cost=cost1,
                   realized_cost_ratio=cost1 / cost0, joint_limit_ok=joint_ok,
                   local_deviation_ok=local_ok, cost_budget_ok=cost_ok,
                   velocity_ok=dynamic[0], acceleration_ok=dynamic[1],
                   improvement_ok=improvement,
                   message=("dense candidate accepted numerically" if accepted else
                            "dense candidate rejected; support solve: " + result.message)
                   + "; physical validity unassessed; retiming and robot validation required")
