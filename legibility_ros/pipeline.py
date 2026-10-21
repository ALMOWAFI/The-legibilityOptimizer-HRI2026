"""Preview orchestration, with injected ROS operations for offline testing."""
from dataclasses import asdict

import numpy as np

from experimental.legibility_optimizer.optimizer_core import legibility_score, joint_motion_cost
from experimental.legibility_optimizer.sampled_optimizer import optimize_sampled_path
from experimental.legibility_optimizer.ros1_adapter import extract_path
from .contracts import replace_path, retime_checked


def optimize_preview(baseline, seed, goals, intended, fk, group, validate,
                     duration_factory, *, config, support_points, constraints, scale):
    """Rejected paths never become the returned preview; exceptions fail closed.

    Numerical status is retained when a later retiming/validation stage fails.
    All returned diagnostics deny execution readiness, including successful previews.
    """
    diagnostics = {"solver_success": False, "numerical_accepted": False,
                   "sample_checks_passed": False, "physical_validity": "unassessed",
                   "execution_ready": False, "stage": "baseline_checks"}

    def checked(path):
        status = validate(path)
        if not isinstance(status, dict) or status.get("sample_checks_passed") is not True:
            raise ValueError("validator did not explicitly pass the sampled checks")
        if status.get("execution_ready", False) is not False:
            raise ValueError("sampled validation cannot authorize execution")
        return status

    try:
        _, q, t = extract_path(baseline)
        diagnostics["baseline_checks"] = checked(baseline)
        diagnostics["stage"] = "optimization"
        result = optimize_sampled_path(q, t, goals, intended, fk,
                                      support_points=support_points, config=config,
                                      **constraints)
        metrics = asdict(result)
        del metrics["positions"], metrics["times"]
        diagnostics.update(metrics)
        if not result.numerical_accepted:
            return None, diagnostics
        diagnostics["stage"] = "retiming"
        candidate = replace_path(baseline, result.positions, result.times, duration_factory)
        candidate = retime_checked(group, seed, candidate, scale)
        diagnostics["stage"] = "candidate_checks"
        checks = checked(candidate)
        _, new_q, new_t = extract_path(candidate)
        after = legibility_score(new_q, new_t, goals, intended, fk,
                                 beta=config.beta, early_weight_power=config.early_weight_power,
                                 priors=constraints.get("priors"))
        retimed_cost = joint_motion_cost(new_q, new_t)
        diagnostics.update(retimed_legibility=after, baseline_duration=float(t[-1] - t[0]),
                           candidate_duration=float(new_t[-1] - new_t[0]),
                           retimed_joint_cost=retimed_cost,
                           retimed_cost_ratio=retimed_cost / result.baseline_joint_cost)
        if after < result.baseline_legibility + config.min_legibility_improvement:
            raise ValueError("legibility improvement lost after retiming")
        maximum_cost = config.cost_ratio * result.baseline_joint_cost
        if retimed_cost > maximum_cost + config.constraint_tolerance * max(1, maximum_cost):
            raise ValueError("motion cost budget exceeded after retiming")
        diagnostics.update(checks)
        diagnostics.update(physical_validity="unassessed", execution_ready=False)
        diagnostics["stage"] = "preview_only"
        return candidate, diagnostics
    except Exception as error:
        diagnostics["error"] = str(error)
        return None, diagnostics
