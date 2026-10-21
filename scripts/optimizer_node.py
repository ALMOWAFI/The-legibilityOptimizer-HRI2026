#!/usr/bin/env python3
"""Optimizer node: baseline trajectory -> FK-based solve -> trajectory proposal.

Read handle_optimize() for the solve and TopicOptimizer for the topic handoff.
The standalone mathematics live in experimental/legibility_optimizer.
Published proposals still require downstream retiming and physical validation.
"""
from copy import deepcopy
from dataclasses import asdict
import json
import sys

import numpy as np

from experimental.legibility_optimizer.optimizer_core import OptimizerConfig
from experimental.legibility_optimizer.ros1_adapter import extract_path
from experimental.legibility_optimizer.sampled_optimizer import optimize_sampled_path
from legibility_ros.contracts import positive, replace_path
from legibility_ros.layout import snapshot_layout, validate_goals
from legibility_ros.pipeline import optimize_preview
from legibility_ros.runtime import Runtime, json_status
from legibility_ros.topics import TopicWorker


class OptimizerNode(Runtime):
    """Optimize joint positions using FK, with explicit numerical acceptance."""

    def handle_optimize(self, request):
        response = self.services.OptimizeLegibilityResponse()
        diagnostics = {"execution_ready": False, "physical_validity": "unassessed",
                       "config_sha256": self.config_sha256, "trial_id": request.trial_id}
        with self.lock:
            try:
                if not request.trial_id.strip() or request.goal_header.frame_id != self.frame:
                    raise ValueError("nonempty trial_id and matching configured goal frame required")
                goals = np.array([[p.x, p.y, p.z] for p in request.goals])
                goals = validate_goals(goals, self.cfg["safety"])
                if not 0 <= request.intended_goal_index < len(goals):
                    raise ValueError("intended_goal_index is out of range")
                names, fk, bounds, validate = self.context(request.start_state, request.baseline)
                _, q, _ = extract_path(request.baseline)
                if np.linalg.norm(fk(q[-1]) - goals[request.intended_goal_index]) > 0.005:
                    raise ValueError("baseline endpoint misses the intended hover goal by more than 5 mm")
                config = OptimizerConfig(
                    cost_ratio=self.rospy.get_param("~cost_ratio", 1.2),
                    beta=self.rospy.get_param("~beta", 4.0),
                    maxiter=self.rospy.get_param("~maxiter", 80))
                deviation = positive(self.rospy.get_param("~max_joint_deviation_rad", 0.08),
                                     "maximum joint deviation")
                constraints = {"joint_lower": bounds[:, 0], "joint_upper": bounds[:, 1],
                               "max_joint_deviation": np.full(len(names), deviation)}
                support_points = self.rospy.get_param("~support_points", 11)
                if request.preview_validation:
                    candidate, status = optimize_preview(
                        request.baseline, request.start_state, goals, request.intended_goal_index,
                        fk, self.group, validate, self.rospy.Duration.from_sec, config=config,
                        support_points=support_points, scale=self.scale, constraints=constraints)
                else:
                    _, q, times = extract_path(request.baseline)
                    result = optimize_sampled_path(q, times, goals, request.intended_goal_index, fk,
                                                   support_points=support_points, config=config,
                                                   **constraints)
                    status = asdict(result)
                    del status["positions"], status["times"]
                    status.update(sample_checks_passed=False, physical_validity="unassessed",
                                  retiming_required=True, execution_ready=False, stage="numerical_candidate")
                    candidate = (replace_path(request.baseline, result.positions, result.times,
                                              self.rospy.Duration.from_sec)
                                 if result.numerical_accepted else None)
                diagnostics.update(status, fk_service_calls=fk.calls)
                response.solver_success = status["solver_success"]
                response.numerical_accepted = status["numerical_accepted"]
                response.sample_checks_passed = status["sample_checks_passed"]
                if candidate is not None:
                    response.candidate = candidate
            except Exception as error:
                diagnostics["error"] = str(error)
            response.execution_ready = False
            response.diagnostics = json_status(diagnostics)
        return response


class TopicOptimizer(TopicWorker):
    def __init__(self, runtime, messages=None, string_type=None, start_job=None, subscribe=True):
        super().__init__(runtime, messages, string_type, start_job)
        self.candidate = self.ros.Publisher('/legibility/optimized_candidate', self.messages.OptimizedCandidate,
                                            queue_size=10, latch=False)
        self.proposal = self.ros.Publisher('/legibility/trajectory_proposal', self.messages.TrajectoryProposal,
                                          queue_size=10, latch=False)
        self.subscribers = []
        if subscribe:
            self.subscribers.append(self.ros.Subscriber('/legibility/baseline', self.messages.BaselinePlan,
                                                       self.on_baseline, queue_size=10))

    def on_baseline(self, message):
        source = deepcopy(message)
        try:
            if not source.trial_id.strip() or source.condition not in ('baseline', 'legible'):
                raise ValueError('invalid trial metadata')
            if (source.header.frame_id != self.runtime.frame or source.planning_group != self.runtime.group_name
                    or source.ee_link != self.runtime.link or source.model_name != self.runtime.model_name
                    or source.config_sha256 != self.runtime.config_sha256):
                raise ValueError('baseline frame/model/group/tool/config do not match this optimizer')
            layout, ids, goals = snapshot_layout(source.layout, self.runtime.frame, self.runtime.cfg['safety'])
            if source.target_id not in ids:
                raise ValueError('target_id missing from baseline layout')
            extract_path(source.trajectory)
            self.submit(source.trial_id, layout.layout_id, lambda: self.optimize(source, ids, goals))
        except Exception as error:
            self.report(source.trial_id, source.layout.layout_id, 'baseline_rejected', False, str(error))

    def optimize(self, source, ids, goals):
        if source.condition == 'baseline':
            self.report(source.trial_id, source.layout.layout_id, 'optimizer_bypassed', True,
                        'baseline condition: numerical optimization bypassed')
            self.publish_proposal(source, source.trajectory, False, '{"optimization": "bypassed"}')
            return
        request = self.runtime.services.OptimizeLegibilityRequest()
        request.trial_id, request.baseline, request.start_state = source.trial_id, source.trajectory, source.start_state
        request.goal_header.frame_id = source.layout.header.frame_id
        request.goals = [self.runtime.Point(*map(float, xyz)) for xyz in goals]
        request.intended_goal_index = ids.index(source.target_id)
        request.preview_validation = False
        self.report(source.trial_id, source.layout.layout_id, 'optimizing', True, 'FK-based numerical optimization')
        response = self.runtime.handle_optimize(request)
        if not response.numerical_accepted:
            self.report(source.trial_id, source.layout.layout_id, 'optimizer_rejected', False, response.diagnostics)
            return
        if not response.solver_success or response.execution_ready:
            raise ValueError('optimizer returned inconsistent acceptance/readiness flags')
        names0, q0, t0 = extract_path(source.trajectory)
        names, q, t = extract_path(response.candidate)
        if (names != names0 or q.shape != q0.shape or not np.array_equal(t, t0)
                or not np.array_equal(q[[0, -1]], q0[[0, -1]])):
            raise ValueError('candidate changed joint order, timestamp grid or endpoints')
        metrics = json.loads(response.diagnostics)
        if metrics.get('trial_id') != source.trial_id or metrics.get('config_sha256') != source.config_sha256:
            raise ValueError('optimizer diagnostics do not match the baseline trial/config')
        output = self.messages.OptimizedCandidate()
        output.source, output.trajectory = source, response.candidate
        output.solver_success, output.numerical_accepted = True, True
        output.retiming_required, output.physical_validation_required, output.execution_ready = True, True, False
        for key in ('baseline_legibility', 'optimized_legibility', 'baseline_joint_cost', 'optimized_joint_cost'):
            setattr(output, key, float(metrics[key]))
        output.diagnostics = response.diagnostics
        self.candidate.publish(output)
        self.publish_proposal(source, output.trajectory, True, output.diagnostics)
        self.report(source.trial_id, source.layout.layout_id, 'candidate_published', True,
                    'numerically accepted; downstream retiming and physical validation required')

    def publish_proposal(self, source, trajectory, optimized, diagnostics):
        output = self.messages.TrajectoryProposal()
        output.source, output.trajectory = deepcopy(source), deepcopy(trajectory)
        output.optimization_applied = optimized
        output.retiming_required = output.physical_validation_required = True
        output.execution_ready = False
        output.diagnostics = diagnostics
        self.proposal.publish(output)
        self.report(source.trial_id, source.layout.layout_id, 'proposal_published', True,
                    'awaiting separate retiming/validation; no execution requested')


def main():
    import rospy
    import moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("optimizer")
    try:
        node = OptimizerNode()
        topics = TopicOptimizer(node)
        service = rospy.Service("~optimize", node.services.OptimizeLegibility, node.handle_optimize)
        rospy.loginfo("optimizer ready for preview requests only; config %s", node.config_sha256)
        rospy.spin()
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
