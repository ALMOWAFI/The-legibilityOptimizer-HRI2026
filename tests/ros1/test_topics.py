"""Topic-to-topic integration: fake ROS/MoveIt transport, real SLSQP optimizer.

This validates wiring, not the RRT implementation or physical UR5e kinematics.
"""
from copy import deepcopy
import json
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

from legibility_ros.layout import layout_digest, snapshot_layout
from legibility_ros.topics import TopicPlanner, TopicOptimizer
import test_runtime as runtime_tests
from test_preview import IdentityFK, seed_state


def header():
    return NS(frame_id='', stamp=0)


MESSAGES = NS(
    BlockLayout=lambda: NS(header=header(), layout_id='', blocks=[], goal_tool_z=0.),
    Block=lambda: NS(id=0, name='', pose=runtime_tests.pose()),
    TrialRequest=lambda: NS(header=header(), trial_id='', target_id=0, condition='', layout_id=''),
    BaselinePlan=lambda: NS(header=header()), OptimizedCandidate=lambda: NS(),
    TrajectoryProposal=lambda: NS(), MotionFeedback=lambda: NS(header=header()))


class Publisher:
    def __init__(self, bus, name, latch):
        self.bus, self.name, self.latch, self.sent = bus, name, latch, []

    def publish(self, message):
        self.sent.append(deepcopy(message))
        for callback in self.bus.subscribers.get(self.name, []):
            callback(deepcopy(message))

    def get_num_connections(self):
        return len(self.bus.subscribers.get(self.name, []))


class Bus:
    def __init__(self, params):
        self.params, self.publishers, self.subscribers = params, {}, {}
        self.Time = NS(now=lambda: 123)
        self.Duration = NS(from_sec=runtime_tests.Duration)

    def get_param(self, name, default=None):
        return self.params.get(name, default)

    def is_shutdown(self):
        return False

    def Publisher(self, name, kind, queue_size, latch=False):
        if name not in self.publishers:
            self.publishers[name] = Publisher(self, name, latch)
        return self.publishers[name]

    def Subscriber(self, name, kind, callback, queue_size):
        self.subscribers.setdefault(name, []).append(callback)
        pub = self.publishers.get(name)
        if pub and pub.latch and pub.sent:
            callback(deepcopy(pub.sent[-1]))
        return NS(name=name)


class TopicTests(unittest.TestCase):
    def setUp(self):
        template = runtime_tests.RuntimeTests()
        template.setUp()
        self.engine, self.q, self.t, self.goals, self.path = template.node, template.q, template.t, template.goals, template.path
        self.engine.names, self.engine.group_name, self.engine.link = ['j0', 'j1', 'j2'], 'arm', 'arm_tool0'
        self.engine.max_fk_calls, self.engine.orientation_tolerance = 1000, 0.05
        self.engine.cfg['motion'] = {'start_joints': dict(zip(self.engine.names, self.q[0])), 'duration': 4}
        self.engine.cfg['safety']['max_joint_step_deg'] = 2
        self.engine.robot = NS(get_current_state=seed_state)
        self.engine.fk_service = Mock()
        self.engine.srv = NS(GetPositionFKRequest=object)
        self.engine.services.PlanPreviewRequest = lambda: NS()
        self.engine.msg = NS(OrientationConstraint=lambda: NS(header=header()), Constraints=lambda: NS())
        self.engine.display = Mock()
        retime = self.engine.group.retime_trajectory
        self.engine.group = Mock()
        self.engine.group.retime_trajectory.side_effect = retime
        self.engine.group.plan.return_value = (True, self.path, 0.01, NS(val=1))
        params = dict(template.params, **{
            '/move_group/planner_configs': {'RRTConnectLab': {'type': 'geometric::RRTConnect'}},
            '/move_group/arm/planner_configs': ['RRTConnectLab']})
        self.bus = self.engine.rospy = Bus(params)
        self.layout = MESSAGES.BlockLayout()
        self.layout.header.frame_id = self.engine.frame
        for i, goal in enumerate(self.goals):
            block = MESSAGES.Block()
            block.id, block.name = 10 * (i + 1), 'B' + str(i + 1)
            block.pose.position.x, block.pose.position.y, block.pose.position.z = float(goal[0]), float(goal[1]), -0.07
            self.layout.blocks.append(block)
        self.layout.layout_id = layout_digest(self.layout)
        # Publish BEFORE planner startup to check persistent latched layout semantics.
        self.bus.Publisher('/hri/block_layout', MESSAGES.BlockLayout, 1, latch=True).publish(self.layout)
        self.optimizer = TopicOptimizer(self.engine, MESSAGES, NS, start_job=lambda work: work())
        self.planner = TopicPlanner(self.engine, MESSAGES, NS, start_job=lambda work: work())

    def trial(self, condition='legible', trial_id='trial-1'):
        return NS(header=header(), trial_id=trial_id, target_id=10, condition=condition, layout_id=self.layout.layout_id)

    def send(self, trial):
        with patch('legibility_ros.runtime.FK', return_value=IdentityFK()):
            self.planner.on_trial(trial)

    def stages(self):
        return [json.loads(m.data)['stage'] for m in self.bus.publishers['/legibility/status'].sent]

    def test_end_to_end_rrt_request_real_optimizer_and_candidate_publication(self):
        self.send(self.trial())
        self.assertEqual(set(self.bus.subscribers),
                         {'/hri/block_layout', '/legibility/trial_request', '/legibility/baseline'})
        self.engine.group.set_planner_id.assert_called_once_with('RRTConnectLab')
        self.engine.group.set_planning_pipeline_id.assert_called_once_with('ompl')
        self.engine.group.clear_pose_targets.assert_called_once()
        self.engine.group.clear_path_constraints.assert_called_once()
        self.assertEqual(len(self.optimizer.candidate.sent), 1, self.stages())
        output = self.optimizer.candidate.sent[0]
        self.assertTrue(output.solver_success and output.numerical_accepted)
        self.assertGreater(output.optimized_legibility, output.baseline_legibility)
        self.assertLessEqual(output.optimized_joint_cost, 1.45 * output.baseline_joint_cost + 1e-7)
        self.assertTrue(output.retiming_required and output.physical_validation_required)
        self.assertFalse(output.execution_ready)
        self.assertEqual(len(self.optimizer.proposal.sent), 1)
        self.assertTrue(self.optimizer.proposal.sent[0].optimization_applied)
        self.assertFalse(self.optimizer.proposal.sent[0].execution_ready)
        self.assertEqual(output.source.layout.layout_id, self.layout.layout_id)
        self.assertEqual(output.source.target_id, 10)
        points = output.trajectory.joint_trajectory.points
        np.testing.assert_array_equal(points[0].positions, self.q[0])
        np.testing.assert_array_equal(points[-1].positions, self.q[-1])
        self.assertTrue(all(p.velocities == [] and p.accelerations == [] for p in points))
        self.engine.group.execute.assert_not_called()
        self.engine.group.go.assert_not_called()

    def test_predictable_condition_bypasses_optimizer(self):
        with patch.object(self.engine, 'handle_optimize') as optimize:
            self.send(self.trial('baseline'))
            optimize.assert_not_called()
        self.assertEqual(len(self.planner.baseline.sent), 1)
        self.assertEqual(len(self.optimizer.candidate.sent), 0)
        self.assertEqual(len(self.optimizer.proposal.sent), 1)
        self.assertFalse(self.optimizer.proposal.sent[0].optimization_applied)
        np.testing.assert_array_equal(
            [p.positions for p in self.optimizer.proposal.sent[0].trajectory.joint_trajectory.points], self.q)
        self.assertIn('optimizer_bypassed', self.stages())

    def test_missing_layout_unknown_target_and_mismatched_layout_do_not_plan(self):
        trial = self.trial()
        trial.target_id = 999
        self.send(trial)
        trial = self.trial()
        trial.layout_id = 'unknown-layout'
        self.send(trial)
        self.planner.layout = None
        self.send(self.trial())
        self.engine.group.plan.assert_not_called()

    def test_bad_layout_invalidates_previous_layout(self):
        bad = deepcopy(self.layout)
        bad.blocks[0].pose.position.x += 0.1  # Digest is now stale.
        self.planner.on_layout(bad)
        self.send(self.trial())
        self.engine.group.plan.assert_not_called()
        self.assertIn('layout_rejected', self.stages())

    def test_duplicate_trials_are_not_replanned_or_republished(self):
        self.send(self.trial())
        self.send(self.trial())
        self.engine.group.plan.assert_called_once()
        self.assertEqual(len(self.optimizer.candidate.sent), 1)
        self.assertIn('rejected_duplicate', self.stages())

    def test_inflight_layout_snapshot_does_not_change_when_new_layout_arrives(self):
        pending = []
        self.planner._start_job = pending.append
        self.planner.on_trial(self.trial())
        replacement = deepcopy(self.layout)
        replacement.blocks[1].pose.position.y += 0.1
        replacement.layout_id = layout_digest(replacement)
        self.planner.on_layout(replacement)
        busy = self.trial(trial_id='trial-2')
        busy.layout_id = replacement.layout_id
        self.planner.on_trial(busy)
        self.assertIn('rejected_busy', self.stages())
        with patch('legibility_ros.runtime.FK', return_value=IdentityFK()):
            pending[0]()
        self.assertEqual(self.optimizer.candidate.sent[0].source.layout.layout_id, self.layout.layout_id)

    def test_wrong_planner_type_and_failed_rrt_never_publish_baselines(self):
        self.bus.params['/move_group/planner_configs']['RRTConnectLab']['type'] = 'geometric::PRM'
        self.send(self.trial())
        self.engine.group.plan.assert_not_called()
        self.bus.params['/move_group/planner_configs']['RRTConnectLab']['type'] = 'geometric::RRTConnect'
        self.engine.group.plan.return_value = (False, self.path, 0.01, NS(val=-1))
        self.send(self.trial(trial_id='trial-2'))
        self.assertEqual(len(self.planner.baseline.sent), 0)
        self.assertEqual(len(self.optimizer.candidate.sent), 0)
        self.engine.group.clear_path_constraints.assert_called_once()

    def test_optimizer_rejection_has_status_and_no_candidate(self):
        rejected = runtime_tests.response()
        rejected.diagnostics = '{"error": "no improvement"}'
        with patch.object(self.engine, 'handle_optimize', return_value=rejected):
            self.send(self.trial())
        self.assertEqual(len(self.optimizer.candidate.sent), 0)
        self.assertIn('optimizer_rejected', self.stages())
        self.assertEqual(len(self.optimizer.proposal.sent), 0)

    def test_baseline_metadata_mismatch_and_endpoint_corruption_rejected(self):
        self.send(self.trial('baseline'))
        bad = deepcopy(self.planner.baseline.sent[0])
        bad.trial_id, bad.condition, bad.ee_link = 'bad-1', 'legible', 'wrong-tool'
        self.optimizer.on_baseline(bad)
        self.assertIn('baseline_rejected', self.stages())
        good_source = deepcopy(self.planner.baseline.sent[0])
        good_source.trial_id, good_source.condition = 'bad-2', 'legible'
        reply = runtime_tests.response()
        reply.numerical_accepted, reply.solver_success = True, True
        reply.candidate = deepcopy(self.path)
        reply.candidate.joint_trajectory.points[-1].positions[0] += 0.1
        with patch.object(self.engine, 'handle_optimize', return_value=reply):
            self.optimizer.on_baseline(good_source)
        self.assertEqual(len(self.optimizer.candidate.sent), 0)
        self.assertIn('failed', self.stages())

    def test_group_trial_message_maps_id_condition_and_rejects_nonzero_bend(self):
        request = NS(header=header(), trial_id=42, target_id=10, condition='legible', bend=0.)
        with patch('legibility_ros.runtime.FK', return_value=IdentityFK()):
            self.planner.on_group_trial(request)
        self.assertEqual(self.optimizer.candidate.sent[0].source.trial_id, '42')
        request.trial_id, request.bend = 43, 0.2
        self.planner.on_group_trial(request)
        self.assertEqual(len(self.planner.baseline.sent), 1)
        self.assertIn('trial_rejected', self.stages())

    def test_layout_rejects_wrong_frame_duplicate_ids_and_nonfinite_values(self):
        for change in (lambda b: setattr(b.header, 'frame_id', 'base'),
                       lambda b: setattr(b.blocks[1], 'id', 10),
                       lambda b: setattr(b, 'goal_tool_z', float('nan'))):
            bad = deepcopy(self.layout)
            change(bad)
            with self.assertRaises(ValueError):
                snapshot_layout(bad, self.engine.frame, self.engine.cfg['safety'])


if __name__ == '__main__':
    unittest.main()
