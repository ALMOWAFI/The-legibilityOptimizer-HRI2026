"""Real optimizer and downstream preparation, with fake robot transport only."""
from collections import OrderedDict
from copy import deepcopy
from types import SimpleNamespace as NS
import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np

from legibility_execution.ros_node import Backend
from legibility_execution.trajectory import check_curve
from experimental.legibility_optimizer.ros1_adapter import extract_path
import test_experiment_client as client_tests
from test_preview import IdentityFK, Duration


class PreparationTests(unittest.TestCase):
    def setUp(self):
        self.fixture = client_tests.ClientTests()
        self.fixture.setUp()
        self.b = Backend.__new__(Backend)
        b = self.b
        b.rt = self.fixture.engine
        b.rt.names = tuple(b.rt.names)
        b.ros = NS(Duration=NS(from_sec=Duration))
        b.settings = dict(beta=4., cost_ratio=1.45, max_joint_deviation=.25)
        b.data_lock = threading.Lock()
        b.proposals = OrderedDict()
        b.expected_layout = self.fixture.client.expected_layout.layout_id
        seed = NS(is_diff=False, joint_state=NS(name=list(b.rt.names) + ['gripper'],
                position=[0., 0., 0., .7], velocity=[], effort=[]),
                multi_dof_joint_state=NS(marker='full state'), attached_collision_objects=['tool'])
        b.scene_context = Mock(return_value=('frozen-scene', seed, {'gripper': .7}))
        b.measurements = Mock()
        self.checks = []
        def checks(path, state):
            self.checks.append(deepcopy(state))
            report = check_curve(path, IdentityFK(), lambda q: True, lambda q: np.eye(3),
                                [[-2., 2.]]*3, [10.]*3, [100.]*3,
                                dict(workspace_min=[-2.]*3, workspace_max=[2.]*3,
                                     max_tcp_speed=10, max_joint_step_deg=2))
            return report, IdentityFK()
        b.check_path = checks

    def proposal(self, condition='predictable'):
        result = self.fixture.prepare(condition)
        self.b.proposals[result.source.trial_id] = 'exact-message'
        return result

    def prepare(self, proposal):
        with patch('legibility_execution.ros_node.wire_digest', return_value='exact-message'):
            return self.b.prepare(proposal, 4.)

    def test_baseline_real_retiming_curve_checks_full_state_preservation_no_controller(self):
        proposal = self.proposal()
        result = self.prepare(proposal)
        np.testing.assert_array_equal(extract_path(result['path'])[1], extract_path(proposal.trajectory)[1])
        self.assertEqual(result['duration'], 4.)
        self.assertEqual(self.checks[0].attached_collision_objects, ['tool'])
        self.assertEqual(self.checks[0].joint_state.position[-1], .7)
        self.b.rt.group.execute.assert_not_called()
        self.assertFalse(hasattr(self.b, 'client'))

    def test_real_solver_result_survives_retimed_metric_checks(self):
        proposal = self.proposal('legible')
        result = self.prepare(proposal)
        self.assertIn('retimed_legibility', result['detail'])
        np.testing.assert_array_equal(extract_path(result['path'])[1][[0, -1]],
                                      extract_path(proposal.source.trajectory)[1][[0, -1]])

    def test_altered_endpoints_readiness_and_model_are_rejected(self):
        proposal = self.proposal()
        for mutate in (lambda p: setattr(p, 'execution_ready', True),
                       lambda p: setattr(p.source, 'model_name', 'UR5'),
                       lambda p: p.trajectory.joint_trajectory.points[-1].positions.__setitem__(0, .9)):
            bad = deepcopy(proposal)
            mutate(bad)
            with self.assertRaises(ValueError):
                self.prepare(bad)
        self.assertFalse(self.checks)

    def test_scene_change_during_validation_is_not_accepted(self):
        proposal = self.proposal()
        before = self.b.scene_context.return_value
        self.b.scene_context.side_effect = [before, ('changed', before[1], before[2])]
        with self.assertRaisesRegex(ValueError, 'changed during validation'):
            self.prepare(proposal)

    def test_wrong_proposal_transport_payload_and_duration_rejected(self):
        proposal = self.proposal()
        self.b.proposals[proposal.source.trial_id] = 'different-message'
        with self.assertRaisesRegex(ValueError, 'exact proposal'):
            self.prepare(proposal)
        self.b.proposals[proposal.source.trial_id] = 'exact-message'
        with patch('legibility_execution.ros_node.wire_digest', return_value='exact-message'):
            with self.assertRaisesRegex(ValueError, 'duration'):
                self.b.prepare(proposal, 3.)

    def test_changed_configuration_cannot_pass_even_with_valid_math(self):
        proposal = self.proposal('legible')
        proposal.source.config_sha256 = 'different'
        with self.assertRaisesRegex(ValueError, 'metadata'):
            self.prepare(proposal)


if __name__ == '__main__':
    unittest.main()
