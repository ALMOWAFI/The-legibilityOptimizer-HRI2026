"""Session -> real numeric solver -> proposal, with fake transport and robot."""
from copy import deepcopy
import hashlib
import json
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

from legibility_ros.experiment_client import ExperimentClient
from legibility_ros.runtime import json_status
import test_topics as topic_tests
from test_preview import IdentityFK, Duration, ROOT


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.fixture = topic_tests.TopicTests()
        self.fixture.setUp()
        self.bus, self.engine = self.fixture.bus, self.fixture.engine
        self.bus.Time = NS(now=lambda: Duration(100.), from_sec=Duration)
        self.cfg = self.engine.cfg
        self.cfg.update(frame_id=self.engine.frame, blocks=[
            dict(id=b.id, name=b.name, position=[b.pose.position.x, b.pose.position.y, b.pose.position.z])
            for b in self.fixture.layout.blocks])
        self.cfg['motion'].update(planning_group='arm', ee_link='arm_tool0', touch_z=-.05, hover_height=.05)
        self.cfg['safety'].update(max_tcp_speed=.5, max_joint_speed_deg=60)
        self.services = NS(ValidateTrajectory=object, ValidateTrajectoryRequest=NS,
                           RequestMotion=object, RequestMotionRequest=NS)
        self.client = ExperimentClient(self.bus, self.cfg, topic_tests.MESSAGES, self.services, NS, timeout=.3)
        self.engine.config_sha256 = hashlib.sha256(json_status(self.cfg).encode()).hexdigest()
        self.bus.wait_for_service = Mock()
        self.bus.ServiceProxy = Mock(return_value=Mock(return_value=NS(
            accepted=True, validation_id='validated-1', duration=4., detail='test only')))

    def prepare(self, motion='legible'):
        with patch('legibility_ros.runtime.FK', return_value=IdentityFK()):
            return self.client.prepare(dict(trial_id=1, target_id=10, motion=motion))

    def event(self, state, stamp=100., **overrides):
        command = self.client.active_command
        values = dict(command_id=command.command_id, trial_id=command.trial_id,
                      validation_id=command.validation_id, state=state,
                      header=NS(stamp=Duration(stamp)), detail='')
        values.update(overrides)
        self.client.on_feedback(NS(**values))

    def test_real_solve_through_session_and_common_handoff_no_motion(self):
        output = self.prepare()
        self.assertTrue(output.optimization_applied)
        self.assertFalse(output.execution_ready)
        self.assertEqual(output.source.config_sha256, self.client.config_sha256)
        self.assertEqual(len(self.fixture.optimizer.candidate.sent), 1)
        self.bus.ServiceProxy.assert_not_called()
        self.engine.group.execute.assert_not_called()
        self.assertTrue(self.client.feedback.empty())

    def test_predictable_maps_to_baseline_and_retry_has_fresh_id(self):
        with patch.object(self.engine, 'handle_optimize') as optimize:
            first, second = self.prepare('predictable'), self.prepare('predictable')
        optimize.assert_not_called()
        self.assertEqual(first.source.condition, 'baseline')
        self.assertNotEqual(first.source.trial_id, second.source.trial_id)
        self.assertFalse(first.optimization_applied)
        self.assertEqual(len(self.fixture.optimizer.proposal.sent), 2)

    def test_failed_solver_does_not_fall_back_or_validate(self):
        rejected = NS(numerical_accepted=False, diagnostics='rejected')
        with patch.object(self.engine, 'handle_optimize', return_value=rejected):
            with self.assertRaisesRegex(RuntimeError, 'optimizer_rejected'):
                self.prepare()
        self.assertEqual(len(self.fixture.optimizer.proposal.sent), 0)
        self.bus.ServiceProxy.assert_not_called()

    def test_mismatched_config_rejected_and_late_output_cannot_complete_retry(self):
        self.engine.config_sha256 = 'other-config'
        with self.assertRaisesRegex(RuntimeError, 'configuration'):
            self.prepare('predictable')
        self.assertIsNone(self.client.pending)
        late = self.fixture.optimizer.proposal.sent[0]
        self.client.proposal = None
        self.client.on_proposal(late)
        self.assertIsNone(self.client.proposal)

    def test_layout_change_invalidates_matching_snapshot_and_times_out(self):
        bad = deepcopy(self.fixture.layout)
        bad.blocks[0].pose.position.x += .01
        self.client.on_layout(bad)
        self.assertIsNone(self.client.layout)
        with self.assertRaisesRegex(RuntimeError, 'layout/planner.*timed out'):
            self.prepare()
        self.engine.group.plan.assert_not_called()

    def test_prepare_timeout_uses_wall_clock_and_clears_pending(self):
        self.bus.subscribers['/legibility/trial_request'] = [lambda m: None]
        with self.assertRaisesRegex(RuntimeError, 'preparation.*timed out'):
            self.prepare()
        self.assertIsNone(self.client.pending)
        self.bus.ServiceProxy.assert_not_called()

    def test_absent_validator_fails_explicitly(self):
        self.prepare('predictable')
        self.bus.wait_for_service.side_effect = RuntimeError('no provider')
        with self.assertRaisesRegex(RuntimeError, 'unavailable'):
            self.client.validate()
        self.assertIsNone(self.client.validation_id)
        self.bus.ServiceProxy.assert_not_called()

    def test_validation_must_be_separate_and_duration_must_match(self):
        self.prepare('predictable')
        with self.assertRaisesRegex(RuntimeError, 'confirmation'):
            self.client.request_motion('run')
        with self.assertRaisesRegex(RuntimeError, 'validated'):
            self.client.request_motion('run', operator_confirmed=True)
        self.bus.ServiceProxy.return_value.return_value.duration = 8.
        with self.assertRaisesRegex(RuntimeError, 'duration'):
            self.client.validate()
        self.assertIsNone(self.client.validation_id)

    def test_correlation_onset_and_timestamp_order_and_single_use_validation(self):
        self.prepare('predictable')
        self.client.validate()
        self.client.request_motion('run', operator_confirmed=True)
        self.event('done')  # No measured onset yet.
        self.event('executing', command_id='old-attempt')
        self.event('executing', validation_id='old-validation')
        self.assertTrue(self.client.feedback.empty())
        self.event('executing', stamp=100)
        self.event('done', stamp=99)
        self.assertEqual(self.client.wait_status({'executing'}, .1), ('executing', 100))
        self.assertTrue(self.client.feedback.empty())
        self.event('done', stamp=104)
        self.assertEqual(self.client.wait_status({'done'}, .1), ('done', 104))
        self.event('executing', stamp=105)
        self.assertTrue(self.client.feedback.empty())
        with self.assertRaisesRegex(RuntimeError, 'validated'):
            self.client.request_motion('run', operator_confirmed=True)
        with self.assertRaisesRegex(RuntimeError, 'fresh'):
            self.client.validate()

    def test_home_requires_confirmation_and_is_not_a_trial_onset(self):
        with self.assertRaisesRegex(RuntimeError, 'confirmation'):
            self.client.request_motion('home')
        self.client.request_motion('home', operator_confirmed=True)
        self.event('executing')
        self.assertTrue(self.client.feedback.empty())
        self.event('home')
        self.assertEqual(self.client.wait_status({'home'}, .1), ('home', 100))

    def test_rejected_motion_service_does_not_generate_feedback(self):
        self.bus.ServiceProxy.return_value.return_value.accepted = False
        with self.assertRaisesRegex(RuntimeError, 'rejected'):
            self.client.request_motion('home', operator_confirmed=True)
        self.assertIsNone(self.client.active_command)
        self.assertTrue(self.client.feedback.empty())
        with self.assertRaisesRegex(RuntimeError, 'unresolved'):
            self.client.request_motion('home', operator_confirmed=True)

    def test_group_launch_does_not_start_legacy_motion_or_trial_bridge(self):
        launch = ET.parse(ROOT / 'launch/group_pipeline.launch').getroot()
        self.assertEqual([n.attrib['type'] for n in launch.findall('.//node')], ['display_node.py'])
        args = {a.attrib['name']: a.attrib['value'] for a in launch.findall('./include/arg')}
        self.assertEqual(args['subscribe_group_trials'], 'false')
        cmake = (ROOT / 'CMakeLists.txt').read_text()
        for name in ('TrajectoryProposal.msg', 'MotionFeedback.msg', 'ValidateTrajectory.srv', 'RequestMotion.srv'):
            self.assertIn(name, cmake)


if __name__ == '__main__':
    unittest.main()
