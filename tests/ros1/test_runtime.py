"""Exercise actual node handlers with ROS messages/services replaced at the boundary."""
from copy import deepcopy
import json
import sys
from types import SimpleNamespace as NS
import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np

from scripts.planner_node import PlannerNode
from scripts.optimizer_node import OptimizerNode
from scripts import planner_node, optimizer_node
from test_preview import Duration, IdentityFK, seed_state, trajectory, synthetic_case


def response():
    return NS(solver_success=False, numerical_accepted=False, sample_checks_passed=False,
              execution_ready=False, baseline_planned=False, candidate=None, baseline=None,
              diagnostics="")


def query():
    return NS(goal_header=NS(frame_id=""))


def pose():
    return NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.q, self.t, self.goals = synthetic_case()
        self.path = trajectory(self.q, self.t)
        self.node = OptimizerNode.__new__(OptimizerNode)
        self.node.lock = threading.Lock()
        self.node.config_sha256, self.node.frame = "test-digest", "arm_base_link"
        self.node.model_name = "cu_ur5e"
        self.node.ids, self.node.goals = [0, 1, 2], self.goals
        self.node.scale = 0.2
        self.node.cfg = {'safety': {'workspace_min': [-2, -2, -2], 'workspace_max': [2, 2, 2]}}
        self.node.services = NS(OptimizeLegibilityResponse=response, PlanPreviewResponse=response,
                                OptimizeLegibilityRequest=query, OptimizeLegibility=object)
        self.node.Point = lambda x, y, z: NS(x=x, y=y, z=z)
        self.node.Pose = pose
        self.params = {"~cost_ratio": 1.45, "~max_joint_deviation_rad": 0.25, "~maxiter": 200}
        self.node.rospy = NS(get_param=lambda name, default=None: self.params.get(name, default),
                             Duration=NS(from_sec=Duration))
        fk = IdentityFK()
        fk.calls = 0
        self.node.context = Mock(return_value=(['j0', 'j1', 'j2'], fk,
                                                np.array([[-2., 2.]] * 3),
                                                lambda path: dict(sample_checks_passed=True,
                                                                  execution_ready=False,
                                                                  physical_validity="unassessed")))
        def retime(seed, path, **kwargs):
            for p in path.joint_trajectory.points:
                p.velocities, p.accelerations = [0.] * 3, [0.] * 3
            return path
        self.node.group = NS(retime_trajectory=retime)
        self.planner = PlannerNode.__new__(PlannerNode)
        self.planner.__dict__.update(self.node.__dict__)
        self.planner.lock = threading.Lock()

    def test_node_entrypoints_keep_service_names_and_display_topic(self):
        for module, name, cls, wiring, instance, endpoint, service_type, callback in (
                (planner_node, 'planner', 'PlannerNode', 'TopicPlanner', self.planner,
                 '~plan', 'PlanPreview', 'handle_plan'),
                (optimizer_node, 'optimizer', 'OptimizerNode', 'TopicOptimizer', self.node,
                 '~optimize', 'OptimizeLegibility', 'handle_optimize')):
            with self.subTest(node=name):
                ros = NS(init_node=Mock(), Publisher=Mock(), Service=Mock(), loginfo=Mock(), spin=Mock())
                commander = NS(roscpp_initialize=Mock(), roscpp_shutdown=Mock())
                instance.msg = NS(DisplayTrajectory=object)
                setattr(instance.services, service_type, object)
                with patch.dict(sys.modules, rospy=ros, moveit_commander=commander), \
                     patch.object(module, cls, return_value=instance), \
                     patch.object(module, wiring) as topics:
                    module.main()
                ros.init_node.assert_called_once_with(name)
                topics.assert_called_once_with(instance)
                ros.Service.assert_called_once_with(endpoint, object, getattr(instance, callback))
                ros.spin.assert_called_once()
                commander.roscpp_shutdown.assert_called_once()
                if name == 'planner':
                    ros.Publisher.assert_called_once_with('~display', object, queue_size=1, latch=True)
                else:
                    ros.Publisher.assert_not_called()

    def test_node_initialization_failure_does_not_register_services(self):
        for module, cls in ((planner_node, 'PlannerNode'), (optimizer_node, 'OptimizerNode')):
            with self.subTest(node=cls):
                ros = NS(init_node=Mock(), Service=Mock(), spin=Mock())
                commander = NS(roscpp_initialize=Mock(), roscpp_shutdown=Mock())
                with patch.dict(sys.modules, rospy=ros, moveit_commander=commander), \
                     patch.object(module, cls, side_effect=ValueError('missing robot configuration')):
                    with self.assertRaisesRegex(ValueError, 'missing robot configuration'):
                        module.main()
                ros.Service.assert_not_called()
                ros.spin.assert_not_called()
                commander.roscpp_shutdown.assert_called_once()

    def optimize_request(self):
        return NS(trial_id="trial-1", goal_header=NS(frame_id=self.node.frame),
                  goals=[self.node.Point(*g) for g in self.goals], intended_goal_index=0,
                  start_state=seed_state(), baseline=self.path, preview_validation=True)

    def test_optimizer_handler_runs_real_solver_and_returns_preview_status(self):
        result = self.node.handle_optimize(self.optimize_request())
        self.assertTrue(result.numerical_accepted, result.diagnostics)
        self.assertTrue(result.sample_checks_passed, result.diagnostics)
        self.assertFalse(result.execution_ready)
        status = json.loads(result.diagnostics)
        self.assertEqual(status['trial_id'], 'trial-1')
        self.assertEqual(status['physical_validity'], 'unassessed')
        self.assertIsNotNone(result.candidate)

    def test_mismatched_or_stale_trial_data_rejected_before_fk(self):
        for mutate in (lambda r: setattr(r.goal_header, 'frame_id', 'base'),
                       lambda r: setattr(r.goals[1], 'x', 50),
                       lambda r: setattr(r, 'intended_goal_index', 99),
                       lambda r: setattr(r, 'trial_id', '')):
            request = self.optimize_request()
            mutate(request)
            result = self.node.handle_optimize(request)
            self.assertFalse(result.numerical_accepted)
            self.assertFalse(result.sample_checks_passed)
            self.assertIsNone(result.candidate)
            self.assertIn('error', json.loads(result.diagnostics))
        self.node.context.assert_not_called()

    def test_endpoint_must_match_selected_goal(self):
        request = self.optimize_request()
        request.intended_goal_index = 1
        result = self.node.handle_optimize(request)
        self.assertIn('endpoint', json.loads(result.diagnostics)['error'])
        self.assertFalse(result.numerical_accepted)

    def test_planner_rejection_keeps_baseline_and_never_displays_bad_candidate(self):
        self.planner.plan_baseline = Mock(return_value=(seed_state(), self.path, {'sample_checks_passed': True}))
        self.planner.display = Mock()
        optimizer_result = response()
        optimizer_result.diagnostics = json.dumps({'trial_id': 'trial-1', 'config_sha256': 'test-digest'})
        remote = Mock(return_value=optimizer_result)
        self.planner.proxy = Mock(return_value=remote)
        request = NS(trial_id='trial-1', target_id=0, baseline_mode='cartesian', optimize=True, preview_validation=True)
        result = self.planner.handle_plan(request)
        self.assertTrue(result.baseline_planned)
        self.assertFalse(result.numerical_accepted)
        self.assertIsNone(result.candidate)
        self.assertEqual(self.planner.display.call_count, 1)
        sent = remote.call_args.args[0]
        self.assertEqual(sent.goal_header.frame_id, 'arm_base_link')
        self.assertEqual(sent.intended_goal_index, 0)
        self.assertEqual(sent.trial_id, 'trial-1')

    def test_display_uses_urdf_model_name_and_dedicated_publisher(self):
        self.planner.msg = NS(DisplayTrajectory=lambda: NS())
        self.planner.preview = Mock()
        self.planner.display(seed_state(), self.path)
        sent = self.planner.preview.publish.call_args.args[0]
        self.assertEqual(sent.model_id, 'cu_ur5e')
        self.assertEqual(sent.trajectory, [self.path])

    def test_cartesian_planner_checks_fraction_and_service_flags(self):
        self.planner.names, self.planner.group_name, self.planner.link = ['j0', 'j1', 'j2'], 'arm', 'arm_tool0'
        self.planner.max_fk_calls = 100
        self.planner.cfg = {'motion': {'start_joints': dict(zip(self.planner.names, self.q[0])), 'duration': 4},
                         'safety': {'max_joint_step_deg': 2}}
        self.planner.robot = NS(get_current_state=seed_state)
        self.planner.fk_service = Mock()
        self.planner.srv = NS(GetPositionFKRequest=object, GetCartesianPath=object,
                            GetCartesianPathRequest=lambda: NS(header=NS()))
        remote = Mock(return_value=NS(error_code=NS(val=1), fraction=0.8))
        self.planner.proxy = Mock(return_value=remote)
        request = NS(trial_id='t', target_id=0, baseline_mode='cartesian')
        with patch('scripts.planner_node.FK', return_value=IdentityFK()):
            with self.assertRaisesRegex(ValueError, 'complete'):
                self.planner.plan_baseline(request)
        sent = remote.call_args.args[0]
        self.assertTrue(sent.avoid_collisions)
        self.assertEqual(sent.group_name, 'arm')
        self.assertEqual(sent.header.frame_id, 'arm_base_link')
        self.assertEqual(len(sent.waypoints), 400)
        self.assertEqual(sent.link_name, 'arm_tool0')


if __name__ == '__main__':
    unittest.main()
