"""Offline downstream tests. Every controller/ROS boundary is a fake."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
import ast
import threading
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import numpy as np

from legibility_execution.gate import ExecutionGate
from legibility_execution.state import ArmState, Sample
from legibility_execution.trajectory import check_curve, derivatives, fit_duration, reversed_path, segment, extrema
from legibility_execution.ros_node import Backend, bounded


class Duration:
    def __init__(self, value=0.):
        self.value = float(value)

    def to_sec(self):
        return self.value


def path(q=None, times=None):
    q = np.array(q if q is not None else [[0.], [.02], [.04]])
    times = times if times is not None else np.arange(len(q))
    return NS(joint_trajectory=NS(joint_names=['j%d' % j for j in range(q.shape[1])],
        header=NS(stamp=Duration()), points=[NS(positions=list(row), velocities=[0.] * len(row),
        accelerations=[0.] * len(row), effort=[], time_from_start=Duration(t)) for row, t in zip(q, times)]),
        multi_dof_joint_trajectory=NS(points=[], joint_names=[]))


class FK:
    def pose(self, q):
        return np.array([q[0], 0., 0.]), np.array([0., 0., 0., 1.])


class CurveTests(unittest.TestCase):
    def check(self, trajectory=None, **changes):
        values = dict(fk=FK(), state_valid=lambda q: True, jacobian=lambda q: [[1.], [0.], [0.]],
                      bounds=[[-1, 1]], vmax=[10.], amax=[100.], safety=dict(
                          workspace_min=[-2]*3, workspace_max=[2]*3, max_tcp_speed=10, max_joint_step_deg=2))
        values.update(changes)
        return check_curve(trajectory or path(), **values)

    def test_six_boundary_conditions_and_midpoint_independent_reference(self):
        poly = segment(0., 1., 0., 0., 0., 0., 2.)
        self.assertAlmostEqual(poly(.5), .5)
        self.assertAlmostEqual(poly.deriv()(.5) / 2, .9375)
        for point, position in [(0., 0.), (1., 1.)]:
            self.assertAlmostEqual(poly(point), position)
            self.assertAlmostEqual(poly.deriv()(point), 0.)
            self.assertAlmostEqual(poly.deriv(2)(point), 0.)
        self.assertAlmostEqual(extrema(poly.deriv())[1], 1.875)

    def test_checks_actual_curve_between_knots(self):
        report = self.check()
        self.assertGreater(report['checked_states'], 3)
        self.assertFalse(report['continuous_collision_certified'])
        with self.assertRaisesRegex(ValueError, 'state validity'):
            self.check(state_valid=lambda q: not .005 < q[0] < .015)

    def test_overshoot_rejected_even_when_all_knot_positions_in_bounds(self):
        trajectory = path([[0.], [.9], [.9], [0.]])
        trajectory.joint_trajectory.points[1].velocities = [2.]
        trajectory.joint_trajectory.points[2].velocities = [-2.]
        with self.assertRaisesRegex(ValueError, 'position limit'):
            self.check(trajectory)

    def test_interior_velocity_and_acceleration_extrema_not_just_knot_fields(self):
        for limits in (dict(vmax=[.03]), dict(amax=[.05])):
            with self.assertRaisesRegex(ValueError, 'velocity/acceleration'):
                self.check(**limits)

    def test_workspace_tcp_orientation_bad_jacobian_and_budget_fail(self):
        settings = dict(workspace_min=[-1]*3, workspace_max=[.01, 1, 1], max_tcp_speed=10, max_joint_step_deg=2)
        with self.assertRaisesRegex(ValueError, 'workspace'):
            self.check(safety=settings)
        settings.update(workspace_max=[1]*3, max_tcp_speed=.01)
        with self.assertRaisesRegex(ValueError, 'TCP speed'):
            self.check(safety=settings)
        with self.assertRaisesRegex(ValueError, 'Jacobian'):
            self.check(jacobian=lambda q: [[float('nan')]]*3)
        with self.assertRaisesRegex(ValueError, 'budget'):
            self.check(max_samples=2)
        class Rotate(FK):
            def pose(self, q):
                return super().pose(q)[0], np.array([0, 0, np.sin(q[0]), np.cos(q[0])])
        with self.assertRaisesRegex(ValueError, 'orientation'):
            self.check(fk=Rotate(), orientation_tolerance=.01)

    def test_missing_callbacks_or_derivatives_never_return_success(self):
        trajectory = path()
        trajectory.joint_trajectory.points[1].accelerations = []
        with self.assertRaisesRegex(ValueError, 'accelerations'):
            self.check(trajectory)
        with self.assertRaises(TypeError):
            self.check(state_valid=None)
        with self.assertRaisesRegex(RuntimeError, 'offline'):
            self.check(state_valid=Mock(side_effect=RuntimeError('offline')))

    def test_fit_duration_rescales_derivatives_keeps_exact_positions_and_rechecks(self):
        trajectory = path()
        trajectory.joint_trajectory.points[1].velocities = [.02]
        trajectory.joint_trajectory.points[1].accelerations = [.01]
        group = NS(retime_trajectory=lambda seed, p, **kw: p)
        result = fit_duration(group, None, trajectory, 4., Duration)
        _, q, t, v, a = derivatives(result)
        np.testing.assert_array_equal(q, [[0.], [.02], [.04]])
        np.testing.assert_array_equal(t, [0., 2., 4.])
        self.assertAlmostEqual(v[1, 0], .01)
        self.assertAlmostEqual(a[1, 0], .0025)
        self.assertEqual(trajectory.joint_trajectory.points[1].velocities, [.02])
        fast = fit_duration(group, None, trajectory, .01, Duration)
        with self.assertRaisesRegex(ValueError, 'velocity/acceleration'):
            self.check(fast)

    def test_bad_retimer_timestamps_and_nonrest_endpoint_reject(self):
        group = NS(retime_trajectory=Mock(side_effect=RuntimeError('retimer missing')))
        with self.assertRaisesRegex(RuntimeError, 'retimer missing'):
            fit_duration(group, None, path(), 4., Duration)
        for mutation in (lambda p: setattr(p.points[0], 'time_from_start', Duration(.1)),
                         lambda p: setattr(p.points[-1], 'velocities', [.1])):
            trajectory = path()
            mutation(trajectory.joint_trajectory)
            with self.assertRaises(ValueError):
                self.check(trajectory)

    def test_home_reverses_positions_and_velocity_sign_and_preserves_acceleration_sign(self):
        trajectory = path()
        trajectory.joint_trajectory.points[1].velocities = [.02]
        trajectory.joint_trajectory.points[1].accelerations = [.01]
        result = reversed_path(trajectory, 4., Duration)
        _, q, t, v, a = derivatives(result)
        np.testing.assert_array_equal(q[:, 0], [.04, .02, 0.])
        np.testing.assert_array_equal(t, [0., 2., 4.])
        self.assertAlmostEqual(v[1, 0], -.01)
        self.assertAlmostEqual(a[1, 0], .0025)


class GateTests(unittest.TestCase):
    def setUp(self):
        self.now = 1.
        self.workers = []
        self.backend = NS(prepare=Mock(return_value=dict(duration=4., detail='checked', path=path())),
                          dispatch_deadline=Mock(return_value=99), prepare_home=Mock(return_value=None),
                          perform=Mock(), failed=Mock())
        self.gate = ExecutionGate(self.backend, 'config', enabled=True, clock=lambda: self.now,
                                  start_worker=self.workers.append)
        self.proposal = NS(source=NS(trial_id='trial', config_sha256='config'))

    def token(self):
        return self.gate.validate(self.proposal, 4.)[0]

    def command(self, token='', **changes):
        data = dict(command_id='command', config_sha256='config', operation='run',
                    operator_confirmed=True, validation_id=token, trial_id='trial')
        data.update(changes)
        return NS(**data)

    def test_validation_never_sends_motion_and_stores_a_copy(self):
        token = self.token()
        self.backend.perform.assert_not_called()
        self.backend.prepare.return_value['path'].joint_trajectory.points[0].positions = [99.]
        self.assertEqual(self.gate.tickets[token].artifact['path'].joint_trajectory.points[0].positions, [0.])

    def test_explicit_gate_and_no_automatic_execution(self):
        token = self.token()
        self.gate.enabled = False
        with self.assertRaisesRegex(RuntimeError, 'disabled'):
            self.gate.request(self.command(token))
        self.gate.enabled = True
        with self.assertRaisesRegex(RuntimeError, 'confirmation'):
            self.gate.request(self.command(token, operator_confirmed=False))
        self.backend.perform.assert_not_called()

    def test_expired_wrong_trial_missing_and_replaced_tokens_reject(self):
        token = self.token()
        self.token()  # replaces previous token
        for request in (self.command(token), self.command('missing'),
                        self.command(self.token(), trial_id='another')):
            with self.assertRaises(ValueError):
                self.gate.request(request)
        token = self.token()
        self.now += 121
        with self.assertRaises(ValueError):
            self.gate.request(self.command(token))

    def test_acceptance_is_prompt_consumes_token_and_excludes_concurrent_work(self):
        token = self.token()
        self.gate.request(self.command(token))
        self.backend.perform.assert_not_called()
        self.assertTrue(self.gate.active)
        with self.assertRaises(RuntimeError):
            self.gate.validate(self.proposal, 4.)
        with self.assertRaises(RuntimeError):
            self.gate.request(self.command(token, command_id='other'))
        self.workers.pop()()
        self.assertFalse(self.gate.active)
        self.backend.perform.assert_called_once()
        with self.assertRaisesRegex(ValueError, 'duplicate'):
            self.gate.request(self.command(token))
        with self.assertRaises(ValueError):
            self.gate.request(self.command(token, command_id='new'))

    def test_failure_latches_and_cannot_trigger_home_retry(self):
        self.backend.perform.side_effect = RuntimeError('controller failed')
        self.gate.request(self.command(self.token()))
        self.workers.pop()()
        self.assertTrue(self.gate.faulted)
        self.backend.failed.assert_called_once()
        with self.assertRaisesRegex(RuntimeError, 'faulted'):
            self.gate.request(self.command(operation='home', trial_id='', command_id='home'))

    def test_home_is_separate_command_after_completed_reach(self):
        self.gate.request(self.command(self.token()))
        self.workers.pop()()
        self.backend.prepare_home.assert_not_called()
        self.gate.request(self.command(operation='home', trial_id='', command_id='home'))
        self.workers.pop()()
        self.backend.prepare_home.assert_called_once()
        self.assertIsNone(self.gate.last_completed)

    def test_rejected_validation_does_not_create_ticket(self):
        self.backend.prepare.side_effect = ValueError('collision')
        with self.assertRaisesRegex(ValueError, 'collision'):
            self.token()
        self.assertFalse(self.gate.tickets)
        self.backend.perform.assert_not_called()


class StateTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.wall = 1.
        self.arm = ArmState(['j1', 'j0'], lambda: self.now, lambda: self.wall)

    def message(self, stamp=100., names=None, position=None, velocity=None):
        return NS(header=NS(stamp=Duration(stamp)), name=names or ['gripper', 'j0', 'j1'],
                  position=position if position is not None else [1, .1, .2],
                  velocity=velocity if velocity is not None else [9., 0., 0.])

    def test_orders_by_joint_name_and_excludes_gripper_motion(self):
        self.arm.update(self.message())
        sample = self.arm.current()
        np.testing.assert_array_equal(sample.positions, [.2, .1])
        np.testing.assert_array_equal(sample.velocities, [0., 0.])

    def test_missing_velocity_nan_duplicate_name_future_and_stale_reject(self):
        for message in (self.message(velocity=[]), self.message(velocity=[0, np.nan, 0]),
                        self.message(names=['j0', 'j0', 'j1']), self.message(stamp=200), self.message(stamp=1)):
            self.arm.update(message)
            with self.assertRaises(RuntimeError):
                self.arm.current()

    def test_frozen_ros_time_does_not_keep_state_fresh(self):
        self.arm.update(self.message())
        self.wall += .6
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            self.arm.current()

    def test_backward_stamp_invalidates_last_good_state(self):
        self.arm.update(self.message())
        self.arm.update(self.message(stamp=99.9))
        with self.assertRaisesRegex(RuntimeError, 'timestamp'):
            self.arm.current()


class BackendTests(unittest.TestCase):
    def setUp(self):
        self.backend = Backend.__new__(Backend)
        b = self.backend
        self.wall, self.ros_now = 1., 100.
        self.samples = [Sample(1, 100., 1., np.array([0.]), np.array([0.])),
                        Sample(2, 100.02, 1.02, np.array([.01]), np.array([.02])),
                        Sample(3, 100.04, 1.04, np.array([.04]), np.array([0.])),
                        Sample(4, 100.15, 1.15, np.array([.04]), np.array([0.]))]
        self.index = 0
        def after(seq):
            self.index = min(self.index + 1, len(self.samples)-1)
            return [self.samples[self.index]]
        b.arm = NS(current=lambda: self.samples[self.index], after=after)
        b.enabled = True
        b.settings = dict(commissioned_model_sha256='model', controller_interpolation_verified=True)
        b.model_digest = Mock(return_value='model')
        b.scene_context = Mock(return_value=('context', None, {}))
        b.measurements = Mock(return_value=self.samples[0])
        b.check_scaling, b.check_nonarm, b.publish = Mock(), Mock(), Mock()
        b.rt = NS(cfg=dict(safety=dict(start_tolerance_deg=.6)))
        b.ros = NS(Time=NS(now=lambda: Duration(self.ros_now)), Duration=Duration, is_shutdown=lambda: False)
        class RosTime(Duration):
            @classmethod
            def now(cls):
                return Duration(self.ros_now)
        b.ros.Time = RosTime
        b.control = NS(FollowJointTrajectoryGoal=lambda: NS(path_tolerance=[], goal_tolerance=[]),
                       JointTolerance=lambda **kw: NS(**kw))
        b.client = Mock()
        b.client.wait_for_server.return_value = True
        b.client.get_state.return_value = 3
        b.client.get_result.return_value = NS(error_code=0)
        b.expected_layout = b.layout = 'layout'
        b.scene_revision = 0
        self.command = NS(operation='run', dispatch_deadline=Duration(107))
        self.artifact = dict(path=path(), context='context', nonarm={}, duration=2.)

    def perform(self):
        def sleep(_):
            self.wall += .1
        with patch('legibility_execution.ros_node.time.monotonic', side_effect=lambda: self.wall), \
             patch('legibility_execution.ros_node.time.sleep', side_effect=sleep):
            self.backend.perform(self.command, self.artifact, 8.)

    def test_measured_onset_and_completion_exact_stored_path_no_automatic_home(self):
        self.perform()
        self.backend.client.send_goal.assert_called_once()
        goal = self.backend.client.send_goal.call_args.args[0]
        self.assertEqual([p.positions for p in goal.trajectory.points], [[0.], [.02], [.04]])
        events = self.backend.publish.call_args_list
        self.assertEqual([event.args[1] for event in events], ['executing', 'done'])
        self.assertEqual([event.args[2] for event in events], [100.02, 100.04])
        self.backend.client.cancel_goal.assert_not_called()

    def test_controller_failure_requests_cancellation_never_done(self):
        self.backend.client.get_state.return_value = 4
        with self.assertRaisesRegex(RuntimeError, 'failed'):
            self.perform()
        self.backend.client.cancel_goal.assert_called_once()
        self.assertNotIn('done', [c.args[1] for c in self.backend.publish.call_args_list])

    def test_absent_onset_cannot_be_reported_as_executing_even_if_action_succeeds(self):
        for sample in self.samples:
            sample.velocities[:] = 0
        with self.assertRaisesRegex(RuntimeError, 'onset'):
            self.perform()
        self.backend.publish.assert_not_called()
        self.backend.client.cancel_goal.assert_called_once()

    def test_disabled_uncommissioned_changed_scene_and_expired_command_send_nothing(self):
        mutations = [lambda b: setattr(b, 'enabled', False),
                     lambda b: b.settings.update(commissioned_model_sha256=''),
                     lambda b: b.settings.update(controller_interpolation_verified=False),
                     lambda b: setattr(b.scene_context, 'return_value', ('changed', None, {})),
                     lambda b: setattr(self.command, 'dispatch_deadline', Duration(99))]
        for mutation in mutations:
            self.setUp()
            mutation(self.backend)
            with self.assertRaises(RuntimeError):
                self.perform()
            self.backend.client.send_goal.assert_not_called()

    def test_scene_change_during_motion_cancels(self):
        def changed(seq):
            self.backend.scene_revision += 1
            return []
        self.backend.arm.after = changed
        with self.assertRaisesRegex(RuntimeError, 'scene update'):
            self.perform()
        self.backend.client.cancel_goal.assert_called_once()

    def test_action_success_without_measured_endpoint_never_done(self):
        for sample in self.samples[2:]:
            sample.positions[:] = .02
        with self.assertRaisesRegex(RuntimeError, 'completion timed out'):
            self.perform()
        self.assertEqual([c.args[1] for c in self.backend.publish.call_args_list], ['executing'])

    def test_read_only_service_failure_and_timeout_propagate(self):
        with self.assertRaisesRegex(ValueError, 'unavailable'):
            bounded(Mock(side_effect=ValueError('unavailable')))
        event = threading.Event()
        try:
            with self.assertRaisesRegex(RuntimeError, 'timed out'):
                bounded(lambda: event.wait(1.), timeout=.01)
        finally:
            event.set()

    def test_command_deadline_is_bounded_and_required(self):
        with patch('legibility_execution.ros_node.time.monotonic', return_value=1.):
            self.assertEqual(self.backend.dispatch_deadline(self.command), 8.)
            for deadline in (0, 99, float('nan'), 9999):
                self.command.dispatch_deadline = Duration(deadline)
                with self.assertRaises(ValueError):
                    self.backend.dispatch_deadline(self.command)


class InstallationTests(unittest.TestCase):
    def test_launch_defaults_disable_execution_and_never_start_legacy_motion(self):
        root = Path(__file__).resolve().parents[2]
        for filename in ('group_pipeline.launch', 'execution.launch'):
            tree = ET.parse(root / 'launch' / filename).getroot()
            args = {a.attrib['name']: a.attrib.get('default') for a in tree.findall('arg')}
            self.assertEqual(args['enable_execution'], 'false')
            self.assertNotIn('motion_node.py', [n.attrib['type'] for n in tree.findall('.//node')])
        for file in (root / 'legibility_ros').glob('*.py'):
            for node in ast.walk(ast.parse(file.read_text())):
                if isinstance(node, ast.ImportFrom):
                    self.assertFalse((node.module or '').startswith('legibility_execution'))


def message(**fields):
    """ROS-like slots ensure the real context serializer is exercised."""
    value = type('Message', (), {'__slots__': tuple(fields)})()
    for key, item in fields.items():
        setattr(value, key, item)
    return value


class ContextTests(unittest.TestCase):
    def setUp(self):
        self.b = Backend.__new__(Backend)
        b = self.b
        self.cfg = dict(block_size=[.1, .2, .3], blocks=[dict(name='B1', position=[.2, 0, 0]),
                                                       dict(name='B2', position=[.4, 0, 0])],
                        motion=dict(start_joints={'j0': 0.}))
        def cube(name, x):
            return message(id=name, header=message(frame_id='base'),
                           primitives=[message(type=1, dimensions=[.1, .2, .3])],
                           primitive_poses=[message(position=message(x=x, y=0., z=0.),
                               orientation=message(x=0., y=0., z=0., w=1.))], meshes=[], planes=[])
        objects = [cube('cube_B1', .2), cube('cube_B2', .4), cube('table', 0.)]
        state = message(is_diff=False, joint_state=message(name=['j0', 'gripper'], position=[0., .7],
                        velocity=[], effort=[]), multi_dof_joint_state=message(joint_names=[]),
                        attached_collision_objects=[])
        self.scene = message(is_diff=False, robot_state=state, world=message(collision_objects=objects),
                             allowed_collision_matrix=message(entry_names=[], entry_values=[]),
                             link_padding=[], link_scale=[], fixed_frame_transforms=[])
        b.scene_service = Mock(return_value=NS(scene=self.scene))
        components = ['ROBOT_STATE', 'ROBOT_STATE_ATTACHED_OBJECTS', 'WORLD_OBJECT_NAMES',
                      'WORLD_OBJECT_GEOMETRY', 'OCTOMAP', 'TRANSFORMS', 'ALLOWED_COLLISION_MATRIX',
                      'LINK_PADDING_AND_SCALING']
        b.rt = NS(cfg=self.cfg, frame='arm_base_link', names=('j0',),
                  config_sha256='config', robot=NS(get_planning_frame=lambda: 'base'),
                  msg=NS(PlanningSceneComponents=NS(**{name: 2**i for i, name in enumerate(components)})),
                  srv=NS(GetPlanningSceneRequest=lambda: NS(components=NS(components=0))))
        b.ros = NS(Time=Duration, Duration=Duration)
        b.settings = dict(required_world_objects=['table'], controller_name='scaled_pos_joint_traj_controller')
        b.model_digest = Mock(return_value='model')
        b.check_controller = Mock(return_value={'name': 'controller'})
        b.tf = NS(lookup_transform=Mock(return_value=message(transform='identity')))
        b.PoseStamped = lambda **kw: NS(**kw)
        b.transform_pose = lambda pose, transform: pose
        b.layout = b.expected_layout = 'layout'

    def test_context_binds_geometry_model_and_tool_attachments(self):
        original, state, nonarm = self.b.scene_context()
        self.assertEqual(nonarm, {'gripper': .7})
        self.scene.world.collision_objects[-1].primitive_poses[0].position.z = .01
        self.assertNotEqual(self.b.scene_context()[0], original)
        self.scene.world.collision_objects[-1].primitive_poses[0].position.z = 0
        self.scene.robot_state.attached_collision_objects = [message(id='new-tool')]
        self.assertNotEqual(self.b.scene_context()[0], original)
        self.b.tf.lookup_transform.assert_any_call('arm_base_link', 'base', unittest.mock.ANY, unittest.mock.ANY)

    def test_changed_model_parameters_require_restart_of_loaded_stack(self):
        b = self.b
        del b.model_digest
        b.rt.description = '/robot_description'
        params = {'/robot_description': 'calibrated-urdf', '/robot_description_semantic': 'srdf',
                  '/robot_description_planning/joint_limits': {'j0': 1}, '/robot_description_kinematics': {}}
        b.ros.get_param = lambda key: params[key]
        b.model_at_start = b.model_digest()
        params['/robot_description'] = 'changed-urdf'
        with self.assertRaisesRegex(RuntimeError, 'changed after node startup'):
            b.model_digest()

    def test_absent_table_or_geometry_and_missing_cubes_reject(self):
        self.b.settings['required_world_objects'] = []
        with self.assertRaisesRegex(ValueError, 'table/environment'):
            self.b.scene_context()
        self.b.settings['required_world_objects'] = ['table']
        self.scene.world.collision_objects[-1].primitives = []
        with self.assertRaisesRegex(ValueError, 'no collision geometry'):
            self.b.scene_context()
        self.scene.world.collision_objects.pop(0)
        with self.assertRaisesRegex(ValueError, 'cubes'):
            self.b.scene_context()

    def test_cube_pose_dimensions_and_quaternion_are_verified(self):
        for mutate in (lambda s: setattr(s.world.collision_objects[0].primitive_poses[0].position, 'x', 9.),
                       lambda s: setattr(s.world.collision_objects[0].primitives[0], 'dimensions', [1, 2, 3]),
                       lambda s: setattr(s.world.collision_objects[0].primitive_poses[0].orientation, 'w', np.nan)):
            self.setUp()
            mutate(self.scene)
            with self.assertRaises(ValueError):
                self.b.scene_context()

    def test_partial_scene_and_invalid_live_layout_reject(self):
        self.scene.is_diff = True
        with self.assertRaisesRegex(ValueError, 'full planning'):
            self.b.scene_context()
        self.scene.is_diff = False
        self.b.layout = None
        with self.assertRaisesRegex(ValueError, 'live block layout'):
            self.b.scene_context()

    def test_controller_type_state_resources_and_action_are_checked(self):
        b = self.b
        del b.check_controller
        controller = NS(name='scaled_pos_joint_traj_controller', state='running',
                        type='position_controllers/ScaledJointTrajectoryController',
                        claimed_resources=[NS(resources=['j0'])])
        b.controller_service = Mock(return_value=NS(controller=[controller]))
        b.action = '/scaled_pos_joint_traj_controller/follow_joint_trajectory'
        self.assertEqual(b.check_controller()['joints'], ['j0'])
        for field, value in [('state', 'stopped'), ('type', 'another/controller'),
                             ('claimed_resources', [NS(resources=['gripper'])])]:
            previous = getattr(controller, field)
            setattr(controller, field, value)
            with self.assertRaises(RuntimeError):
                b.check_controller()
            setattr(controller, field, previous)
        b.action = '/other/action'
        with self.assertRaises(RuntimeError):
            b.check_controller()

    def test_home_rejects_unknown_start_or_changed_scene_and_reuses_only_completed_curve(self):
        b = self.b
        b.rt.cfg['safety'] = dict(start_tolerance_deg=.6)
        b.arm = NS(current=lambda: Sample(1, 100., 1., np.array([.04]), np.array([0.])))
        b.measurements = Mock()
        b.ros.Duration = NS(from_sec=Duration)
        b.scene_context = Mock(return_value=('context', self.scene.robot_state, {}))
        with self.assertRaisesRegex(ValueError, 'no completed reach'):
            b.prepare_home(None)
        last = dict(path=path(), nonarm={}, duration=2., context='wrong', detail='prior checks')
        with self.assertRaisesRegex(ValueError, 'scene/model changed'):
            b.prepare_home(last)
        last['context'] = 'context'
        result = b.prepare_home(last)
        self.assertEqual(result['duration'], 8.)
        np.testing.assert_array_equal(derivatives(result['path'])[1][[0, -1], 0], [.04, 0.])
        b.arm.current = lambda: Sample(1, 100., 1., np.array([0.]), np.array([0.]))
        self.assertIsNone(b.prepare_home(None))


if __name__ == '__main__':
    unittest.main()
