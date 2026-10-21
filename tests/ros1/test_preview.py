"""Real numeric solves + fake ROS message/service boundaries. No ROS installation."""
import ast
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import numpy as np

from experimental.legibility_optimizer.demo import synthetic_case, identity_fk
from experimental.legibility_optimizer.optimizer_core import OptimizerConfig
from experimental.legibility_optimizer.ros1_adapter import extract_path
from legibility_ros import contracts as c
from legibility_ros.pipeline import optimize_preview

ROOT = Path(__file__).resolve().parents[2]


class Duration:
    def __init__(self, seconds):
        self.seconds = float(seconds)

    def to_sec(self):
        return self.seconds


def trajectory(q, t, names=None):
    names = list(names or ["j" + str(i) for i in range(q.shape[1])])
    return NS(joint_trajectory=NS(joint_names=names, points=[
        NS(positions=list(v), velocities=[0.] * len(names), accelerations=[0.] * len(names),
           effort=[], time_from_start=Duration(s)) for v, s in zip(q, t)]),
        multi_dof_joint_trajectory=NS(joint_names=[], points=[]))


def seed_state():
    return NS(is_diff=False, joint_state=NS(name=["other", "j1", "j0", "j2"],
              position=[0.7, 0.0, 0.0, 0.0], velocity=[1, 2, 3, 4], effort=[1, 1, 1, 1]),
              attached_collision_objects=["tool"], multi_dof_joint_state=NS(marker="preserved"))


def experiment():
    return dict(frame_id="arm_base_link", motion=dict(planning_group="arm", ee_link="arm_tool0",
                 touch_z=0.003, hover_height=0.05, duration=4),
                blocks=[dict(id=3, position=[0.4, -0.2, -0.074]),
                        dict(id=8, position=[0.5, 0.2, -0.074])],
                safety=dict(workspace_min=[0.2, -0.8, 0.023], workspace_max=[0.95, 0.65, 0.5],
                            max_tcp_speed=0.5, max_joint_speed_deg=60, max_joint_step_deg=2))


def fk_request():
    return NS(header=NS(frame_id=""), fk_link_names=[], robot_state=None)


def fk_response():
    return NS(error_code=NS(val=1), fk_link_names=["arm_tool0"], pose_stamped=[
        NS(header=NS(frame_id="arm_base_link"), pose=NS(position=NS(x=0.4, y=0.1, z=0.2),
                                                        orientation=NS(x=0., y=0., z=0., w=1.)))])


class ContractTests(unittest.TestCase):
    def test_layout_uses_hover_goals_and_ids_without_relabeling_frame(self):
        cfg = experiment()
        ids, goals = c.read_experiment(cfg)
        self.assertEqual(ids, [3, 8])
        np.testing.assert_allclose(goals, [[0.4, -0.2, 0.053], [0.5, 0.2, 0.053]])
        self.assertEqual(cfg["frame_id"], "arm_base_link")
        self.assertEqual(cfg["blocks"][0]["position"][2], -0.074)

    def test_invalid_layout_fails(self):
        for change in (lambda x: x.update(frame_id=""),
                       lambda x: x["blocks"][1].update(id=3),
                       lambda x: x["motion"].update(hover_height=float("nan")),
                       lambda x: x["blocks"][0].update(position=[2, 0, 0])):
            cfg = experiment()
            change(cfg)
            with self.assertRaises(ValueError):
                c.read_experiment(cfg)

    def test_full_state_preserved_and_joint_order_is_name_based(self):
        seed = seed_state()
        state = c.state_with_positions(seed, ["j0", "j1"], [0.2, -0.4])
        self.assertEqual(state.joint_state.position, [0.7, -0.4, 0.2, 0.0])
        self.assertEqual(state.attached_collision_objects, ["tool"])
        self.assertEqual(state.multi_dof_joint_state.marker, "preserved")
        self.assertEqual(state.joint_state.velocity, [])
        self.assertEqual(seed.joint_state.position, [0.7, 0., 0., 0.])
        with self.assertRaises(ValueError):
            c.state_with_positions(seed, ["missing"], [0.2])
        seed.is_diff = True
        with self.assertRaises(ValueError):
            c.state_with_positions(seed, ["j0"], [0.2])

    def test_fk_checks_service_response_and_caches_exact_queries(self):
        service = Mock(return_value=fk_response())
        fk = c.FK(service, fk_request, seed_state(), ["j0", "j1"], "arm_base_link", "arm_tool0")
        first = fk([0.1, 0.2])
        first[:] = 99
        np.testing.assert_allclose(fk([0.1, 0.2]), [0.4, 0.1, 0.2])
        self.assertEqual(service.call_count, 1)
        fk([0.1 + 1e-8, 0.2])
        self.assertEqual(service.call_count, 2)
        req = service.call_args.args[0]
        self.assertEqual(req.header.frame_id, "arm_base_link")
        self.assertEqual(req.robot_state.attached_collision_objects, ["tool"])
        self.assertAlmostEqual(req.robot_state.joint_state.position[2], 0.1 + 1e-8)

    def test_fk_rejects_failure_wrong_frame_wrong_link_and_bad_pose(self):
        mutations = [lambda r: setattr(r.error_code, "val", -1),
                     lambda r: setattr(r.pose_stamped[0].header, "frame_id", "base"),
                     lambda r: setattr(r, "fk_link_names", ["wrong"]),
                     lambda r: setattr(r.pose_stamped[0].pose.position, "x", np.nan),
                     lambda r: setattr(r.pose_stamped[0].pose.orientation, "w", 0)]
        for mutation in mutations:
            response = fk_response()
            mutation(response)
            fk = c.FK(lambda req: response, fk_request, seed_state(), ["j0"], "arm_base_link", "arm_tool0")
            with self.assertRaises((ValueError, RuntimeError)):
                fk([0.1])

    def test_fk_budget_and_service_failure_are_not_success(self):
        fk = c.FK(lambda r: fk_response(), fk_request, seed_state(), ["j0"],
                  "arm_base_link", "arm_tool0", max_calls=1)
        fk([0])
        with self.assertRaisesRegex(RuntimeError, "budget"):
            fk([1])
        service = Mock(side_effect=RuntimeError("service unavailable"))
        fk = c.FK(service, fk_request, seed_state(), ["j0"], "arm_base_link", "arm_tool0")
        with self.assertRaisesRegex(RuntimeError, "unavailable"):
            fk([0])

    def test_loaded_limits_use_actual_urdf_and_tighter_limits(self):
        robot = NS(get_joint=lambda name: NS(bounds=lambda: [-2, 2]))
        urdf = '<robot name="cu_ur5e"><joint name="j0" type="revolute"><limit velocity="2"/></joint></robot>'
        overrides = {"j0": dict(has_acceleration_limits=True, max_acceleration=6.283,
                                has_velocity_limits=True, max_velocity=0.7)}
        bounds, velocity, acceleration = c.loaded_limits(robot, ["j0"], urdf, overrides, 60)
        np.testing.assert_array_equal(bounds, [[-2, 2]])
        np.testing.assert_allclose(velocity, [0.7])
        np.testing.assert_allclose(acceleration, [6.283])
        with self.assertRaisesRegex(ValueError, "acceleration"):
            c.loaded_limits(robot, ["j0"], urdf, {}, 60)
        with self.assertRaises((KeyError, AttributeError)):
            c.loaded_limits(robot, ["j0"], urdf.replace('velocity="2"', ''), overrides, 60)

    def test_retime_drops_stale_derivatives_and_rejects_geometry_changes(self):
        q, t, _ = synthetic_case()
        path = trajectory(q, t)
        copied = c.replace_path(path, q, t, Duration)
        self.assertEqual(copied.joint_trajectory.points[0].velocities, [])
        self.assertEqual(path.joint_trajectory.points[0].velocities, [0.] * 3)
        def change(state, msg, **kwargs):
            msg.joint_trajectory.points[-1].positions[0] += 0.001
            return msg
        with self.assertRaisesRegex(ValueError, "changed"):
            c.retime_checked(NS(retime_trajectory=change), seed_state(), path, 0.2)
        def invalid(state, msg, **kwargs):
            msg.joint_trajectory.points[1].time_from_start = Duration(0)
            return msg
        with self.assertRaisesRegex(ValueError, "increasing"):
            c.retime_checked(NS(retime_trajectory=invalid), seed_state(), path, 0.2)


class IdentityFK:
    def __call__(self, q):
        return np.array(q)

    def pose(self, q):
        return self(q), np.array([0., 0., 0., 1.])


class ValidationAndPipelineTests(unittest.TestCase):
    def setUp(self):
        self.q, self.t, self.goals = synthetic_case()
        self.path = trajectory(self.q, self.t)
        self.fk = IdentityFK()
        self.safety = dict(workspace_min=[-2, -2, -2], workspace_max=[2, 2, 2], max_tcp_speed=2)

    def validate(self, path, **kwargs):
        settings = dict(fk=self.fk, state_valid=lambda q: True,
                        bounds=np.array([[-2, 2]] * 3), vmax=np.array([2.] * 3),
                        amax=np.array([10.] * 3), safety=self.safety, orientation_tolerance=0.05)
        settings.update(kwargs)
        return c.sampled_checks(path, **settings)

    def test_successful_sample_checks_still_not_physical_validity(self):
        status = self.validate(self.path)
        self.assertTrue(status["sample_checks_passed"])
        self.assertGreater(status["checked_states"], len(self.q))
        self.assertEqual(status["physical_validity"], "unassessed")
        self.assertFalse(status["execution_ready"])

    def test_invalid_interior_state_rejects_even_when_knots_are_valid(self):
        def validity(q):
            return not (0.04 < q[0] < 0.06)
        with self.assertRaisesRegex(ValueError, "state validity"):
            self.validate(self.path, state_valid=validity)

    def test_orientation_drift_and_workspace_reject(self):
        class RotatingFK(IdentityFK):
            def pose(self, q):
                return self(q), np.array([0., 0., np.sin(q[0]), np.cos(q[0])])
        with self.assertRaisesRegex(ValueError, "orientation"):
            self.validate(self.path, fk=RotatingFK())
        with self.assertRaisesRegex(ValueError, "workspace"):
            self.validate(self.path, safety=dict(self.safety, workspace_max=[0.5, 2, 2]))

    def test_missing_derivatives_velocity_acceleration_and_tcp_reject(self):
        path = deepcopy(self.path)
        path.joint_trajectory.points[2].accelerations = []
        with self.assertRaises(ValueError):
            self.validate(path)
        for values in (dict(vmax=np.array([0.01] * 3)),
                       dict(safety=dict(self.safety, max_tcp_speed=0.01))):
            with self.assertRaises(ValueError):
                self.validate(self.path, **values)
        path = deepcopy(self.path)
        path.joint_trajectory.points[2].accelerations = [100, 0, 0]
        with self.assertRaisesRegex(ValueError, "accelerations"):
            self.validate(path)

    def run_pipeline(self, group, validate=None):
        return optimize_preview(self.path, seed_state(), self.goals, 0, self.fk, group,
                                validate or self.validate, Duration,
                                config=OptimizerConfig(cost_ratio=1.45, maxiter=200),
                                support_points=7, scale=0.2,
                                constraints=dict(joint_lower=[-2] * 3, joint_upper=[2] * 3,
                                                 max_joint_deviation=[0.25, 0.25, 0.1]))

    def test_real_solve_retime_and_checks_preserve_endpoints_without_execution(self):
        def retime(seed, path, **kwargs):
            for p in path.joint_trajectory.points:
                p.time_from_start = Duration(p.time_from_start.to_sec() * 2)
                p.velocities, p.accelerations = [0.] * 3, [0.] * 3
            return path
        group = NS(retime_trajectory=Mock(side_effect=retime))
        candidate, status = self.run_pipeline(group)
        self.assertIsNotNone(candidate, status)
        self.assertTrue(status["numerical_accepted"])
        self.assertTrue(status["sample_checks_passed"])
        self.assertFalse(status["execution_ready"])
        self.assertEqual(status["physical_validity"], "unassessed")
        _, q, t = extract_path(candidate)
        np.testing.assert_array_equal(q[[0, -1]], self.q[[0, -1]])
        np.testing.assert_allclose(t, self.t * 2)
        self.assertEqual(group.retime_trajectory.call_count, 1)

    def test_retiming_failure_does_not_erase_numerical_success(self):
        group = NS(retime_trajectory=Mock(side_effect=RuntimeError("retimer unavailable")))
        candidate, status = self.run_pipeline(group)
        self.assertIsNone(candidate)
        self.assertTrue(status["numerical_accepted"])
        self.assertFalse(status["sample_checks_passed"])
        self.assertFalse(status["execution_ready"])
        self.assertEqual(status["stage"], "retiming")

    def test_failed_baseline_check_never_runs_optimizer_or_retimer(self):
        group = NS(retime_trajectory=Mock())
        with patch("legibility_ros.pipeline.optimize_sampled_path") as solver:
            candidate, status = self.run_pipeline(group, Mock(side_effect=RuntimeError("collision")))
            solver.assert_not_called()
        group.retime_trajectory.assert_not_called()
        self.assertIsNone(candidate)
        self.assertFalse(status["numerical_accepted"])
        self.assertIn("collision", status["error"])


class BoundaryTests(unittest.TestCase):
    def test_runtime_has_no_controller_clients_and_only_declared_publishers(self):
        forbidden = {"execute", "go", "move", "send_goal", "SimpleActionClient", "pick", "place"}
        for path in list((ROOT / "legibility_ros").glob("*.py")) + list((ROOT / "scripts").glob("*.py")):
            # Only these entrypoints may import the separate execution package.
            # The planner/optimizer packages retain all controller-call guards.
            downstream_entry = path.name in {'executor_node.py', 'inspect_execution.py'}
            tree = ast.parse(path.read_text(encoding="utf-8"))
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    self.assertFalse({n.name.split('.')[0] for n in node.names} & {"actionlib", "control_msgs"})
                if isinstance(node, ast.ImportFrom):
                    self.assertNotIn((node.module or '').split('.')[0], {"actionlib", "control_msgs"})
                    if not downstream_entry:
                        self.assertNotEqual((node.module or '').split('.')[0], 'legibility_execution')
                if isinstance(node, ast.Call):
                    call = getattr(node.func, "attr", getattr(node.func, "id", ""))
                    self.assertNotIn(call, forbidden, str(path))
                    if call == "Publisher":
                        permitted = {'~display': 'DisplayTrajectory', '/hri/block_layout': 'BlockLayout',
                                     '/legibility/trial_request': 'TrialRequest', '/legibility/baseline': 'BaselinePlan',
                                     '/legibility/optimized_candidate': 'OptimizedCandidate', '/legibility/status': 'string_type',
                                     '/legibility/trajectory_proposal': 'TrajectoryProposal'}
                        topic = node.args[0].value
                        self.assertIn(topic, permitted)
                        kind = getattr(node.args[1], 'attr', getattr(node.args[1], 'id', ''))
                        self.assertEqual(kind, permitted[topic])

    def test_package_launch_and_service_contracts(self):
        package = ET.parse(ROOT / "package.xml").getroot()
        self.assertEqual(package.findtext("name"), "hri_legibility_preview")
        launch = ET.parse(ROOT / "launch/preview.launch").getroot()
        nodes = launch.findall(".//node")
        self.assertEqual({n.attrib['type'] for n in nodes}, {"planner_node.py", "optimizer_node.py"})
        for filename in ("PlanPreview.srv", "OptimizeLegibility.srv"):
            source = (ROOT / "srv" / filename).read_text()
            self.assertEqual(source.count("---"), 1)
            self.assertIn("bool execution_ready", source)


if __name__ == "__main__":
    unittest.main()
