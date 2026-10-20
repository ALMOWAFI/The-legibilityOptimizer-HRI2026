"""ROS 1 planner/optimizer operations shared by services and topic adapters.

Numerical handoff mode stops before downstream retiming/validation. No execution.
"""
from dataclasses import asdict
import hashlib
import json
import sys
import threading
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from experimental.legibility_optimizer.optimizer_core import OptimizerConfig
from experimental.legibility_optimizer.ros1_adapter import extract_path
from experimental.legibility_optimizer.sampled_optimizer import optimize_sampled_path
from .contracts import (FK, loaded_limits, positive, read_experiment, retime_checked,
                        sampled_checks, state_with_positions, replace_path)
from .layout import snapshot_layout, validate_goals
from .pipeline import optimize_preview


def json_status(data):
    return json.dumps(data, sort_keys=True, allow_nan=False,
                      default=lambda v: v.item() if isinstance(v, np.generic) else str(v))


class Runtime:
    def __init__(self):
        # Local imports keep protocol tests independent of an installed ROS runtime.
        import rospy
        import moveit_commander
        from moveit_msgs import msg, srv
        from geometry_msgs.msg import Pose, Point
        from hri_legibility_preview import srv as preview_srv
        self.rospy, self.msg, self.srv = rospy, msg, srv
        self.Pose, self.Point, self.services = Pose, Point, preview_srv
        with open(rospy.get_param("~experiment_config"), encoding="utf-8") as source:
            self.cfg = yaml.safe_load(source)
        self.ids, self.goals = read_experiment(self.cfg)
        self.config_sha256 = hashlib.sha256(
            json_status(self.cfg).encode("utf-8")).hexdigest()
        self.frame = self.cfg["frame_id"]
        self.group_name = self.cfg["motion"]["planning_group"]
        self.link = self.cfg["motion"]["ee_link"]
        self.description = rospy.get_param("~robot_description", "/robot_description")
        self.model_name = ET.fromstring(rospy.get_param(self.description)).attrib["name"]
        self.moveit_ns = rospy.get_param("~moveit_ns", "/")
        self.robot = moveit_commander.RobotCommander(robot_description=self.description, ns=self.moveit_ns)
        if self.group_name not in self.robot.get_group_names():
            raise ValueError("configured planning group is absent from the loaded SRDF")
        self.group = moveit_commander.MoveGroupCommander(
            self.group_name, robot_description=self.description, ns=self.moveit_ns)
        self.names = tuple(self.group.get_active_joints())
        if not self.names or set(self.names) != set(self.cfg["motion"]["start_joints"]):
            raise ValueError("configured start joints must exactly match the active planning group")
        if self.link not in self.robot.get_link_names(group=self.group_name):
            raise ValueError("configured end-effector link is absent from the planning group")
        self.scale = positive(rospy.get_param("~retiming_scale", 0.2), "retiming_scale")
        if self.scale > 1:
            raise ValueError("retiming_scale must be <= 1")
        self.orientation_tolerance = positive(
            rospy.get_param("~orientation_tolerance_rad", 0.05), "orientation tolerance")
        self.max_fk_calls = int(rospy.get_param("~max_fk_calls", 30000))
        self.service_timeout = positive(rospy.get_param("~service_timeout", 10.0), "service timeout")
        self.fk_service = self.proxy("~fk_service", "/compute_fk", srv.GetPositionFK)
        self.validity_service = None  # Only the optional preview validator needs this service.
        self.lock = threading.Lock()

    def proxy(self, parameter, default, service_type):
        name = self.rospy.get_param(parameter, default)
        self.rospy.wait_for_service(name, timeout=self.service_timeout)
        return self.rospy.ServiceProxy(name, service_type)

    def context(self, seed, trajectory):
        names, q, _ = extract_path(trajectory)
        if set(names) != set(self.names):
            raise ValueError("trajectory joints must exactly match the configured group")
        model_state = self.robot.get_current_state()
        if not set(model_state.joint_state.name).issubset(seed.joint_state.name):
            raise ValueError("request omits joints from the full model state")
        state_with_positions(seed, names, q[0])  # Validate complete message first.
        seed_map = dict(zip(seed.joint_state.name, seed.joint_state.position))
        if not np.allclose([seed_map[n] for n in names], q[0], atol=1e-7, rtol=0):
            raise ValueError("trajectory first point disagrees with supplied start state")
        bounds, vmax, amax = loaded_limits(
            self.robot, names, self.rospy.get_param(self.description),
            self.rospy.get_param(self.description + "_planning/joint_limits", {}),
            self.cfg["safety"]["max_joint_speed_deg"])
        fk = FK(self.fk_service, self.srv.GetPositionFKRequest, seed, names,
                self.frame, self.link, self.max_fk_calls)

        def valid(q_value):
            if self.validity_service is None:
                self.validity_service = self.proxy("~validity_service", "/check_state_validity",
                                                   self.srv.GetStateValidity)
            request = self.srv.GetStateValidityRequest()
            request.robot_state = state_with_positions(seed, names, q_value)
            request.group_name = self.group_name
            return self.validity_service(request).valid

        def validate(path):
            return sampled_checks(path, fk, valid, bounds, vmax, amax,
                                  self.cfg["safety"], self.orientation_tolerance)
        return names, fk, bounds, validate

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

    def plan_baseline(self, request):
        ids, goals = self.request_goals(request)
        if not request.trial_id.strip() or request.target_id not in ids:
            raise ValueError("trial_id and a target_id from the selected layout are required")
        seed = state_with_positions(self.robot.get_current_state(), self.names,
                                   [self.cfg["motion"]["start_joints"][n] for n in self.names])
        fk = FK(self.fk_service, self.srv.GetPositionFKRequest, seed, self.names,
                self.frame, self.link, self.max_fk_calls)
        start_q = [self.cfg["motion"]["start_joints"][n] for n in self.names]
        xyz0, quaternion = fk.pose(start_q)
        goal = goals[ids.index(request.target_id)]

        def pose(xyz):
            p = self.Pose()
            p.position.x, p.position.y, p.position.z = map(float, xyz)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quaternion)
            return p

        if request.baseline_mode == "cartesian":
            service = self.proxy("~cartesian_service", "/compute_cartesian_path", self.srv.GetCartesianPath)
            duration = float(self.cfg["motion"]["duration"])
            n = max(3, int(round(duration * 100)) + 1)
            if n > 5001:
                raise ValueError("Cartesian preview is limited to 5001 nominal samples")
            times = np.linspace(0, duration, n)
            tau = times / duration
            progress = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
            query = self.srv.GetCartesianPathRequest()
            query.header.frame_id, query.start_state = self.frame, seed
            query.group_name, query.link_name = self.group_name, self.link
            query.waypoints = [pose(xyz0 + s * (goal - xyz0)) for s in progress[1:]]
            query.max_step, query.jump_threshold, query.avoid_collisions = 0.005, 0.0, True
            planned = service(query)
            if (planned.error_code.val != 1 or not np.isfinite(planned.fraction)
                    or not 1.0 - 1e-9 <= planned.fraction <= 1.0):
                raise ValueError("MoveIt did not produce the complete collision-checked Cartesian path")
            trajectory = planned.solution
            points = trajectory.joint_trajectory.points
            if len(points) != n:
                raise ValueError("Cartesian service changed the sample count; nominal timing cannot be assigned")
            for point, time in zip(points, times):
                point.time_from_start = self.rospy.Duration.from_sec(float(time))
                point.velocities, point.accelerations, point.effort = [], [], []
            _, values, _ = extract_path(trajectory)
            if np.any(np.abs(np.diff(values, axis=0)) > np.deg2rad(self.cfg["safety"]["max_joint_step_deg"])):
                raise ValueError("Cartesian IK joint step exceeds configured jump limit")
        elif request.baseline_mode == "rrtconnect":
            planner_id = self.rospy.get_param("~planner_id", "")
            configs = self.rospy.get_param("~planner_configs_param", "/move_group/planner_configs")
            planner_configs = self.rospy.get_param(configs)
            group_key = self.rospy.get_param("~group_planners_param",
                                            "/move_group/" + self.group_name + "/planner_configs")
            group_planners = self.rospy.get_param(group_key)
            eligible = [p for p in group_planners if planner_configs.get(p, {}).get("type") == "geometric::RRTConnect"]
            if not planner_id and len(eligible) == 1:
                planner_id = eligible[0]
            if planner_id not in eligible:
                raise ValueError("select a verified RRTConnect planner_id enabled for this group: " + str(eligible))
            self.group.set_planning_pipeline_id(self.rospy.get_param("~planning_pipeline_id", "ompl"))
            self.group.set_planner_id(planner_id)
            self.group.set_start_state(seed)
            self.group.set_pose_reference_frame(self.frame)
            self.group.set_end_effector_link(self.link)
            self.group.set_max_velocity_scaling_factor(self.scale)
            self.group.set_max_acceleration_scaling_factor(self.scale)
            self.group.set_planning_time(10.0)
            orientation = self.msg.OrientationConstraint()
            orientation.header.frame_id, orientation.link_name = self.frame, self.link
            orientation.orientation = pose(goal).orientation
            orientation.absolute_x_axis_tolerance = self.orientation_tolerance / 2
            orientation.absolute_y_axis_tolerance = self.orientation_tolerance / 2
            orientation.absolute_z_axis_tolerance = self.orientation_tolerance / 2
            orientation.weight = 1.0
            constraints = self.msg.Constraints()
            constraints.orientation_constraints = [orientation]
            try:
                self.group.set_path_constraints(constraints)
                self.group.set_pose_target(pose(goal), self.link)
                planned = self.group.plan()
                if not isinstance(planned, tuple) or len(planned) != 4:
                    raise RuntimeError("expected ROS 1 MoveIt plan() tuple API; verify installed version")
                success, trajectory, _, error = planned
                if not success or error.val != 1:
                    raise ValueError("RRTConnect planning failed")
            finally:
                self.group.clear_pose_targets()
                self.group.clear_path_constraints()
        else:
            raise ValueError("baseline_mode must be cartesian or rrtconnect")
        trajectory = retime_checked(self.group, seed, trajectory, self.scale)
        names, fk, bounds, validate = self.context(seed, trajectory)
        _, q, _ = extract_path(trajectory)
        if np.linalg.norm(fk(q[-1]) - goal) > 0.005:
            raise ValueError("planned endpoint misses the requested hover goal")
        if np.any(q < bounds[:, 0]) or np.any(q > bounds[:, 1]):
            raise ValueError("planned baseline violates loaded joint bounds")
        checks = (validate(trajectory) if request.preview_validation else
                  {"sample_checks_passed": False, "physical_validity": "unassessed", "execution_ready": False})
        return seed, trajectory, checks

    def request_goals(self, request):
        layout = getattr(request, "layout", None)
        if layout is not None and layout.layout_id:
            _, ids, goals = snapshot_layout(layout, self.frame, self.cfg["safety"])
            return ids, goals
        return self.ids, self.goals

    def handle_plan(self, request):
        response = self.services.PlanPreviewResponse()
        status = {"trial_id": request.trial_id, "target_id": request.target_id,
                  "baseline_mode": request.baseline_mode, "config_sha256": self.config_sha256,
                  "physical_validity": "unassessed", "execution_ready": False}
        with self.lock:
            try:
                seed, baseline, checks = self.plan_baseline(request)
                response.baseline, response.baseline_planned = baseline, True
                response.start_state = seed
                status["baseline_checks"] = checks
                self.display(seed, baseline)
                if request.optimize:
                    optimize = self.proxy("~optimizer_service", "/legibility_preview/optimizer/optimize",
                                          self.services.OptimizeLegibility)
                    query = self.services.OptimizeLegibilityRequest()
                    query.trial_id, query.baseline, query.start_state = request.trial_id, baseline, seed
                    query.goal_header.frame_id = self.frame
                    ids, goals = self.request_goals(request)
                    query.goals = [self.Point(*map(float, xyz)) for xyz in goals]
                    query.intended_goal_index = ids.index(request.target_id)
                    query.preview_validation = request.preview_validation
                    result = optimize(query)
                    optimizer_status = json.loads(result.diagnostics)
                    status["optimizer"] = optimizer_status
                    if (optimizer_status.get("trial_id") != request.trial_id or
                            optimizer_status.get("config_sha256") != self.config_sha256):
                        raise ValueError("optimizer response trial/config does not match the planner")
                    response.numerical_accepted = result.numerical_accepted
                    if result.execution_ready:
                        raise ValueError("unexpected optimizer execution-ready flag")
                    if result.numerical_accepted and (not request.preview_validation or result.sample_checks_passed):
                        extract_path(result.candidate)
                        response.candidate = result.candidate
                        response.sample_checks_passed = result.sample_checks_passed
                        self.display(seed, result.candidate)
            except Exception as error:
                status["error"] = str(error)
            response.execution_ready = False
            response.diagnostics = json_status(status)
        return response

    def display(self, seed, trajectory):
        msg = self.msg.DisplayTrajectory()
        msg.model_id, msg.trajectory_start = self.model_name, seed
        msg.trajectory = [trajectory]
        self.preview.publish(msg)


def main(role):
    import rospy
    import moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node(role)
    try:
        runtime = Runtime()
        from .topics import TopicPlanner, TopicOptimizer
        if role == "optimizer":
            topics = TopicOptimizer(runtime)
            service = rospy.Service("~optimize", runtime.services.OptimizeLegibility, runtime.handle_optimize)
        elif role == "planner":
            runtime.preview = rospy.Publisher("~display", runtime.msg.DisplayTrajectory,
                                              queue_size=1, latch=True)
            topics = TopicPlanner(runtime)
            service = rospy.Service("~plan", runtime.services.PlanPreview, runtime.handle_plan)
        else:
            raise ValueError("unknown node role")
        rospy.loginfo("%s ready for preview requests only; config %s", role, runtime.config_sha256)
        rospy.spin()
    finally:
        moveit_commander.roscpp_shutdown()
