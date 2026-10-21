#!/usr/bin/env python3
"""Planner node: block layout + trial -> MoveIt RRT-Connect -> baseline trajectory.

Read plan_baseline() for planning and TopicPlanner for subscriptions/publication.
This node also publishes an RViz preview; it never executes a trajectory.
"""
from copy import deepcopy
import json
import sys

import numpy as np

from experimental.legibility_optimizer.ros1_adapter import extract_path
from legibility_ros.contracts import FK, retime_checked, state_with_positions
from legibility_ros.layout import snapshot_layout
from legibility_ros.runtime import Runtime, json_status
from legibility_ros.topics import TopicWorker


class PlannerNode(Runtime):
    """Plan against the loaded robot model using the configured study start."""

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


class TopicPlanner(TopicWorker):
    def __init__(self, runtime, messages=None, string_type=None, start_job=None, subscribe=True):
        super().__init__(runtime, messages, string_type, start_job)
        self.layout = None
        self.baseline = self.ros.Publisher('/legibility/baseline', self.messages.BaselinePlan,
                                           queue_size=10, latch=False)
        self.subscribers = []
        if subscribe:
            self.subscribers.append(self.ros.Subscriber('/hri/block_layout', self.messages.BlockLayout,
                                                       self.on_layout, queue_size=1))
            self.subscribers.append(self.ros.Subscriber('/legibility/trial_request', self.messages.TrialRequest,
                                                       self.on_trial, queue_size=10))
            if self.ros.get_param('~subscribe_group_trials', False):
                # Optional dependency: the group's existing message, never a copied definition.
                from hri_legible_motion.msg import TrialInfo
                self.subscribers.append(self.ros.Subscriber('/trial_start', TrialInfo,
                                                           self.on_group_trial, queue_size=10))

    def on_layout(self, message):
        try:
            snapshot, _, _ = snapshot_layout(message, self.runtime.frame, self.runtime.cfg['safety'])
            with self._lock:
                self.layout = snapshot
            self.report('', snapshot.layout_id, 'layout_received', True, 'layout available; no planning triggered')
        except Exception as error:
            # Do not retain an old layout when a replacement failed validation.
            with self._lock:
                self.layout = None
            self.report('', getattr(message, 'layout_id', ''), 'layout_rejected', False, str(error))

    def on_group_trial(self, message):
        if message.bend != 0:
            self.report(str(message.trial_id), '', 'trial_rejected', False,
                        'TrialInfo.bend is unsupported; use optimizer cost/deviation parameters')
            return
        trial = self.messages.TrialRequest()
        trial.header = deepcopy(message.header)
        trial.trial_id, trial.target_id, trial.condition = str(message.trial_id), message.target_id, message.condition
        self.on_trial(trial)

    def on_trial(self, message):
        trial = deepcopy(message)
        try:
            if not trial.trial_id.strip() or trial.condition not in ('baseline', 'legible'):
                raise ValueError('nonempty trial_id and condition baseline or legible are required')
            if trial.header.frame_id and trial.header.frame_id != self.runtime.frame:
                raise ValueError('trial frame does not match the layout/FK frame')
            with self._lock:
                layout = deepcopy(self.layout)
            if layout is None:
                raise ValueError('no valid block layout received yet')
            if trial.layout_id and trial.layout_id != layout.layout_id:
                raise ValueError('requested layout_id is not the currently received layout')
            _, ids, _ = snapshot_layout(layout, self.runtime.frame, self.runtime.cfg['safety'])
            if trial.target_id not in ids:
                raise ValueError('target_id is absent from the block layout')
            self.submit(trial.trial_id, layout.layout_id, lambda: self.plan(trial, layout))
        except Exception as error:
            self.report(trial.trial_id, trial.layout_id, 'trial_rejected', False, str(error))

    def plan(self, trial, layout):
        request = self.runtime.services.PlanPreviewRequest()
        request.trial_id, request.target_id = trial.trial_id, trial.target_id
        request.layout = layout
        request.baseline_mode = self.ros.get_param('~baseline_mode', 'rrtconnect')
        request.optimize, request.preview_validation = False, False
        self.report(trial.trial_id, layout.layout_id, 'planning', True, request.baseline_mode)
        with self.runtime.lock:
            seed, trajectory, _ = self.runtime.plan_baseline(request)
        result = self.messages.BaselinePlan()
        result.header.stamp, result.header.frame_id = self.ros.Time.now(), self.runtime.frame
        result.trial_id, result.target_id, result.condition = trial.trial_id, trial.target_id, trial.condition
        result.layout = deepcopy(layout)
        result.planning_group, result.ee_link = self.runtime.group_name, self.runtime.link
        result.model_name, result.config_sha256 = self.runtime.model_name, self.runtime.config_sha256
        result.baseline_mode, result.start_state, result.trajectory = request.baseline_mode, seed, trajectory
        self.baseline.publish(result)
        self.runtime.display(seed, trajectory)
        self.report(trial.trial_id, layout.layout_id, 'baseline_published', True,
                    'baseline is a planning result; downstream physical validation is still required')


def main():
    import rospy
    import moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("planner")
    try:
        node = PlannerNode()
        node.preview = rospy.Publisher("~display", node.msg.DisplayTrajectory,
                                       queue_size=1, latch=True)
        topics = TopicPlanner(node)
        service = rospy.Service("~plan", node.services.PlanPreview, node.handle_plan)
        rospy.loginfo("planner ready for preview requests only; config %s", node.config_sha256)
        rospy.spin()
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
