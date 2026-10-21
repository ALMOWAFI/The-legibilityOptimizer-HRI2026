"""Shared ROS model/configuration, FK and sampled-check context for the nodes.

Planning lives in scripts/planner_node.py; optimization in scripts/optimizer_node.py.
This module only provides their shared robot access and configuration helpers.
"""
import hashlib
import json
import threading
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from experimental.legibility_optimizer.ros1_adapter import extract_path
from .contracts import FK, loaded_limits, positive, read_experiment, sampled_checks, state_with_positions


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
