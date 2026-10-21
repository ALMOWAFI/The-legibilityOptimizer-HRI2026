"""Opt-in ROS 1 validator/executor for a static, commissioned lab scene.

This module alone owns the controller client. Construction and validation never
send goals. Hardware use requires matching commissioning data, an enabled node,
and a separately operator-confirmed RequestMotion. No controller switching,
dashboard calls, automatic retry, direct home move or protective-stop recovery.
"""
from collections import OrderedDict
from copy import deepcopy
import hashlib
import io
import json
import queue
import threading
import time

import numpy as np
import yaml

from experimental.legibility_optimizer.ros1_adapter import extract_path
from experimental.legibility_optimizer.optimizer_core import joint_motion_cost, legibility_score
from legibility_ros.contracts import FK, loaded_limits, state_with_positions, positive
from legibility_ros.layout import layout_from_config, snapshot_layout
from legibility_ros.runtime import Runtime
from .gate import ExecutionGate
from .state import ArmState
from .trajectory import check_curve, fit_duration, reversed_path


def ros_data(value):
    """Stable model/geometry payload; header sequence and timestamps are not geometry."""
    if hasattr(value, '__slots__'):
        return {key: ros_data(getattr(value, key)) for key in value.__slots__ if key not in ('stamp', 'seq')}
    if isinstance(value, (list, tuple)):
        return [ros_data(v) for v in value]
    if isinstance(value, bytes):
        return value.hex()
    return value


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def wire_digest(message):
    stream = io.BytesIO()
    message.serialize(stream)
    return hashlib.sha256(stream.getvalue()).hexdigest()


def bounded(call, timeout=3.):
    result = queue.Queue(maxsize=1)
    def worker():
        try:
            result.put((True, call()))
        except Exception as error:
            result.put((False, error))
    threading.Thread(target=worker, daemon=True).start()
    try:
        ok, value = result.get(timeout=timeout)
    except queue.Empty:
        raise RuntimeError('read-only ROS service timed out')
    if not ok:
        raise value
    return value


class Backend:
    def __init__(self):
        import rospy
        import actionlib
        import tf2_ros
        from tf2_geometry_msgs import do_transform_pose
        from control_msgs import msg as control
        from controller_manager_msgs.srv import ListControllers
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64
        from hri_legibility_preview import msg, srv

        self.rt = rt = Runtime()
        self.ros, self.msg, self.srv, self.control = rospy, msg, srv, control
        with open(rospy.get_param('~execution_config'), encoding='utf-8') as stream:
            self.settings = yaml.safe_load(stream)
        self.enabled = rospy.get_param('~enable_execution', False) is True
        self.model_at_start = self.model_digest()
        self.expected_layout = layout_from_config(rt.cfg, msg).layout_id
        self.layout = None
        self.proposals = OrderedDict()
        self.data_lock = threading.Lock()
        self.scaling = None
        self.scene_revision = 0
        self.joints = None
        self.arm = ArmState(rt.names, lambda: rospy.Time.now().to_sec())
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf)
        self.transform_pose, self.PoseStamped = do_transform_pose, PoseStamped
        self.scene_service = rt.proxy('~scene_service', '/get_planning_scene', rt.srv.GetPlanningScene)
        self.validity_service = rt.proxy('~validity_service', '/check_state_validity', rt.srv.GetStateValidity)
        self.controller_service = rospy.ServiceProxy(self.settings['controller_manager_service'], ListControllers)
        self.action = rt.cfg['motion']['action']
        self.client = actionlib.SimpleActionClient(self.action, control.FollowJointTrajectoryAction)
        self.feedback = rospy.Publisher('/legibility/motion_feedback', msg.MotionFeedback, queue_size=30)
        self.validated = rospy.Publisher('/legibility/validated_trajectory', msg.ValidatedTrajectory, queue_size=2)
        self.subscribers = [
            rospy.Subscriber('/joint_states', JointState, self.on_joints, queue_size=100),
            rospy.Subscriber(self.settings['speed_scaling_topic'], Float64, self.on_scaling, queue_size=10),
            rospy.Subscriber('/hri/block_layout', msg.BlockLayout, self.on_layout, queue_size=1),
            rospy.Subscriber('/legibility/trajectory_proposal', msg.TrajectoryProposal, self.on_proposal, queue_size=10),
            rospy.Subscriber(self.settings['scene_topic'], rt.msg.PlanningScene, self.on_scene, queue_size=20)]
        self.gate = ExecutionGate(self, rt.config_sha256, self.enabled, float(self.settings['validation_ttl']))
        self.services = [rospy.Service('/legibility/validate_trajectory', srv.ValidateTrajectory, self.validate_service),
                         rospy.Service('/legibility/request_motion', srv.RequestMotion, self.motion_service)]
        rospy.on_shutdown(self.cancel)
        rospy.loginfo('Downstream services ready; execution enabled=%s; no motion requested', self.enabled)

    def model_digest(self):
        r = self.rt
        current = digest(dict(urdf=self.ros.get_param(r.description),
                           srdf=self.ros.get_param(r.description + '_semantic'),
                           limits=self.ros.get_param(r.description + '_planning/joint_limits'),
                           kinematics=self.ros.get_param(r.description + '_kinematics')))
        if getattr(self, 'model_at_start', current) != current:
            raise RuntimeError('model parameters changed after node startup; restart the loaded MoveIt stack')
        return current

    def on_joints(self, message):
        self.arm.update(message)
        with self.data_lock:
            self.joints = deepcopy(message)

    def on_scaling(self, message):
        self.scaling = (float(message.data), time.monotonic())

    def on_layout(self, message):
        try:
            layout, _, _ = snapshot_layout(message, self.rt.frame, self.rt.cfg['safety'])
            value = layout.layout_id if layout.layout_id == self.expected_layout else None
        except Exception:
            value = None
        with self.data_lock:
            self.layout = value

    def on_proposal(self, message):
        with self.data_lock:
            self.proposals[message.source.trial_id] = wire_digest(message)
            while len(self.proposals) > 16:
                self.proposals.popitem(last=False)

    def on_scene(self, message):
        if (not message.is_diff or message.world.collision_objects or message.world.octomap.octomap.data
                or message.robot_state.attached_collision_objects or message.allowed_collision_matrix.entry_names
                or message.allowed_collision_matrix.default_entry_names or message.link_padding or message.link_scale
                or message.fixed_frame_transforms):
            with self.data_lock:
                self.scene_revision += 1  # Conservative: even repeated geometry updates invalidate an active motion.

    def check_scaling(self):
        item = self.scaling
        if item is None or time.monotonic() - item[1] > 0.5 or not np.isfinite(item[0]) or abs(item[0] - 1.) > 0.001:
            raise RuntimeError('fresh speed scaling of 1.0 is required for study timing')

    def check_controller(self):
        response = bounded(lambda: self.controller_service())
        expected = self.settings['controller_name']
        controller = next((c for c in response.controller if c.name == expected), None)
        if (controller is None or controller.state != 'running'
                or controller.type != 'position_controllers/ScaledJointTrajectoryController'
                or self.action.rstrip('/') != '/' + expected.strip('/') + '/follow_joint_trajectory'):
            raise RuntimeError('expected running UR scaled position trajectory controller/action not verified')
        claimed = {n for resource in controller.claimed_resources for n in resource.resources}
        if claimed != set(self.rt.names):
            raise RuntimeError('controller joint resources do not match the arm')
        return dict(name=controller.name, type=controller.type, joints=sorted(claimed))

    def scene_context(self):
        r = self.rt
        req = r.srv.GetPlanningSceneRequest()
        components = r.msg.PlanningSceneComponents
        req.components.components = (components.ROBOT_STATE | components.ROBOT_STATE_ATTACHED_OBJECTS |
            components.WORLD_OBJECT_NAMES | components.WORLD_OBJECT_GEOMETRY | components.OCTOMAP |
            components.TRANSFORMS | components.ALLOWED_COLLISION_MATRIX | components.LINK_PADDING_AND_SCALING)
        scene = bounded(lambda: self.scene_service(req)).scene
        if scene.is_diff or scene.robot_state.is_diff:
            raise ValueError('full planning scene/state required')
        objects = {obj.id: obj for obj in scene.world.collision_objects}
        required = self.settings['required_world_objects']
        cube_names = {'cube_' + b['name'] for b in r.cfg['blocks']}
        if (not isinstance(required, list) or not required or not all(isinstance(n, str) and n for n in required)
                or not set(required).difference(cube_names) or not (set(required) | cube_names).issubset(objects)):
            raise ValueError('configure existing table/environment collision object IDs; all cubes must also be present')
        for name in required:
            obj = objects[name]
            if not (obj.primitives or obj.meshes or obj.planes):
                raise ValueError('required environment object has no collision geometry: ' + name)
        # Compare cube geometry in the YAML frame, explicitly transforming instead of relabeling.
        for block in r.cfg['blocks']:
            obj = objects['cube_' + block['name']]
            if (len(obj.primitives) != 1 or obj.primitives[0].type != 1 or len(obj.primitive_poses) != 1
                    or obj.meshes or obj.planes or not np.allclose(obj.primitives[0].dimensions, r.cfg['block_size'], atol=1e-6, rtol=0)):
                raise ValueError('cube collision geometry differs from configured box')
            pose = self.PoseStamped(header=obj.header, pose=obj.primitive_poses[0])
            transform = self.tf.lookup_transform(r.frame, pose.header.frame_id, self.ros.Time(0), self.ros.Duration(0.5))
            pose = self.transform_pose(pose, transform).pose
            xyz = [pose.position.x, pose.position.y, pose.position.z]
            half = np.deg2rad(block.get('yaw', 0)) / 2
            quat = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            target = np.array([0, 0, np.sin(half), np.cos(half)])
            if (not np.isfinite(quat).all() or abs(np.linalg.norm(quat) - 1.) > 1e-6
                    or not np.allclose(xyz, block['position'], atol=0.002, rtol=0)
                    or abs(np.dot(quat, target)) < np.cos(0.01 / 2)):
                raise ValueError('scene cube pose differs from measured layout')
        frame_transform = self.tf.lookup_transform(r.robot.get_planning_frame(), r.frame,
                                                   self.ros.Time(0), self.ros.Duration(0.5))
        seed = scene.robot_state
        state_with_positions(seed, r.names, [r.cfg['motion']['start_joints'][n] for n in r.names])
        nonarm = {n: q for n, q in zip(seed.joint_state.name, seed.joint_state.position) if n not in r.names}
        # Round only stationary non-arm readings to suppress encoder noise; compare
        # these independently before and during motion as well.
        context = dict(world=ros_data(scene.world), attachments=ros_data(seed.attached_collision_objects),
                       multi_dof=ros_data(seed.multi_dof_joint_state), acm=ros_data(scene.allowed_collision_matrix),
                       padding=ros_data(scene.link_padding), scale=ros_data(scene.link_scale),
                       transforms=ros_data(scene.fixed_frame_transforms), goal_frame=ros_data(frame_transform),
                       model=self.model_digest(), controller=self.check_controller(), settings=self.settings,
                       config=r.config_sha256, layout=self.layout)
        if self.layout != self.expected_layout:
            raise ValueError('no matching live block layout')
        return digest(context), seed, nonarm

    def measurements(self, expected, nonarm=None):
        sample = self.arm.current()
        tolerance = np.deg2rad(positive(self.rt.cfg['safety']['start_tolerance_deg'], 'start tolerance'))
        if np.max(np.abs(sample.positions - expected)) > tolerance or np.max(np.abs(sample.velocities)) > 0.002:
            raise ValueError('arm must be stationary at the validated start configuration')
        self.check_scaling()
        self.check_nonarm(nonarm or {})
        return sample

    def check_nonarm(self, expected):
        with self.data_lock:
            message = self.joints
        current = dict(zip(message.name, message.position)) if message is not None else {}
        for name, value in expected.items():
            if name not in current or not np.isfinite(current[name]) or abs(current[name] - value) > 1e-4:
                raise ValueError('non-arm joint changed or is not measured: ' + name)

    def check_path(self, path, seed):
        r = self.rt
        names, _, _ = extract_path(path)
        if names != r.names:
            raise ValueError('executor requires the exact loaded arm joint order')
        bounds, vmax, amax = loaded_limits(r.robot, names, self.ros.get_param(r.description),
            self.ros.get_param(r.description + '_planning/joint_limits'), r.cfg['safety']['max_joint_speed_deg'])
        fk = FK(lambda req: bounded(lambda: r.fk_service(req)), r.srv.GetPositionFKRequest,
                seed, names, r.frame, r.link, r.max_fk_calls)
        def valid(q):
            request = r.srv.GetStateValidityRequest()
            request.robot_state = state_with_positions(seed, names, q)
            request.group_name = ''  # Empty group checks the whole robot, including tool and attachments.
            return bool(bounded(lambda: self.validity_service(request)).valid)
        def jacobian(q):
            # FK finite differences target the configured link/frame, avoiding the
            # MoveIt Python Jacobian's implicit final-group-link assumption.
            eps = 1e-5
            return np.column_stack([(fk(q + np.eye(len(names))[j] * eps) -
                                     fk(q - np.eye(len(names))[j] * eps)) / (2 * eps)
                                    for j in range(len(names))])
        report = check_curve(path, fk, valid, jacobian, bounds, vmax, amax, r.cfg['safety'],
                             r.orientation_tolerance, self.settings['max_sample_dt'],
                             self.settings['max_sample_joint_step'])
        report['tcp_velocity_method'] = 'central FK difference, epsilon=1e-5 rad'
        return report, fk

    def prepare(self, proposal, required_duration):
        r, source = self.rt, proposal.source
        # Topic and service transports can arrive in either order.
        deadline = time.monotonic() + 1.
        while True:
            with self.data_lock:
                observed = self.proposals.get(source.trial_id)
            if observed is not None or time.monotonic() >= deadline:
                break
            time.sleep(.01)
        if observed != wire_digest(proposal):
            raise ValueError('exact proposal has not been received on the proposal topic')
        if (source.model_name != r.model_name or source.config_sha256 != r.config_sha256
                or source.header.frame_id != r.frame or source.planning_group != r.group_name or source.ee_link != r.link
                or source.condition not in ('baseline', 'legible') or source.baseline_mode != 'rrtconnect'
                or source.layout.layout_id != self.expected_layout or not source.trial_id.strip()
                or proposal.optimization_applied != (source.condition == 'legible')
                or proposal.execution_ready or not proposal.retiming_required or not proposal.physical_validation_required):
            raise ValueError('proposal metadata or readiness flags are inconsistent')
        _, ids, goals = snapshot_layout(source.layout, r.frame, r.cfg['safety'])
        names, q, t = extract_path(proposal.trajectory)
        names0, q0, t0 = extract_path(source.trajectory)
        if (names != names0 or names != r.names or q.shape != q0.shape or not np.array_equal(t, t0)
                or not np.array_equal(q[[0, -1]], q0[[0, -1]]) or source.target_id not in ids
                or (source.condition == 'baseline' and not np.array_equal(q, q0))):
            raise ValueError('proposal joint order, endpoints, timing or baseline changed')
        home = [r.cfg['motion']['start_joints'][n] for n in names]
        if not np.allclose(q[0], home, atol=1e-7, rtol=0):
            raise ValueError('proposal does not begin at study start')
        if abs(positive(required_duration, 'required duration') - r.cfg['motion']['duration']) > 1e-6:
            raise ValueError('requested duration differs from experiment configuration')
        self.measurements(q[0])
        context, current, nonarm = self.scene_context()
        # Rebuild the seed from the current full scene so stale attachments cannot
        # disappear. The geometric arm path remains exactly as supplied.
        seed = state_with_positions(current, names, q[0])
        timed = fit_duration(r.group, seed, proposal.trajectory, required_duration, self.ros.Duration.from_sec)
        report, fk = self.check_path(timed, seed)
        if np.linalg.norm(fk(q[-1]) - goals[ids.index(source.target_id)]) > 0.005:
            raise ValueError('validated endpoint misses selected goal')
        baseline = fit_duration(r.group, seed, source.trajectory, required_duration, self.ros.Duration.from_sec)
        _, _, tb = extract_path(baseline)
        _, _, tc = extract_path(timed)
        cost0, cost = joint_motion_cost(q0, tb), joint_motion_cost(q, tc)
        beta = positive(self.settings['beta'], 'beta')
        score0 = legibility_score(q0, tb, goals, ids.index(source.target_id), fk, beta=beta)
        score = legibility_score(q, tc, goals, ids.index(source.target_id), fk, beta=beta)
        if (np.max(np.abs(q - q0)) > positive(self.settings['max_joint_deviation'], 'deviation') + 1e-8
                or cost > positive(self.settings['cost_ratio'], 'cost ratio') * cost0 + 1e-8
                or (source.condition == 'legible' and score < score0 + 1e-5)):
            raise ValueError('retimed knot metrics fail motion budget/deviation/legibility acceptance')
        report.update(retimed_baseline_cost=cost0, retimed_cost=cost,
                      retimed_baseline_legibility=score0, retimed_legibility=score)
        if self.scene_context()[0] != context:
            raise ValueError('model/scene/controller/layout changed during validation')
        self.measurements(q[0], nonarm)
        return dict(path=timed, seed=seed, context=context, nonarm=nonarm,
                    duration=float(required_duration), detail=json.dumps(report, sort_keys=True))

    def prepare_home(self, last):
        r = self.rt
        home = np.array([r.cfg['motion']['start_joints'][n] for n in r.names])
        current = self.arm.current()
        tolerance = np.deg2rad(r.cfg['safety']['start_tolerance_deg'])
        if np.max(np.abs(current.positions - home)) <= tolerance:
            self.measurements(home)
            return None  # Already home; publish measured verification without sending a goal.
        if last is None:
            raise ValueError('no completed reach to reverse; use the lab MoveIt workflow to reach study start')
        _, q, _ = extract_path(last['path'])
        self.measurements(q[-1], last['nonarm'])
        context, current, nonarm = self.scene_context()
        if context != last['context']:
            raise ValueError('scene/model changed since reach; return requires a new lab-approved plan')
        path = reversed_path(last['path'], max(8., last['duration']), self.ros.Duration.from_sec)
        seed = state_with_positions(current, r.names, q[-1])
        # Exact reversal of a P/V/A spline preserves its geometry. Uniform
        # slowdown reduces all |v| and |a|; existing curve checks therefore apply
        # in the identical scene. No unplanned direct return or extra waypoint.
        report = dict(return_basis='exact reversed validated spline in unchanged context',
                      original_validation=last['detail'], continuous_collision_certified=False)
        return dict(path=path, seed=seed, nonarm=nonarm, context=context,
                    duration=extract_path(path)[2][-1], detail=json.dumps(report))

    def dispatch_deadline(self, command):
        remaining = command.dispatch_deadline.to_sec() - self.ros.Time.now().to_sec()
        maximum = 7. if command.operation == 'run' else 25.
        if not np.isfinite(remaining) or not 0 < remaining <= maximum + 0.05:
            raise ValueError('missing, expired or excessive command dispatch deadline')
        return time.monotonic() + min(remaining, maximum)

    def check_deadline(self, command, deadline):
        if time.monotonic() >= deadline or self.ros.Time.now().to_sec() >= command.dispatch_deadline.to_sec():
            raise RuntimeError('command expired before dispatch; no controller goal sent')

    def publish(self, command, state, stamp, detail):
        message = self.msg.MotionFeedback()
        message.header.stamp = self.ros.Time.from_sec(stamp)
        message.command_id, message.trial_id = command.command_id, command.trial_id
        message.validation_id, message.state, message.detail = command.validation_id, state, detail
        self.feedback.publish(message)

    def cancel(self):
        if getattr(self, 'sent', False):
            self.client.cancel_goal()  # Only this node's goal; never cancel another client's commands.

    def failed(self, command, detail):
        self.publish(command, 'failed', self.ros.Time.now().to_sec(), detail)

    def perform(self, command, artifact, deadline):
        if not self.enabled:
            raise RuntimeError('hardware execution disabled')
        self.check_deadline(command, deadline)
        if artifact is None:
            self.publish(command, 'home', self.arm.current().stamp, 'already at study start; no controller goal')
            return
        if self.settings['commissioned_model_sha256'] != self.model_digest():
            raise RuntimeError('loaded calibrated model/limits have not been commissioned in execution config')
        if self.settings.get('controller_interpolation_verified') is not True:
            raise RuntimeError('installed controller interpolation has not been verified')
        revision = self.scene_revision
        if self.scene_context()[0] != artifact['context']:
            raise RuntimeError('validation context changed; prepare and validate again')
        names, q, _ = extract_path(artifact['path'])
        first = self.measurements(q[0], artifact['nonarm'])
        if not bounded(lambda: self.client.wait_for_server(self.ros.Duration(2.)), 3.):
            raise RuntimeError('controller action server unavailable')
        goal = self.control.FollowJointTrajectoryGoal()
        goal.trajectory = deepcopy(artifact['path'].joint_trajectory)
        goal.trajectory.header.stamp = self.ros.Time(0)
        # Controller checks tracking error; finite tolerances supplement our final
        # measured-state check. Startup splice/tracking behavior must be commissioned.
        for name in names:
            goal.path_tolerance.append(self.control.JointTolerance(name=name, position=np.deg2rad(1.0)))
            goal.goal_tolerance.append(self.control.JointTolerance(name=name,
                                       position=np.deg2rad(self.rt.cfg['safety']['start_tolerance_deg']), velocity=0.002))
        goal.goal_time_tolerance = self.ros.Duration(1.)
        onset = None
        sequence = first.sequence
        sent_at, sent_ros = time.monotonic(), self.ros.Time.now().to_sec()
        self.check_deadline(command, deadline)
        # Recheck fresh measurements after all potentially blocking preflight work.
        self.measurements(q[0], artifact['nonarm'])
        if self.scene_revision != revision or self.layout != self.expected_layout:
            raise RuntimeError('scene/layout changed during dispatch preflight')
        self.sent = True
        try:
            self.client.send_goal(goal)
            settled = None
            while not self.ros.is_shutdown():
                elapsed = time.monotonic() - sent_at
                if elapsed > artifact['duration'] + 3.:
                    raise RuntimeError('motion completion timed out')
                self.check_scaling()
                self.check_nonarm(artifact['nonarm'])
                if self.layout != self.expected_layout or self.scene_revision != revision:
                    raise RuntimeError('layout or planning-scene update during motion')
                for sample in self.arm.after(sequence):
                    sequence = sample.sequence
                    if sample.stamp < sent_ros:
                        continue
                    if onset is None and np.max(np.abs(sample.velocities)) > 0.01:
                        onset = sample.stamp
                        if command.operation == 'run':
                            self.publish(command, 'executing', onset, 'measured arm velocity onset')
                    at_goal = (np.max(np.abs(sample.positions - q[-1])) <= np.deg2rad(self.rt.cfg['safety']['start_tolerance_deg'])
                               and np.max(np.abs(sample.velocities)) <= 0.002)
                    settled = (settled or sample.stamp) if at_goal and onset is not None else None
                state = self.client.get_state()
                if state in (2, 4, 5, 8, 9):
                    raise RuntimeError('controller action failed/preempted/lost (state %d)' % state)
                if onset is None and elapsed > 2.:
                    raise RuntimeError('no measured arm onset; motion is not reported as executing')
                if state == 3:
                    result = self.client.get_result()
                    if result is None or result.error_code != 0:
                        raise RuntimeError('controller returned unsuccessful result')
                    if settled is not None and self.arm.current().stamp - settled >= 0.1:
                        self.publish(command, 'done' if command.operation == 'run' else 'home', settled,
                                     'controller success and measured endpoint settled for 0.1 s')
                        self.sent = False
                        return
                time.sleep(0.01)
            raise RuntimeError('ROS shutdown during motion')
        except Exception:
            self.cancel()
            # Cancellation is requested, not asserted successful. The gate latches
            # fault and cannot send another motion even if cancellation is delayed.
            raise

    def validate_service(self, request):
        response = self.srv.ValidateTrajectoryResponse()
        try:
            token, duration, detail = self.gate.validate(request.proposal, request.required_duration)
            response.accepted, response.validation_id, response.duration, response.detail = True, token, duration, detail
            with self.gate.lock:
                artifact = deepcopy(self.gate.tickets[token].artifact)
            message = self.msg.ValidatedTrajectory()
            message.header.stamp = self.ros.Time.now()
            message.trial_id, message.validation_id = request.proposal.source.trial_id, token
            message.context_sha256, message.trajectory, message.detail = artifact['context'], artifact['path'], detail
            self.validated.publish(message)
            self.rt.display(artifact['seed'], artifact['path'])
        except Exception as error:
            response.accepted, response.detail = False, str(error)
        return response

    def motion_service(self, request):
        response = self.srv.RequestMotionResponse()
        try:
            response.detail = self.gate.request(request)
            response.accepted = True
        except Exception as error:
            response.detail = str(error)
        return response


def main():
    import sys
    import rospy
    import moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('executor')
    try:
        Backend()
        rospy.spin()
    finally:
        moveit_commander.roscpp_shutdown()
