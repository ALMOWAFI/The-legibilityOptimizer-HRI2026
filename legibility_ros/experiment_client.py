"""Optional group-session adapter. No controller client or safety implementation.

Preparation uses the ordinary planner/optimizer topics. Validation and explicitly
requested motion use the separate downstream services; absence is an error.
ROS dependencies are injected in tests and imported only at construction.
"""
from copy import deepcopy
import hashlib
import json
import math
import queue
import threading
import time
import uuid

import numpy as np

from experimental.legibility_optimizer.ros1_adapter import extract_path
from .layout import layout_from_config, snapshot_layout


class ExperimentClient:
    def __init__(self, ros, cfg, messages=None, services=None, string_type=None, timeout=180.0):
        if messages is None:
            from hri_legibility_preview import msg as messages
        if services is None:
            from hri_legibility_preview import srv as services
        if string_type is None:
            from std_msgs.msg import String as string_type
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("preparation timeout must be positive")
        self.ros, self.cfg, self.msg, self.srv = ros, deepcopy(cfg), messages, services
        self.timeout = timeout
        self.session_id = uuid.uuid4().hex
        self.config_sha256 = hashlib.sha256(json.dumps(
            cfg, sort_keys=True, allow_nan=False).encode()).hexdigest()
        self.expected_layout = layout_from_config(cfg, messages)
        self.lock = threading.RLock()
        self.layout = self.pending = self.proposal = self.error = None
        self.validation_id = None
        self.active_command = None
        self.feedback_state = None
        self.feedback_time = None
        self.motion_uncertain = False
        self.feedback = queue.Queue()
        self.used_validations = set()
        self.trials = ros.Publisher('/legibility/trial_request', messages.TrialRequest,
                                    queue_size=1, latch=False)
        self.subscribers = [
            ros.Subscriber('/hri/block_layout', messages.BlockLayout, self.on_layout, queue_size=1),
            ros.Subscriber('/legibility/trajectory_proposal', messages.TrajectoryProposal,
                           self.on_proposal, queue_size=10),
            ros.Subscriber('/legibility/status', string_type, self.on_status, queue_size=20),
            ros.Subscriber('/legibility/motion_feedback', messages.MotionFeedback,
                           self.on_feedback, queue_size=30)]

    def on_layout(self, message):
        with self.lock:
            try:
                layout, _, _ = snapshot_layout(message, self.cfg['frame_id'], self.cfg['safety'])
                if layout.layout_id != self.expected_layout.layout_id:
                    raise ValueError("published layout differs from the session configuration")
                self.layout = layout
            except Exception:
                self.layout = None

    def on_status(self, message):
        try:
            data = json.loads(message.data)
            with self.lock:
                if self.pending and data.get('trial_id') == self.pending.trial_id and data.get('ok') is False:
                    self.error = str(data.get('stage')) + ': ' + str(data.get('detail'))
        except (ValueError, TypeError, AttributeError):
            return

    def on_proposal(self, message):
        with self.lock:
            trial = self.pending
            if trial is None or message.source.trial_id != trial.trial_id:
                return
            try:
                source = message.source
                if (source.target_id != trial.target_id or source.condition != trial.condition
                        or source.layout.layout_id != trial.layout_id
                        or source.config_sha256 != self.config_sha256
                        or source.header.frame_id != self.cfg['frame_id']
                        or source.planning_group != self.cfg['motion']['planning_group']
                        or source.ee_link != self.cfg['motion']['ee_link']):
                    raise ValueError("proposal does not match the requested trial/configuration")
                snapshot_layout(source.layout, self.cfg['frame_id'], self.cfg['safety'])
                if (message.execution_ready or not message.retiming_required
                        or not message.physical_validation_required
                        or message.optimization_applied != (trial.condition == 'legible')):
                    raise ValueError("proposal has inconsistent optimization/validation flags")
                names0, q0, t0 = extract_path(source.trajectory)
                names, q, t = extract_path(message.trajectory)
                if (names != names0 or q.shape != q0.shape or not np.array_equal(t, t0)
                        or not np.array_equal(q[[0, -1]], q0[[0, -1]])):
                    raise ValueError("proposal changed joint order, timestamps or endpoints")
                if trial.condition == 'baseline' and not np.array_equal(q, q0):
                    raise ValueError("baseline proposal was modified")
                if self.proposal is not None:
                    raise ValueError("duplicate proposal for one trial")
                self.proposal = deepcopy(message)
            except Exception as error:
                self.error = str(error)

    def _wait(self, predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while not self.ros.is_shutdown():
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(label + " timed out")
            time.sleep(0.02)
        raise RuntimeError("ROS shutdown while waiting for " + label)

    def prepare(self, trial):
        """New ID per attempt. Neither validation nor motion is requested here."""
        with self.lock:
            if self.pending is not None:
                raise RuntimeError("a trial is already preparing")
            self.proposal = self.validation_id = None
        condition = {'predictable': 'baseline', 'baseline': 'baseline', 'legible': 'legible'}.get(trial['motion'])
        if condition is None:
            raise ValueError("unknown experimental condition")
        self._wait(lambda: self.layout is not None and self.trials.get_num_connections() > 0,
                   min(10.0, self.timeout), "matching layout/planner")
        with self.lock:
            if self.pending is not None:
                raise RuntimeError("a trial is already preparing")
            if self.layout is None:
                raise RuntimeError("layout changed while preparing the request")
            request = self.msg.TrialRequest()
            request.header.stamp, request.header.frame_id = self.ros.Time.now(), self.cfg['frame_id']
            request.trial_id = '%s-%s-%s' % (self.session_id, trial['trial_id'], uuid.uuid4().hex)
            request.target_id, request.condition = trial['target_id'], condition
            request.layout_id = self.layout.layout_id
            if request.target_id not in [b.id for b in self.layout.blocks]:
                raise ValueError("target absent from layout")
            self.pending, self.proposal, self.error, self.validation_id = request, None, None, None
        try:
            self.trials.publish(request)
            self._wait(lambda: self.error is not None or self.proposal is not None,
                       self.timeout, "planner/optimizer preparation")
            if self.error:
                raise RuntimeError(self.error)
            return deepcopy(self.proposal)
        except Exception:
            self.proposal = self.validation_id = None
            raise
        finally:
            with self.lock:
                self.pending = None

    def _service(self, name, kind, request, timeout):
        # Calls are bounded with wall time even if the ROS clock stops. A timeout
        # does NOT cancel a remote command: never retry it automatically.
        result = queue.Queue()
        def call():
            try:
                self.ros.wait_for_service(name, timeout=min(5.0, timeout))
                result.put((True, self.ros.ServiceProxy(name, kind)(request)))
            except Exception as error:
                result.put((False, error))
        threading.Thread(target=call, daemon=True).start()
        try:
            ok, response = result.get(timeout=timeout)
        except queue.Empty:
            raise RuntimeError(name + " timed out; remote state unknown, do not automatically retry")
        if not ok:
            raise RuntimeError(name + " unavailable/failed: " + str(response))
        if not response.accepted:
            raise RuntimeError(name + " rejected: " + response.detail)
        return response

    def validate(self):
        if self.proposal is None:
            raise RuntimeError("no prepared proposal")
        request = self.srv.ValidateTrajectoryRequest()
        request.proposal = deepcopy(self.proposal)
        request.required_duration = float(self.cfg['motion']['duration'])
        self.validation_id = None
        reply = self._service('/legibility/validate_trajectory', self.srv.ValidateTrajectory,
                              request, self.timeout)
        if (not reply.validation_id.strip() or reply.validation_id in self.used_validations
                or not math.isfinite(reply.duration)
                or abs(reply.duration - request.required_duration) > 1e-6):
            raise RuntimeError("validator must bind a fresh trajectory with the required study duration")
        self.validation_id = reply.validation_id
        return reply

    def request_motion(self, operation, *, operator_confirmed=False):
        """Only the optional experiment controller calls this after a human prompt."""
        if operator_confirmed is not True:
            raise RuntimeError("explicit operator confirmation is required")
        if operation not in ('home', 'run'):
            raise ValueError("unknown motion operation")
        if self.motion_uncertain or (self.active_command is not None
                                     and self.feedback_state not in ('done', 'home')):
            raise RuntimeError("previous motion state is unresolved; inspect before restarting the session")
        if operation == 'run' and (not self.validation_id or self.proposal is None):
            raise RuntimeError("run requires a separately validated proposal")
        request = self.srv.RequestMotionRequest()
        request.command_id, request.operation = uuid.uuid4().hex, operation
        request.config_sha256 = self.config_sha256
        request.operator_confirmed = True
        request.dispatch_deadline = self.ros.Time.from_sec(
            self.ros.Time.now().to_sec() + (7.0 if operation == 'run' else 25.0))
        request.trial_id = self.proposal.source.trial_id if operation == 'run' else ''
        request.validation_id = self.validation_id if operation == 'run' else ''
        with self.lock:
            self.active_command = request
            self.feedback_state = self.feedback_time = None
            while not self.feedback.empty():
                self.feedback.get_nowait()
        if operation == 'run':
            self.used_validations.add(self.validation_id)
            self.validation_id = None  # Consumed even if the request times out.
        try:
            return self._service('/legibility/request_motion', self.srv.RequestMotion, request, 10.0)
        except Exception:
            with self.lock:
                self.active_command = None
                self.motion_uncertain = True
            raise

    def on_feedback(self, message):
        with self.lock:
            command = self.active_command
            if (command is None or message.command_id != command.command_id
                    or message.trial_id != command.trial_id
                    or message.validation_id != command.validation_id):
                return  # Old attempts and other sessions cannot advance this trial.
            stamp = message.header.stamp.to_sec()
            if (not math.isfinite(stamp) or stamp <= 0
                    or (self.feedback_time is not None and stamp < self.feedback_time)):
                return
            allowed = ({None: {'home', 'failed'}} if command.operation == 'home' else
                       {None: {'executing', 'failed'}, 'executing': {'done', 'failed'}})
            if message.state not in allowed.get(self.feedback_state, set()):
                return
            self.feedback_state, self.feedback_time = message.state, stamp
            self.feedback.put((message.state, stamp))

    def wait_status(self, wanted, timeout):
        deadline = time.monotonic() + timeout
        while not self.ros.is_shutdown() and time.monotonic() < deadline:
            try:
                state, stamp = self.feedback.get(timeout=min(0.2, max(0.001, deadline - time.monotonic())))
            except queue.Empty:
                continue
            if state in wanted:
                return state, stamp
        return None, None
