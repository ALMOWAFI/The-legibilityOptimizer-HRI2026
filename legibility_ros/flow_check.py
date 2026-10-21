"""Check registered topic edges AND observed messages for one planning-only trial."""
from copy import deepcopy
import json
import math
import queue
import threading
import time
import uuid

import numpy as np

from experimental.legibility_optimizer.ros1_adapter import extract_path
from .layout import layout_digest


def graph_errors(state, types, namespace='/legibility_preview'):
    """ROS Master registration is a prerequisite, not proof of message delivery."""
    pubs, subs, _ = state
    pubs, subs, types = dict(pubs), dict(subs), dict(types)
    prefix = '/' + namespace.strip('/') if namespace.strip('/') else ''
    planner, optimizer, layout = (prefix + '/' + n for n in ('planner', 'optimizer', 'block_layout'))
    expected = [
        ('/hri/block_layout', 'BlockLayout', layout, planner),
        ('/legibility/trial_request', 'TrialRequest', None, planner),
        ('/legibility/baseline', 'BaselinePlan', planner, optimizer),
        ('/legibility/optimized_candidate', 'OptimizedCandidate', optimizer, None),
        ('/legibility/trajectory_proposal', 'TrajectoryProposal', optimizer, None),
        ('/legibility/status', 'std_msgs/String', planner, None),
        ('/legibility/status', 'std_msgs/String', optimizer, None)]
    errors = []
    for topic, kind, publisher, subscriber in expected:
        full_type = kind if '/' in kind else 'hri_legibility_preview/' + kind
        if types.get(topic) != full_type:
            errors.append(topic + ': missing/wrong type (expected ' + full_type + ')')
        if publisher and publisher not in pubs.get(topic, []):
            errors.append(publisher + ' is not publishing ' + topic)
        if subscriber and subscriber not in subs.get(topic, []):
            errors.append(subscriber + ' is not subscribed to ' + topic)
    return errors


def same_path(left, right):
    n0, q0, t0 = extract_path(left)
    n1, q1, t1 = extract_path(right)
    return n0 == n1 and np.array_equal(q0, q1) and np.array_equal(t0, t1)


class FlowCheck:
    def __init__(self, ros, messages, string_type, graph, report=print, namespace='/legibility_preview'):
        self.ros, self.msg, self.graph, self.report, self.namespace = ros, messages, graph, report, namespace
        self.lock = threading.RLock()
        self.layout, self.request, self.error = None, None, None
        self.received = {}
        self.publisher = ros.Publisher('/legibility/trial_request', messages.TrialRequest,
                                       queue_size=1, latch=False)
        self.subscribers = [
            ros.Subscriber('/hri/block_layout', messages.BlockLayout, self.on_layout, queue_size=1),
            ros.Subscriber('/legibility/baseline', messages.BaselinePlan,
                           lambda m: self.on_result('baseline', m, m), queue_size=10),
            ros.Subscriber('/legibility/optimized_candidate', messages.OptimizedCandidate,
                           lambda m: self.on_result('candidate', m, m.source), queue_size=10),
            ros.Subscriber('/legibility/trajectory_proposal', messages.TrajectoryProposal,
                           lambda m: self.on_result('proposal', m, m.source), queue_size=10),
            ros.Subscriber('/legibility/status', string_type, self.on_status, queue_size=20)]

    def on_layout(self, message):
        with self.lock:
            try:
                if message.layout_id != layout_digest(message):
                    raise ValueError('layout digest mismatch')
                self.layout = deepcopy(message)
            except (ValueError, TypeError, AttributeError):
                self.layout = None

    def on_result(self, stage, message, source):
        with self.lock:
            if self.request and source.trial_id == self.request.trial_id:
                if stage in self.received:
                    self.error = 'duplicate ' + stage + ' for this trial'
                else:
                    self.received[stage] = deepcopy(message)
                    self.report('RECEIVED ' + stage + ': ' + source.trial_id)

    def on_status(self, message):
        try:
            data = json.loads(message.data)
            with self.lock:
                if self.request and data.get('trial_id') == self.request.trial_id:
                    self.report('STATUS ' + str(data.get('stage')) + ': ' + str(data.get('detail'))[:500])
                    if data.get('ok') is False:
                        self.error = str(data.get('stage')) + ': ' + str(data.get('detail'))
        except (ValueError, TypeError, AttributeError):
            return

    def wait_ready(self, timeout):
        # A daemon worker bounds master RPC hangs without changing socket settings
        # used by rospy's transport threads. Only one master lookup is in flight.
        results = queue.Queue()
        deadline, pending, errors = time.monotonic() + timeout, False, ['waiting for ROS graph']
        def query():
            try:
                results.put((self.graph(), None))
            except Exception as error:
                results.put((None, str(error)))
        while not self.ros.is_shutdown() and time.monotonic() < deadline:
            if not pending:
                threading.Thread(target=query, daemon=True).start()
                pending = True
            try:
                snapshot, error = results.get(timeout=min(.1, max(.001, deadline - time.monotonic())))
                pending = False
                errors = ['ROS master: ' + error] if error else graph_errors(*snapshot, namespace=self.namespace)
            except queue.Empty:
                continue
            with self.lock:
                if self.layout is None:
                    errors.append('no valid /hri/block_layout message received')
                if self.publisher.get_num_connections() == 0:
                    errors.append('trial publisher has no transport connection')
                for subscriber in self.subscribers:
                    if subscriber.get_num_connections() == 0:
                        errors.append('no transport connection on ' + subscriber.name)
                if not errors:
                    self.report('READY: node subscriptions, message types, transport connections and layout checked')
                    return
            time.sleep(.05)
        raise RuntimeError('pipeline not ready: ' + '; '.join(errors))

    def run(self, target_id, condition='legible', timeout=180., ready_timeout=20.):
        if condition not in ('baseline', 'legible'):
            raise ValueError('condition must be baseline or legible')
        if any(not math.isfinite(t) or t <= 0 for t in (timeout, ready_timeout)):
            raise ValueError('timeouts must be finite and positive')
        self.wait_ready(ready_timeout)
        with self.lock:
            if self.layout is None:
                raise RuntimeError('layout became invalid')
            if target_id not in [b.id for b in self.layout.blocks]:
                raise ValueError('target ID is absent from the received layout')
            request = self.msg.TrialRequest()
            request.header.frame_id, request.header.stamp = self.layout.header.frame_id, self.ros.Time.now()
            request.trial_id = 'flow-check-' + uuid.uuid4().hex
            request.target_id, request.condition, request.layout_id = target_id, condition, self.layout.layout_id
            self.request, self.received, self.error = request, {}, None
        wanted = {'baseline', 'proposal'} | ({'candidate'} if condition == 'legible' else set())
        self.report('SENDING ' + condition + ' trial ' + request.trial_id + '; planning only')
        self.publisher.publish(request)
        deadline = time.monotonic() + timeout
        while not self.ros.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                if self.error:
                    raise RuntimeError(self.error)
                if wanted.issubset(self.received):
                    self.validate_result()
                    self.report('PASS: matching trial data crossed the pipeline; no execution was requested')
                    return deepcopy(self.received['proposal'])
            time.sleep(.05)
        with self.lock:
            missing = ', '.join(sorted(wanted - self.received.keys()))
        raise RuntimeError('trial timed out/shutdown; missing: ' + missing)

    def validate_result(self):
        request, baseline, proposal = self.request, self.received['baseline'], self.received['proposal']
        sources = [baseline, proposal.source]
        candidate = self.received.get('candidate')
        if candidate is not None:
            sources.append(candidate.source)
        for source in sources:
            if (source.trial_id != request.trial_id or source.target_id != request.target_id
                    or source.condition != request.condition or source.layout.layout_id != request.layout_id
                    or source.header.frame_id != request.header.frame_id
                    or source.layout.header.frame_id != request.header.frame_id
                    or layout_digest(source.layout) != request.layout_id):
                raise ValueError('received output does not match the requested trial/layout')
            for field in ('planning_group', 'ee_link', 'model_name', 'config_sha256', 'baseline_mode'):
                if not getattr(source, field) or getattr(source, field) != getattr(baseline, field):
                    raise ValueError('inconsistent source metadata: ' + field)
            if not same_path(source.trajectory, baseline.trajectory):
                raise ValueError('output refers to a different baseline trajectory')
        outputs = [proposal] + ([candidate] if candidate is not None else [])
        for output in outputs:
            if output.execution_ready or not output.retiming_required or not output.physical_validation_required:
                raise ValueError('output incorrectly advertises validation/readiness')
        if proposal.optimization_applied != (request.condition == 'legible'):
            raise ValueError('wrong optimization mode in proposal')
        names, q, t = extract_path(proposal.trajectory)
        n0, q0, t0 = extract_path(baseline.trajectory)
        if (names != n0 or q.shape != q0.shape or not np.array_equal(t, t0)
                or not np.array_equal(q[[0, -1]], q0[[0, -1]])):
            raise ValueError('proposal changed joint order, time grid or endpoints')
        if request.condition == 'baseline':
            if candidate is not None or not same_path(proposal.trajectory, baseline.trajectory):
                raise ValueError('baseline trial did not bypass optimization')
        elif (candidate is None or not candidate.solver_success or not candidate.numerical_accepted
              or not same_path(candidate.trajectory, proposal.trajectory)):
            raise ValueError('proposal does not match an accepted optimizer result')
