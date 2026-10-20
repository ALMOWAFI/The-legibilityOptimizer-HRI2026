"""Topic wiring with atomic trial/layout snapshots and no execution interface."""
from copy import deepcopy
import json
import threading

import numpy as np

from experimental.legibility_optimizer.ros1_adapter import extract_path
from .layout import snapshot_layout


class TopicWorker:
    def __init__(self, runtime, messages=None, string_type=None, start_job=None):
        if messages is None:
            from hri_legibility_preview import msg as messages
        if string_type is None:
            from std_msgs.msg import String as string_type
        self.runtime, self.ros, self.messages = runtime, runtime.rospy, messages
        self.string_type = string_type
        self.status = self.ros.Publisher('/legibility/status', string_type, queue_size=20, latch=False)
        self._lock, self._busy, self._seen = threading.Lock(), False, set()
        self._start_job = start_job or (lambda work: threading.Thread(target=work, daemon=True).start())

    def report(self, trial_id, layout_id, stage, ok, detail):
        self.status.publish(self.string_type(data=json.dumps(dict(
            trial_id=trial_id, layout_id=layout_id, stage=stage, ok=bool(ok),
            detail=detail, execution_ready=False), allow_nan=False)))

    def submit(self, trial_id, layout_id, work):
        with self._lock:
            if self._busy:
                self.report(trial_id, layout_id, 'rejected_busy', False, 'one trial is already processing; resend later')
                return
            if trial_id in self._seen:
                self.report(trial_id, layout_id, 'rejected_duplicate', False, 'trial_id was already processed; use a new ID')
                return
            self._busy = True
            self._seen.add(trial_id)

        def run():
            try:
                work()
            except Exception as error:
                self.report(trial_id, layout_id, 'failed', False, str(error))
            finally:
                with self._lock:
                    self._busy = False
        self._start_job(run)


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


class TopicOptimizer(TopicWorker):
    def __init__(self, runtime, messages=None, string_type=None, start_job=None, subscribe=True):
        super().__init__(runtime, messages, string_type, start_job)
        self.candidate = self.ros.Publisher('/legibility/optimized_candidate', self.messages.OptimizedCandidate,
                                            queue_size=10, latch=False)
        self.subscribers = []
        if subscribe:
            self.subscribers.append(self.ros.Subscriber('/legibility/baseline', self.messages.BaselinePlan,
                                                       self.on_baseline, queue_size=10))

    def on_baseline(self, message):
        source = deepcopy(message)
        try:
            if not source.trial_id.strip() or source.condition not in ('baseline', 'legible'):
                raise ValueError('invalid trial metadata')
            if (source.header.frame_id != self.runtime.frame or source.planning_group != self.runtime.group_name
                    or source.ee_link != self.runtime.link or source.model_name != self.runtime.model_name
                    or source.config_sha256 != self.runtime.config_sha256):
                raise ValueError('baseline frame/model/group/tool/config do not match this optimizer')
            layout, ids, goals = snapshot_layout(source.layout, self.runtime.frame, self.runtime.cfg['safety'])
            if source.target_id not in ids:
                raise ValueError('target_id missing from baseline layout')
            extract_path(source.trajectory)
            self.submit(source.trial_id, layout.layout_id, lambda: self.optimize(source, ids, goals))
        except Exception as error:
            self.report(source.trial_id, source.layout.layout_id, 'baseline_rejected', False, str(error))

    def optimize(self, source, ids, goals):
        if source.condition == 'baseline':
            self.report(source.trial_id, source.layout.layout_id, 'optimizer_bypassed', True,
                        'predictable condition: downstream consumes /legibility/baseline')
            return
        request = self.runtime.services.OptimizeLegibilityRequest()
        request.trial_id, request.baseline, request.start_state = source.trial_id, source.trajectory, source.start_state
        request.goal_header.frame_id = source.layout.header.frame_id
        request.goals = [self.runtime.Point(*map(float, xyz)) for xyz in goals]
        request.intended_goal_index = ids.index(source.target_id)
        request.preview_validation = False
        self.report(source.trial_id, source.layout.layout_id, 'optimizing', True, 'FK-based numerical optimization')
        response = self.runtime.handle_optimize(request)
        if not response.numerical_accepted:
            self.report(source.trial_id, source.layout.layout_id, 'optimizer_rejected', False, response.diagnostics)
            return
        if not response.solver_success or response.execution_ready:
            raise ValueError('optimizer returned inconsistent acceptance/readiness flags')
        names0, q0, t0 = extract_path(source.trajectory)
        names, q, t = extract_path(response.candidate)
        if (names != names0 or q.shape != q0.shape or not np.array_equal(t, t0)
                or not np.array_equal(q[[0, -1]], q0[[0, -1]])):
            raise ValueError('candidate changed joint order, timestamp grid or endpoints')
        metrics = json.loads(response.diagnostics)
        if metrics.get('trial_id') != source.trial_id or metrics.get('config_sha256') != source.config_sha256:
            raise ValueError('optimizer diagnostics do not match the baseline trial/config')
        output = self.messages.OptimizedCandidate()
        output.source, output.trajectory = source, response.candidate
        output.solver_success, output.numerical_accepted = True, True
        output.retiming_required, output.physical_validation_required, output.execution_ready = True, True, False
        for key in ('baseline_legibility', 'optimized_legibility', 'baseline_joint_cost', 'optimized_joint_cost'):
            setattr(output, key, float(metrics[key]))
        output.diagnostics = response.diagnostics
        self.candidate.publish(output)
        self.report(source.trial_id, source.layout.layout_id, 'candidate_published', True,
                    'numerically accepted; downstream retiming and physical validation required')
