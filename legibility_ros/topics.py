"""Shared status reporting and single-trial scheduling for the two nodes."""
import json
import threading


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
