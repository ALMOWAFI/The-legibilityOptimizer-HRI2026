"""Single-use validation and command state machine, with injectable robot I/O."""
from copy import deepcopy
from dataclasses import dataclass
import threading
import time
import uuid


@dataclass
class Ticket:
    trial_id: str
    artifact: object
    deadline: float


class ExecutionGate:
    def __init__(self, backend, config_sha256, enabled=False, ttl=120., clock=time.monotonic,
                 start_worker=None):
        if not 0 < ttl <= 600:
            raise ValueError('validation lifetime must be in (0, 600] seconds')
        self.backend, self.config_sha256, self.enabled = backend, config_sha256, enabled
        self.ttl, self.clock = ttl, clock
        self.lock = threading.Lock()
        self.active = self.faulted = False
        self.tickets, self.commands = {}, set()
        self.last_completed = None
        self.start_worker = start_worker or (lambda work: threading.Thread(target=work, daemon=True).start())

    def validate(self, proposal, required_duration):
        # Preparation is serialized with execution and never calls a controller.
        if not self.lock.acquire(blocking=False):
            raise RuntimeError('executor is busy')
        try:
            if self.active or self.faulted:
                raise RuntimeError('executor is busy or faulted; inspect before restarting')
            if proposal.source.config_sha256 != self.config_sha256:
                raise ValueError('configuration mismatch')
            self.tickets.clear()
            artifact = self.backend.prepare(deepcopy(proposal), required_duration)
            token = uuid.uuid4().hex
            # One pending reach; a newer validation invalidates older ones.
            self.tickets = {token: Ticket(proposal.source.trial_id, deepcopy(artifact), self.clock() + self.ttl)}
            return token, artifact['duration'], artifact['detail']
        finally:
            self.lock.release()

    def request(self, request):
        if not self.lock.acquire(blocking=False):
            raise RuntimeError('executor is busy')
        try:
            if not self.enabled:
                raise RuntimeError('execution is disabled; validation-only node')
            if getattr(request, 'operator_confirmed', False) is not True:
                raise RuntimeError('explicit operator confirmation is required')
            if self.active or self.faulted:
                raise RuntimeError('executor is busy or faulted; no command queued')
            if (not request.command_id.strip() or request.command_id in self.commands
                    or request.config_sha256 != self.config_sha256):
                raise ValueError('missing/duplicate command ID or configuration mismatch')
            if len(self.commands) >= 10000:
                raise RuntimeError('command history full; restart between sessions')
            deadline = self.backend.dispatch_deadline(request)
            if request.operation == 'run':
                ticket = self.tickets.pop(request.validation_id, None)
                if ticket is None or ticket.deadline <= self.clock() or ticket.trial_id != request.trial_id:
                    raise ValueError('unknown, consumed, expired or wrong-trial validation ID')
                artifact = ticket.artifact
            elif request.operation == 'home':
                if request.validation_id or request.trial_id:
                    raise ValueError('home must not carry a reach token or trial')
                artifact = None
            else:
                raise ValueError('operation must be run or home')
            command = deepcopy(request)
            self.commands.add(command.command_id)
            self.tickets.clear()
            self.active = True
        finally:
            self.lock.release()

        def work():
            try:
                selected = artifact if command.operation == 'run' else self.backend.prepare_home(self.last_completed)
                self.backend.perform(command, selected, deadline)
                self.last_completed = deepcopy(selected) if command.operation == 'run' else None
            except Exception as error:
                # Backend cancels its own active goal before propagating failures.
                self.faulted = True
                self.backend.failed(command, str(error))
            finally:
                with self.lock:
                    self.active = False
        try:
            self.start_worker(work)
        except Exception:
            self.active, self.faulted = False, True
            raise
        return 'accepted; measured outcome follows on /legibility/motion_feedback'
