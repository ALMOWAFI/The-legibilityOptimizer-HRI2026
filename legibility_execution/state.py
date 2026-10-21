"""Fresh arm-only measurements. Never infer successful motion from elapsed time."""
from collections import deque
from dataclasses import dataclass
import threading
import time
import numpy as np


@dataclass
class Sample:
    sequence: int
    stamp: float
    received: float
    positions: np.ndarray
    velocities: np.ndarray


class ArmState:
    def __init__(self, names, now, clock=time.monotonic, max_age=0.5):
        self.names, self.now, self.clock, self.max_age = tuple(names), now, clock, max_age
        self.lock = threading.Lock()
        self.samples = deque(maxlen=4096)
        self.sequence = 0
        self.error = 'no joint state received'

    def update(self, message):
        with self.lock:
            try:
                names = list(message.name)
                if len(set(names)) != len(names) or len(message.position) != len(names) or len(message.velocity) != len(names):
                    raise ValueError('joint state requires unique names, positions and measured velocities')
                indices = [names.index(n) for n in self.names]
                q, v = np.asarray(message.position)[indices], np.asarray(message.velocity)[indices]
                stamp, now = message.header.stamp.to_sec(), self.now()
                if (not np.isfinite([q, v]).all() or not np.isfinite([stamp, now]).all()
                        or stamp <= 0 or not -0.05 <= now - stamp <= self.max_age
                        or (self.samples and stamp <= self.samples[-1].stamp)):
                    raise ValueError('invalid, stale or non-increasing joint-state timestamp/data')
                self.sequence += 1
                self.samples.append(Sample(self.sequence, stamp, self.clock(), q.copy(), v.copy()))
                self.error = None
            except Exception as error:
                self.error = str(error)

    def current(self):
        with self.lock:
            if self.error or not self.samples:
                raise RuntimeError(self.error or 'no arm measurement')
            sample = self.samples[-1]
            age = self.now() - sample.stamp
            if self.clock() - sample.received > self.max_age or not -0.05 <= age <= self.max_age:
                raise RuntimeError('arm measurements are stale or ROS clock changed')
            return sample

    def after(self, sequence):
        self.current()
        with self.lock:
            if self.samples[0].sequence > sequence + 1:
                raise RuntimeError('arm measurement history overflow')
            return [s for s in self.samples if s.sequence > sequence]
