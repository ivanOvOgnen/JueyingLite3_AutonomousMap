"""Thread-safe epochs, source-time health and exclusive operator leases."""
import math
import secrets
import threading
import time


class RuntimeState:
    def __init__(self, *, clock=time.monotonic, lease_seconds=0.8):
        self.lock = threading.RLock()
        self.clock = clock
        self.epoch = 0
        self.reason = 'Startup'
        self.streams = {}
        self.lease_seconds = lease_seconds
        self.lease_owner = None
        self.lease_token = None
        self.lease_until = 0.0

    def reset(self, reason):
        with self.lock:
            self.epoch += 1
            self.reason = reason
            self.streams.clear()
            self.release()
            return self.epoch

    def observe(self, key, stamp_ns, now_ns, *, epoch=None):
        with self.lock:
            if epoch is not None and epoch != self.epoch:
                return False
            valid = isinstance(stamp_ns, int) and not isinstance(stamp_ns, bool) and stamp_ns > 0
            if not valid:
                stamp_ns = 0
            previous = self.streams.get(key)
            ordered = previous is None or stamp_ns > previous['stamp_ns']
            self.streams[key] = {'stamp_ns': stamp_ns, 'rx': self.clock(),
                                 'source_age_at_rx': (now_ns-stamp_ns)/1e9,
                                 'valid': valid and ordered}
            return valid and ordered

    def health(self, key, *, max_age=2.0, acquisition=True):
        with self.lock:
            sample = self.streams.get(key)
            if sample is None:
                return {'state': 'not_ready', 'receipt_age_seconds': None, 'source_age_seconds': None}
            age = self.clock() - sample['rx']
            source_age = sample['source_age_at_rx'] + age
            valid = sample['valid'] and 0 <= age <= max_age and (
                not acquisition or -0.05 <= source_age <= max_age)
            return {'state': 'ready' if valid else 'not_ready', 'receipt_age_seconds': age,
                    'source_age_seconds': source_age, 'stamp_ns': sample['stamp_ns'],
                    'acquisition_time_verified': bool(acquisition and valid)}

    def acquire(self, owner):
        if not isinstance(owner, str) or not owner:
            raise ValueError('Authenticated owner is required')
        with self.lock:
            now = self.clock()
            if self.lease_owner is not None:
                if now >= self.lease_until:
                    raise ValueError('Previous lease expired; STOP/re-arm is required')
                if self.lease_owner != owner:
                    raise ValueError('Another operator owns the control lease')
                # Reacquiring must not rotate a token held by an active goal.
                return self.lease_token
            self.lease_owner, self.lease_token = owner, secrets.token_urlsafe(32)
            self.lease_until = now + self.lease_seconds
            return self.lease_token

    def renew(self, owner, token, epoch):
        with self.lock:
            if epoch != self.epoch or not self.valid_lease(owner, token):
                raise ValueError('Lease is expired, invalid or from another session')
            self.lease_until = self.clock() + self.lease_seconds

    def valid_lease(self, owner=None, token=None):
        with self.lock:
            return bool(self.lease_owner and self.clock() < self.lease_until and
                        (owner is None or owner == self.lease_owner) and
                        ((owner is None and token is None) or isinstance(token, str) and secrets.compare_digest(token, self.lease_token)))

    def release(self):
        with self.lock:
            self.lease_owner = self.lease_token = None
            self.lease_until = 0.0
