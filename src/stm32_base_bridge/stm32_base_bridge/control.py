"""Steady-clock command watchdog independent of ROS time and serial transport."""
import math


class CommandGate:
    def __init__(self, timeout=0.2, max_linear=0.5, max_angular=3.0):
        self.timeout = timeout
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.clear()

    def clear(self):
        self.command = (0.0, 0.0)
        self.received = None

    def accept(self, linear, angular, now, ready=True, unsupported=()):
        if (not ready or not all(math.isfinite(x) for x in (linear, angular, *unsupported))
                or any(abs(x) > 1e-9 for x in unsupported)):
            self.clear()
            return False
        self.command = (max(-self.max_linear, min(self.max_linear, linear)),
                        max(-self.max_angular, min(self.max_angular, angular)))
        self.received = now
        return True

    def output(self, now, ready=True):
        if not ready or self.received is None or now - self.received >= self.timeout:
            self.clear()
            return (0.0, 0.0)
        return self.command
