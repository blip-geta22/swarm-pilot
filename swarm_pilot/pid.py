"""PID controller, with anti-windup. The same maths as a real robot,
just without the hardware."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PID:
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    output_limit: float = 10.0
    integral_limit: float = 10.0

    def __post_init__(self):
        self._integral = 0.0
        self._prev_error: float | None = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            raise ValueError("dt must be positive")
        self._integral += error * dt
        # Anti-windup: clamp the integral term.
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral))
        derivative = 0.0
        if self._prev_error is not None:
            derivative = (error - self._prev_error) / dt
        self._prev_error = error
        out = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, out))
