"""Fixed-period PD control for five normalized line sensor readings."""

from dataclasses import dataclass, field
from math import isfinite
from typing import Sequence


SENSOR_POSITIONS = (-2, -1, 0, 1, 2)


def limit_output(value: float) -> float:
    return max(0.0, min(1.0, value))


@dataclass
class LineFollower:
    """Positive error means the line is to the right of the vehicle."""

    kp: float = 0.22
    kd: float = 0.002
    base_speed: float = 0.35
    period_s: float = 0.02
    _previous_error: float | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        values = (self.kp, self.kd, self.base_speed, self.period_s)
        if not all(isfinite(value) for value in values):
            raise ValueError("controller settings must be finite")
        if min(self.kp, self.kd) < 0 or not 0 <= self.base_speed <= 1 or self.period_s <= 0:
            raise ValueError("invalid gains, base speed, or control period")

    def step(self, readings: Sequence[float]) -> tuple[float, float]:
        """Return left and right forward outputs in [0, 1]."""
        if len(readings) != len(SENSOR_POSITIONS) or not all(0 <= value <= 1 for value in readings):
            raise ValueError("provide five normalized readings from left to right")
        total = sum(readings)
        if total == 0:
            self._previous_error = None
            return 0.0, 0.0

        error = sum(position * value for position, value in zip(SENSOR_POSITIONS, readings)) / total
        derivative = 0.0
        if self._previous_error is not None:
            derivative = (error - self._previous_error) / self.period_s
        self._previous_error = error
        correction = self.kp * error + self.kd * derivative
        return limit_output(self.base_speed + correction), limit_output(self.base_speed - correction)
