from __future__ import annotations

from dataclasses import dataclass
import math

from .config import PIDSettings


@dataclass(frozen=True, slots=True)
class PIDResult:
    error: float
    proportional: float
    integral: float
    derivative: float
    unsaturated: float
    output: float
    saturated: bool


class AltitudePID:
    """Discrete PID with conditional-integration anti-windup."""

    def __init__(self, settings: PIDSettings) -> None:
        self.settings = settings
        self.reset()

    def reset(self) -> None:
        self.integral_state = 0.0
        self.previous_error: float | None = None

    def update(self, reference_m: float, altitude_m: float, dt_s: float) -> PIDResult:
        if not math.isfinite(dt_s) or dt_s <= 0:
            raise ValueError("dt_s must be finite and positive")
        if not math.isfinite(reference_m) or not math.isfinite(altitude_m):
            raise ValueError("reference_m and altitude_m must be finite")
        error = float(reference_m) - float(altitude_m)
        derivative_value = 0.0 if self.previous_error is None else (error - self.previous_error) / dt_s
        candidate_integral = self.integral_state + error * dt_s
        p = self.settings.kp * error
        i = self.settings.ki * candidate_integral
        d = self.settings.kd * derivative_value
        raw = p + i + d
        output = min(max(raw, self.settings.correction_min), self.settings.correction_max)
        integral_drive = self.settings.ki * error
        accept = raw == output or (raw > self.settings.correction_max and integral_drive < 0) or (raw < self.settings.correction_min and integral_drive > 0)
        if self.settings.ki != 0 and accept:
            self.integral_state = candidate_integral
        self.previous_error = error
        return PIDResult(error, p, i, d, raw, output, raw != output)
