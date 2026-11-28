"""Scaled line-inspection reference and controlled observation faults for CrazySim."""

from dataclasses import dataclass, replace

import numpy as np

from velocity_control_framework.references import CircularFlightReference


class LineInspectionReference(CircularFlightReference):
    """Inspect a straight segment along X, then retrace it for landing."""

    distance_m = 0.12
    duration_s = 12.0

    def _circle_reference(self, phase_time):
        leg = self.config.circle_duration / 2
        origin = float(self._initial_position[0])
        end = origin + self.distance_m
        if phase_time <= leg:
            x, vx = self._smooth_segment(origin, end, phase_time, leg)
        else:
            x, vx = self._smooth_segment(end, origin, phase_time - leg, leg)
        return x, float(self._initial_position[1]), vx, 0.0


@dataclass(frozen=True, slots=True)
class NavigationFaultConfig:
    profile: str
    start_step: int = 150
    end_step: int = 350
    drift_y_m_s: float = 0.005
    bias_y_m: float = 0.025

    def __post_init__(self):
        if self.profile not in ("nominal", "position-loss", "position-bias"):
            raise ValueError("unknown inspection navigation profile")


class NavigationFault:
    """Perturb the controller's position input; retain nominal safety sensing."""

    def __init__(self, config: NavigationFaultConfig):
        self.config = config
        self.last_position = None
        self.last_timestamp = None

    def apply(self, state, step):
        cfg = self.config
        active = cfg.profile != "nominal" and cfg.start_step <= step < cfg.end_step
        position = state.position.copy()
        if active and cfg.profile == "position-loss":
            if self.last_position is None or self.last_timestamp is None:
                raise ValueError("position loss requires a preceding position observation")
            dt = max(0.0, state.timestamp - self.last_timestamp)
            position = self.last_position + state.velocity * dt
            position[1] += cfg.drift_y_m_s * dt
        elif active:
            position[1] += cfg.bias_y_m
        self.last_position = position.copy()
        self.last_timestamp = state.timestamp
        return replace(state, position=position), {
            "step": step, "profile": cfg.profile, "active": active,
            "raw_position": state.position.tolist(), "used_position": position.tolist(),
        }
