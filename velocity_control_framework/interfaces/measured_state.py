from __future__ import annotations

from dataclasses import dataclass

from ._state_validation import Vector3, finite_float, vector3


@dataclass(slots=True)
class MeasuredState:
    """Optional state information from direct or external measurements.

    Each field may be unavailable. This supports future sources such as motion
    capture without fabricating values. When supplied, vectors use world-frame
    SI units, attitude uses radians, and ``p``, ``q``, ``r`` are body-frame
    angular rates in radians/second, not Euler angle derivatives.
    """

    position: Vector3 | None = None
    velocity: Vector3 | None = None
    roll: float | None = None
    pitch: float | None = None
    yaw: float | None = None
    timestamp: float | None = None
    p: float | None = None
    q: float | None = None
    r: float | None = None

    def __post_init__(self) -> None:
        if self.position is not None:
            self.position = vector3(self.position, "position")
        if self.velocity is not None:
            self.velocity = vector3(self.velocity, "velocity")

        for name in ("roll", "pitch", "yaw", "timestamp", "p", "q", "r"):
            value = getattr(self, name)
            if value is not None:
                setattr(self, name, finite_float(value, name))
