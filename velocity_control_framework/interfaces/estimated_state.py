from __future__ import annotations

from dataclasses import dataclass

from ._state_validation import Vector3, finite_float, vector3


@dataclass(slots=True)
class EstimatedState:
    """State information produced by a source-specific estimator.

    Position and velocity are world-frame values in meters and meters/second.
    Attitude angles are radians. ``p``, ``q``, and ``r`` are body-frame
    angular rates about body x, y, and z in radians/second; they are not
    generally Euler angle derivatives.
    """

    position: Vector3
    velocity: Vector3
    roll: float
    pitch: float
    yaw: float
    timestamp: float
    p: float
    q: float
    r: float

    def __post_init__(self) -> None:
        self.position = vector3(self.position, "position")
        self.velocity = vector3(self.velocity, "velocity")
        self.roll = finite_float(self.roll, "roll")
        self.pitch = finite_float(self.pitch, "pitch")
        self.yaw = finite_float(self.yaw, "yaw")
        self.timestamp = finite_float(self.timestamp, "timestamp")
        self.p = finite_float(self.p, "p")
        self.q = finite_float(self.q, "q")
        self.r = finite_float(self.r, "r")
