from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray


Vector3 = NDArray[np.float64]


def _vector3(value: ArrayLike, name: str) -> Vector3:
    """Convert an input to a finite float64 vector with shape (3,)."""
    array = np.asarray(value, dtype=np.float64)

    if array.shape != (3,):
        raise ValueError(
            f"{name} must have shape (3,), got {array.shape}"
        )

    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")

    return array.copy()


@dataclass(slots=True)
class Reference:
    """
    Desired trajectory reference for an outer-loop controller.

    Coordinate convention:
        position:
            Desired world-frame position [x, y, z] in meters.

        velocity:
            Desired or feedforward world-frame velocity
            [vx, vy, vz] in meters/second.

        yaw:
            Desired world-frame yaw angle in radians.

        yaw_rate:
            Desired yaw rate in radians/second.

        timestamp:
            Time associated with this reference in seconds.
            The clock source must be defined by the runner.
    """

    position: Vector3
    velocity: Vector3

    yaw: float = 0.0
    yaw_rate: float = 0.0
    timestamp: float = 0.0

    def __post_init__(self) -> None:
        self.position = _vector3(self.position, "position")
        self.velocity = _vector3(self.velocity, "velocity")

        self.yaw = float(self.yaw)
        self.yaw_rate = float(self.yaw_rate)
        self.timestamp = float(self.timestamp)

        if not np.isfinite(self.yaw):
            raise ValueError("yaw must be finite")

        if not np.isfinite(self.yaw_rate):
            raise ValueError("yaw_rate must be finite")

        if not np.isfinite(self.timestamp):
            raise ValueError("timestamp must be finite")

    @classmethod
    def hover(
        cls,
        x: float,
        y: float,
        z: float,
        yaw: float = 0.0,
        timestamp: float = 0.0,
    ) -> Reference:
        """Create a stationary position reference."""
        return cls(
            position=np.array([x, y, z], dtype=np.float64),
            velocity=np.zeros(3, dtype=np.float64),
            yaw=yaw,
            yaw_rate=0.0,
            timestamp=timestamp,
        )