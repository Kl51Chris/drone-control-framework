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
        raise ValueError(
            f"{name} must contain only finite values"
        )

    return array.copy()


def _finite_float(value: float, name: str) -> float:
    """Convert an input to a finite float."""
    result = float(value)

    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite")

    return result


@dataclass(slots=True)
class DroneState:
    """
    Estimated drone state used by all controllers.

    Coordinate and unit convention:
        position:
            World-frame position [x, y, z] in meters.

        velocity:
            World-frame velocity [vx, vy, vz] in meters/second.

        roll:
            Estimated roll angle in radians.

        pitch:
            Estimated pitch angle in radians.

        yaw:
            Estimated world-frame yaw angle in radians.

        timestamp:
            Measurement time in seconds.
    """

    position: Vector3
    velocity: Vector3

    roll: float
    pitch: float
    yaw: float

    timestamp: float = 0.0

    def __post_init__(self) -> None:
        self.position = _vector3(
            self.position,
            "position",
        )
        self.velocity = _vector3(
            self.velocity,
            "velocity",
        )

        self.roll = _finite_float(
            self.roll,
            "roll",
        )
        self.pitch = _finite_float(
            self.pitch,
            "pitch",
        )
        self.yaw = _finite_float(
            self.yaw,
            "yaw",
        )
        self.timestamp = _finite_float(
            self.timestamp,
            "timestamp",
        )

    @classmethod
    def zero(
        cls,
        timestamp: float = 0.0,
    ) -> DroneState:
        """Create a zero-valued drone state."""
        return cls(
            position=np.zeros(
                3,
                dtype=np.float64,
            ),
            velocity=np.zeros(
                3,
                dtype=np.float64,
            ),
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            timestamp=timestamp,
        )