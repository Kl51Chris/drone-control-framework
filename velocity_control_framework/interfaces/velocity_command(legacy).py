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


def _positive_finite(value: float, name: str) -> float:
    """Convert a value to a finite positive float."""
    result = float(value)

    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite")

    if result <= 0.0:
        raise ValueError(f"{name} must be positive")

    return result


@dataclass(slots=True)
class VelocityCommand:
    """
    Velocity command produced by an outer-loop controller.

    Coordinate convention:
        velocity:
            World-frame velocity command [vx, vy, vz] in meters/second.

        yaw_rate:
            Yaw-rate command in radians/second.
    """

    velocity: Vector3
    yaw_rate: float = 0.0

    def __post_init__(self) -> None:
        self.velocity = _vector3(self.velocity, "velocity")
        self.yaw_rate = float(self.yaw_rate)

        if not np.isfinite(self.yaw_rate):
            raise ValueError("yaw_rate must be finite")

    @property
    def vx(self) -> float:
        return float(self.velocity[0])

    @property
    def vy(self) -> float:
        return float(self.velocity[1])

    @property
    def vz(self) -> float:
        return float(self.velocity[2])

    @classmethod
    def zero(cls) -> VelocityCommand:
        """Create a zero-velocity command."""
        return cls(
            velocity=np.zeros(3, dtype=np.float64),
            yaw_rate=0.0,
        )

    def clipped(
        self,
        max_horizontal_speed: float,
        max_vertical_speed: float,
        max_yaw_rate: float,
    ) -> VelocityCommand:
        """
        Return a safety-limited copy of the command.

        Horizontal velocity is limited by vector magnitude rather than
        clipping the x and y components independently.
        """
        max_horizontal_speed = _positive_finite(
            max_horizontal_speed,
            "max_horizontal_speed",
        )
        max_vertical_speed = _positive_finite(
            max_vertical_speed,
            "max_vertical_speed",
        )
        max_yaw_rate = _positive_finite(
            max_yaw_rate,
            "max_yaw_rate",
        )

        limited_velocity = self.velocity.copy()

        horizontal_speed = float(
            np.linalg.norm(limited_velocity[:2])
        )

        if horizontal_speed > max_horizontal_speed:
            limited_velocity[:2] *= (
                max_horizontal_speed / horizontal_speed
            )

        limited_velocity[2] = np.clip(
            limited_velocity[2],
            -max_vertical_speed,
            max_vertical_speed,
        )

        limited_yaw_rate = float(
            np.clip(
                self.yaw_rate,
                -max_yaw_rate,
                max_yaw_rate,
            )
        )

        return VelocityCommand(
            velocity=limited_velocity,
            yaw_rate=limited_yaw_rate,
        )