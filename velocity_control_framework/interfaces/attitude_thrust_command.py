from __future__ import annotations

from dataclasses import dataclass
import math


def _finite_float(value: float, name: str) -> float:
    """Convert a value to float and require it to be finite."""
    result = float(value)

    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")

    return result


def _positive_finite(value: float, name: str) -> float:
    """Convert a value to a finite positive float."""
    result = _finite_float(value, name)

    if result <= 0.0:
        raise ValueError(f"{name} must be positive")

    return result


@dataclass(slots=True)
class AttitudeThrustCommand:
    """
    Attitude and collective-thrust command produced by a controller.

    Coordinate and unit convention:
        roll:
            Desired roll angle in radians.

        pitch:
            Desired pitch angle in radians.

        yaw_rate:
            Desired yaw rate in radians/second.

        thrust:
            Normalized collective-thrust command in the range [0.0, 1.0].

    This class is hardware independent. Conversion to Crazyflie
    send_setpoint() units is handled by the hardware adapter.
    """

    roll: float
    pitch: float
    yaw_rate: float
    thrust: float

    def __post_init__(self) -> None:
        self.roll = _finite_float(self.roll, "roll")
        self.pitch = _finite_float(self.pitch, "pitch")
        self.yaw_rate = _finite_float(
            self.yaw_rate,
            "yaw_rate",
        )
        self.thrust = _finite_float(self.thrust, "thrust")

        if not 0.0 <= self.thrust <= 1.0:
            raise ValueError(
                "thrust must be within the normalized range [0.0, 1.0]"
            )

    @classmethod
    def zero(cls) -> AttitudeThrustCommand:
        """
        Create a zero-attitude, zero-thrust command.

        This is not a hover command. It represents no motor thrust.
        """
        return cls(
            roll=0.0,
            pitch=0.0,
            yaw_rate=0.0,
            thrust=0.0,
        )

    def clipped(
        self,
        max_roll: float,
        max_pitch: float,
        max_yaw_rate: float,
        min_thrust: float = 0.0,
        max_thrust: float = 1.0,
    ) -> AttitudeThrustCommand:
        """
        Return a safety-limited copy of the command.

        Angle and yaw-rate limits must be positive. Thrust limits use
        normalized collective-thrust units.
        """
        max_roll = _positive_finite(max_roll, "max_roll")
        max_pitch = _positive_finite(max_pitch, "max_pitch")
        max_yaw_rate = _positive_finite(
            max_yaw_rate,
            "max_yaw_rate",
        )

        min_thrust = _finite_float(min_thrust, "min_thrust")
        max_thrust = _finite_float(max_thrust, "max_thrust")

        if not 0.0 <= min_thrust <= 1.0:
            raise ValueError(
                "min_thrust must be within [0.0, 1.0]"
            )

        if not 0.0 <= max_thrust <= 1.0:
            raise ValueError(
                "max_thrust must be within [0.0, 1.0]"
            )

        if min_thrust > max_thrust:
            raise ValueError(
                "min_thrust must not exceed max_thrust"
            )

        return AttitudeThrustCommand(
            roll=max(-max_roll, min(self.roll, max_roll)),
            pitch=max(-max_pitch, min(self.pitch, max_pitch)),
            yaw_rate=max(
                -max_yaw_rate,
                min(self.yaw_rate, max_yaw_rate),
            ),
            thrust=max(
                min_thrust,
                min(self.thrust, max_thrust),
            ),
        )