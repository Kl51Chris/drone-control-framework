from __future__ import annotations

from dataclasses import dataclass, field
import math

from velocity_control_framework.interfaces import (
    BodyRateThrustCommand,
    DroneState,
    Reference,
)


def _finite_float(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


@dataclass(frozen=True, slots=True)
class PIDConfig:
    """Gains and output limits for one independent PID channel."""

    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    output_min: float = 0.0
    output_max: float = 0.0

    def __post_init__(self) -> None:
        for name in ("kp", "ki", "kd", "output_min", "output_max"):
            object.__setattr__(
                self,
                name,
                _finite_float(getattr(self, name), name),
            )

        if self.output_min > self.output_max:
            raise ValueError("output_min must not exceed output_max")


@dataclass(frozen=True, slots=True)
class HorizontalCommandMapping:
    """Configured polarities from body translation channels to body rates.

    The vehicle-axis association is body-x to pitch and body-y to roll. The
    corresponding polarities must be selected explicitly because they are not
    established unambiguously by the active repository documentation.
    """

    body_x_to_pitch_rate_sign: int
    body_y_to_roll_rate_sign: int

    def __post_init__(self) -> None:
        if self.body_x_to_pitch_rate_sign not in (-1, 1):
            raise ValueError(
                "body_x_to_pitch_rate_sign must be either -1 or 1"
            )
        if self.body_y_to_roll_rate_sign not in (-1, 1):
            raise ValueError(
                "body_y_to_roll_rate_sign must be either -1 or 1"
            )


@dataclass(frozen=True, slots=True)
class OuterLoopPIDConfig:
    """Configuration for the four-channel outer-loop controller."""

    horizontal_mapping: HorizontalCommandMapping
    horizontal_x: PIDConfig = field(default_factory=PIDConfig)
    horizontal_y: PIDConfig = field(default_factory=PIDConfig)
    altitude: PIDConfig = field(default_factory=PIDConfig)
    heading: PIDConfig = field(default_factory=PIDConfig)
    equilibrium_thrust: float = 0.0
    min_thrust: float = 0.0
    max_thrust: float = 1.0

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "equilibrium_thrust",
            _finite_float(self.equilibrium_thrust, "equilibrium_thrust"),
        )
        object.__setattr__(
            self,
            "min_thrust",
            _finite_float(self.min_thrust, "min_thrust"),
        )
        object.__setattr__(
            self,
            "max_thrust",
            _finite_float(self.max_thrust, "max_thrust"),
        )

        if not 0.0 <= self.min_thrust <= 1.0:
            raise ValueError("min_thrust must be within [0.0, 1.0]")
        if not 0.0 <= self.max_thrust <= 1.0:
            raise ValueError("max_thrust must be within [0.0, 1.0]")
        if self.min_thrust > self.max_thrust:
            raise ValueError("min_thrust must not exceed max_thrust")
        if not self.min_thrust <= self.equilibrium_thrust <= self.max_thrust:
            raise ValueError(
                "equilibrium_thrust must be within the thrust limits"
            )


class _PID:
    """PID channel with output-clamping conditional integration."""

    def __init__(self, config: PIDConfig) -> None:
        self._config = config
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._previous_error: float | None = None

    def update(self, error: float, dt: float) -> float:
        derivative = 0.0
        if self._previous_error is not None and self._config.kd != 0.0:
            derivative = (error - self._previous_error) / dt

        candidate_integral = self._integral
        if self._config.ki != 0.0:
            candidate_integral += error * dt

        proportional = self._config.kp * error
        derivative_term = self._config.kd * derivative
        candidate_output = (
            proportional
            + self._config.ki * candidate_integral
            + derivative_term
        )

        saturated_output = _clamp(
            candidate_output,
            self._config.output_min,
            self._config.output_max,
        )
        integral_drive = self._config.ki * error

        # Accept integration when unsaturated, or when the current error
        # would drive a saturated output back toward its valid range.
        if self._config.ki != 0.0 and (
            candidate_output == saturated_output
            or (
                candidate_output > self._config.output_max
                and integral_drive < 0.0
            )
            or (
                candidate_output < self._config.output_min
                and integral_drive > 0.0
            )
        ):
            self._integral = candidate_integral

        self._previous_error = error
        return saturated_output


class OuterLoopPIDController:
    """Four-channel position, altitude, and heading outer-loop controller."""

    def __init__(self, config: OuterLoopPIDConfig) -> None:
        self.config = config
        self._horizontal_x = _PID(config.horizontal_x)
        self._horizontal_y = _PID(config.horizontal_y)
        self._altitude = _PID(config.altitude)
        self._heading = _PID(config.heading)

    def reset(self) -> None:
        self._horizontal_x.reset()
        self._horizontal_y.reset()
        self._altitude.reset()
        self._heading.reset()

    def update(
        self,
        state: DroneState,
        reference: Reference,
        dt: float,
    ) -> BodyRateThrustCommand:
        dt = _finite_float(dt, "dt")
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        error_x_inertial = float(reference.position[0] - state.position[0])
        error_y_inertial = float(reference.position[1] - state.position[1])
        error_x_body, error_y_body = self._inertial_to_body_error(
            error_x_inertial,
            error_y_inertial,
            state.yaw,
        )

        body_x_output = self._horizontal_x.update(error_x_body, dt)
        body_y_output = self._horizontal_y.update(error_y_body, dt)
        roll_rate, pitch_rate = self._map_horizontal_outputs(
            body_x_output,
            body_y_output,
        )

        altitude_error = float(reference.position[2] - state.position[2])
        altitude_feedback = self._altitude.update(altitude_error, dt)
        thrust = _clamp(
            self.config.equilibrium_thrust + altitude_feedback,
            self.config.min_thrust,
            self.config.max_thrust,
        )

        heading_error = self._wrapped_angle(reference.yaw - state.yaw)
        yaw_rate = self._heading.update(heading_error, dt)

        return BodyRateThrustCommand(
            roll_rate=roll_rate,
            pitch_rate=pitch_rate,
            yaw_rate=yaw_rate,
            thrust=thrust,
        )

    @staticmethod
    def _inertial_to_body_error(
        error_x_inertial: float,
        error_y_inertial: float,
        yaw: float,
    ) -> tuple[float, float]:
        """Apply the planar R(-yaw) inertial-to-body rotation."""
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            cos_yaw * error_x_inertial + sin_yaw * error_y_inertial,
            -sin_yaw * error_x_inertial + cos_yaw * error_y_inertial,
        )

    def _map_horizontal_outputs(
        self,
        body_x_output: float,
        body_y_output: float,
    ) -> tuple[float, float]:
        """Map body translation channels to configured body-rate commands."""
        mapping = self.config.horizontal_mapping
        roll_rate = mapping.body_y_to_roll_rate_sign * body_y_output
        pitch_rate = mapping.body_x_to_pitch_rate_sign * body_x_output
        return roll_rate, pitch_rate

    @staticmethod
    def _wrapped_angle(angle: float) -> float:
        """Wrap an angular error to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))
