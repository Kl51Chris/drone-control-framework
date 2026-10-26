from __future__ import annotations

from dataclasses import dataclass, field, replace
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


def _positive_float(value: float, name: str) -> float:
    result = _finite_float(value, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def _sign(value: int, name: str) -> int:
    if value not in (-1, 1):
        raise ValueError(f"{name} must be either -1 or 1")
    return value


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


@dataclass(frozen=True, slots=True)
class PIDConfig:
    """Gains and output limits for one independent PID/PD channel.

    Horizontal ``kp`` and ``kd`` act on world-frame position and velocity
    errors. Cascaded horizontal output uses the shared vector limits in
    :class:`HorizontalControlConfig`; these per-channel output bounds remain
    active for legacy direct-rate mode. Altitude ``kp`` and ``kd`` act on
    altitude and vertical-velocity errors. Heading retains wrapped-error PID
    behavior.
    """

    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    output_min: float = 0.0
    output_max: float = 0.0

    def __post_init__(self) -> None:
        for name in ("kp", "ki", "kd", "output_min", "output_max"):
            object.__setattr__(self, name, _finite_float(getattr(self, name), name))
        if self.output_min > self.output_max:
            raise ValueError("output_min must not exceed output_max")


@dataclass(frozen=True, slots=True)
class HorizontalCommandMapping:
    """Legacy direct translation-output to body-rate polarities.

    This type preserves the original controller mode for existing validation
    runners. New cascaded configurations use
    :class:`HorizontalAccelerationMapping` instead.
    """

    body_x_to_pitch_rate_sign: int
    body_y_to_roll_rate_sign: int

    def __post_init__(self) -> None:
        _sign(self.body_x_to_pitch_rate_sign, "body_x_to_pitch_rate_sign")
        _sign(self.body_y_to_roll_rate_sign, "body_y_to_roll_rate_sign")


@dataclass(frozen=True, slots=True)
class HorizontalAccelerationMapping:
    """Experimentally identified body-acceleration to attitude polarities."""

    body_x_acceleration_to_pitch_sign: int = -1
    body_y_acceleration_to_roll_sign: int = -1

    def __post_init__(self) -> None:
        _sign(
            self.body_x_acceleration_to_pitch_sign,
            "body_x_acceleration_to_pitch_sign",
        )
        _sign(
            self.body_y_acceleration_to_roll_sign,
            "body_y_acceleration_to_roll_sign",
        )


@dataclass(frozen=True, slots=True)
class HorizontalControlConfig:
    """Geometry and shared limits for the horizontal cascade."""

    max_acceleration_m_s2: float = 0.4
    max_tilt_reference_rad: float = 0.08
    gravity_m_s2: float = 9.80665

    def __post_init__(self) -> None:
        for name in (
            "max_acceleration_m_s2",
            "max_tilt_reference_rad",
            "gravity_m_s2",
        ):
            object.__setattr__(self, name, _positive_float(getattr(self, name), name))
        if self.max_tilt_reference_rad >= math.pi / 2.0:
            raise ValueError("max_tilt_reference_rad must be less than pi/2")


@dataclass(frozen=True, slots=True)
class AttitudeControlConfig:
    """Roll/pitch attitude-P loop and body-rate command conventions."""

    roll_kp: float = 4.0
    pitch_kp: float = 4.0
    roll_command_sign: int = 1
    pitch_command_sign: int = -1
    max_roll_rate_rad_s: float = 0.35
    max_pitch_rate_rad_s: float = 0.35

    def __post_init__(self) -> None:
        for name in ("roll_kp", "pitch_kp"):
            value = _finite_float(getattr(self, name), name)
            if value < 0.0:
                raise ValueError(f"{name} must be nonnegative")
            object.__setattr__(self, name, value)
        for name in ("roll_command_sign", "pitch_command_sign"):
            _sign(getattr(self, name), name)
        for name in ("max_roll_rate_rad_s", "max_pitch_rate_rad_s"):
            object.__setattr__(self, name, _positive_float(getattr(self, name), name))


@dataclass(frozen=True, slots=True)
class OuterLoopPIDConfig:
    """Configuration for the cascaded XY/Z position controller."""

    horizontal_mapping: HorizontalAccelerationMapping | HorizontalCommandMapping
    horizontal_x: PIDConfig = field(default_factory=PIDConfig)
    horizontal_y: PIDConfig = field(default_factory=PIDConfig)
    altitude: PIDConfig = field(default_factory=PIDConfig)
    heading: PIDConfig = field(default_factory=PIDConfig)
    horizontal: HorizontalControlConfig = field(default_factory=HorizontalControlConfig)
    attitude: AttitudeControlConfig = field(default_factory=AttitudeControlConfig)
    altitude_velocity_reference_tracking: bool = False
    equilibrium_thrust: float = 0.0
    min_thrust: float = 0.0
    max_thrust: float = 1.0

    def __post_init__(self) -> None:
        if not isinstance(
            self.horizontal_mapping,
            (HorizontalAccelerationMapping, HorizontalCommandMapping),
        ):
            raise TypeError("horizontal_mapping has an unsupported type")
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
        if not isinstance(self.altitude_velocity_reference_tracking, bool):
            raise TypeError("altitude_velocity_reference_tracking must be bool")
        if not 0.0 <= self.min_thrust <= 1.0:
            raise ValueError("min_thrust must be within [0.0, 1.0]")
        if not 0.0 <= self.max_thrust <= 1.0:
            raise ValueError("max_thrust must be within [0.0, 1.0]")
        if self.min_thrust > self.max_thrust:
            raise ValueError("min_thrust must not exceed max_thrust")
        if not self.min_thrust <= self.equilibrium_thrust <= self.max_thrust:
            raise ValueError("equilibrium_thrust must be within the thrust limits")

    @classmethod
    def stage5_hardware_candidate(cls) -> OuterLoopPIDConfig:
        """Return hardware-tested Stage 5 gains as an experimental example."""

        return cls(
            horizontal_mapping=HorizontalAccelerationMapping(-1, -1),
            horizontal_x=PIDConfig(0.4, 0.0, 0.8, -0.4, 0.4),
            horizontal_y=PIDConfig(0.4, 0.0, 0.8, -0.4, 0.4),
            altitude=PIDConfig(0.06, 0.0, 0.06, -0.045, 0.025),
            heading=PIDConfig(2.0, 0.0, 0.0, -0.35, 0.35),
            horizontal=HorizontalControlConfig(0.4, 0.08),
            attitude=AttitudeControlConfig(),
            altitude_velocity_reference_tracking=False,
            equilibrium_thrust=0.425,
            min_thrust=0.38,
            max_thrust=0.45,
        )


class _PID:
    """PID channel with output-clamping conditional integration."""

    def __init__(self, config: PIDConfig) -> None:
        self._config = config
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._previous_error: float | None = None

    def update(
        self,
        error: float,
        dt: float,
        *,
        derivative_error: float | None = None,
        apply_output_limits: bool = True,
    ) -> float:
        derivative = derivative_error
        if derivative is None:
            derivative = 0.0
            if self._previous_error is not None and self._config.kd != 0.0:
                derivative = (error - self._previous_error) / dt

        candidate_integral = self._integral
        if self._config.ki != 0.0:
            candidate_integral += error * dt
        candidate_output = (
            self._config.kp * error
            + self._config.ki * candidate_integral
            + self._config.kd * derivative
        )
        saturated_output = (
            _clamp(
                candidate_output,
                self._config.output_min,
                self._config.output_max,
            )
            if apply_output_limits
            else candidate_output
        )
        integral_drive = self._config.ki * error
        if self._config.ki != 0.0 and (
            candidate_output == saturated_output
            or (candidate_output > self._config.output_max and integral_drive < 0.0)
            or (candidate_output < self._config.output_min and integral_drive > 0.0)
        ):
            self._integral = candidate_integral
        self._previous_error = error
        return saturated_output


class OuterLoopPIDController:
    """Cascaded XY/Z position, attitude, and heading controller.

    Outputs remain hardware-independent body rates in rad/s and normalized
    thrust. Firmware body-rate control and motor mixing remain downstream.
    """

    def __init__(self, config: OuterLoopPIDConfig) -> None:
        self.config = config
        self._horizontal_x = _PID(config.horizontal_x)
        self._horizontal_y = _PID(config.horizontal_y)
        altitude_min = max(
            config.altitude.output_min,
            config.min_thrust - config.equilibrium_thrust,
        )
        altitude_max = min(
            config.altitude.output_max,
            config.max_thrust - config.equilibrium_thrust,
        )
        if altitude_min > altitude_max:
            raise ValueError(
                "altitude output limits do not overlap available thrust authority"
            )
        altitude_config = replace(
            config.altitude,
            output_min=altitude_min,
            output_max=altitude_max,
        )
        self._altitude = _PID(altitude_config)
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

        if isinstance(self.config.horizontal_mapping, HorizontalCommandMapping):
            roll_rate, pitch_rate = self._legacy_horizontal_rates(state, reference, dt)
        else:
            acceleration_x_world = self._horizontal_x.update(
                float(reference.position[0] - state.position[0]),
                dt,
                derivative_error=float(reference.velocity[0] - state.velocity[0]),
                apply_output_limits=False,
            )
            acceleration_y_world = self._horizontal_y.update(
                float(reference.position[1] - state.position[1]),
                dt,
                derivative_error=float(reference.velocity[1] - state.velocity[1]),
                apply_output_limits=False,
            )
            acceleration_x_world, acceleration_y_world = self._limit_vector(
                acceleration_x_world,
                acceleration_y_world,
                self.config.horizontal.max_acceleration_m_s2,
            )
            acceleration_x_body, acceleration_y_body = self._inertial_to_body_error(
                acceleration_x_world,
                acceleration_y_world,
                state.yaw,
            )
            desired_roll, desired_pitch = self._desired_attitude(
                acceleration_x_body,
                acceleration_y_body,
            )
            roll_rate, pitch_rate = self._attitude_rates(
                desired_roll,
                desired_pitch,
                state,
            )

        desired_vz = (
            float(reference.velocity[2])
            if self.config.altitude_velocity_reference_tracking
            else 0.0
        )
        altitude_feedback = self._altitude.update(
            float(reference.position[2] - state.position[2]),
            dt,
            derivative_error=desired_vz - float(state.velocity[2]),
        )
        thrust = _clamp(
            self.config.equilibrium_thrust + altitude_feedback,
            self.config.min_thrust,
            self.config.max_thrust,
        )
        yaw_rate = self._heading.update(
            self._wrapped_angle(reference.yaw - state.yaw),
            dt,
        )
        return BodyRateThrustCommand(roll_rate, pitch_rate, yaw_rate, thrust)

    @staticmethod
    def _inertial_to_body_error(
        x_world: float,
        y_world: float,
        yaw: float,
    ) -> tuple[float, float]:
        """Apply the planar R(-yaw) world-to-body rotation."""

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            cos_yaw * x_world + sin_yaw * y_world,
            -sin_yaw * x_world + cos_yaw * y_world,
        )

    def _desired_attitude(
        self,
        acceleration_x_body: float,
        acceleration_y_body: float,
    ) -> tuple[float, float]:
        mapping = self.config.horizontal_mapping
        if not isinstance(mapping, HorizontalAccelerationMapping):
            raise TypeError(
                "cascaded attitude mapping requires HorizontalAccelerationMapping"
            )
        gravity = self.config.horizontal.gravity_m_s2
        desired_pitch = (
            mapping.body_x_acceleration_to_pitch_sign
            * math.atan2(acceleration_x_body, gravity)
        )
        desired_roll = (
            mapping.body_y_acceleration_to_roll_sign
            * math.atan2(acceleration_y_body, gravity)
        )
        return self._limit_vector(
            desired_roll,
            desired_pitch,
            self.config.horizontal.max_tilt_reference_rad,
        )

    def _legacy_horizontal_rates(
        self,
        state: DroneState,
        reference: Reference,
        dt: float,
    ) -> tuple[float, float]:
        """Preserve the former direct-rate behavior for existing runners."""

        error_x_body, error_y_body = self._inertial_to_body_error(
            float(reference.position[0] - state.position[0]),
            float(reference.position[1] - state.position[1]),
            state.yaw,
        )
        body_x_output = self._horizontal_x.update(error_x_body, dt)
        body_y_output = self._horizontal_y.update(error_y_body, dt)
        mapping = self.config.horizontal_mapping
        assert isinstance(mapping, HorizontalCommandMapping)
        return (
            mapping.body_y_to_roll_rate_sign * body_y_output,
            mapping.body_x_to_pitch_rate_sign * body_x_output,
        )

    def _attitude_rates(
        self,
        desired_roll: float,
        desired_pitch: float,
        state: DroneState,
    ) -> tuple[float, float]:
        config = self.config.attitude
        roll_rate = (
            config.roll_command_sign * config.roll_kp * (desired_roll - state.roll)
        )
        pitch_rate = (
            config.pitch_command_sign
            * config.pitch_kp
            * (desired_pitch - state.pitch)
        )
        return (
            _clamp(roll_rate, -config.max_roll_rate_rad_s, config.max_roll_rate_rad_s),
            _clamp(
                pitch_rate,
                -config.max_pitch_rate_rad_s,
                config.max_pitch_rate_rad_s,
            ),
        )

    @staticmethod
    def _limit_vector(x: float, y: float, limit: float) -> tuple[float, float]:
        magnitude = math.hypot(x, y)
        if magnitude <= limit or magnitude == 0.0:
            return x, y
        scale = limit / magnitude
        return x * scale, y * scale

    @staticmethod
    def _wrapped_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))
