from __future__ import annotations

from dataclasses import dataclass
import math

from velocity_control_framework.interfaces import (
    AttitudeThrustCommand,
    DroneState,
    Reference,
)


UINT16_MAX = 65535.0
STANDARD_GRAVITY = 9.80665


def _finite_float(value: float, name: str) -> float:
    result = float(value)

    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")

    return result


def _nonnegative_finite(
    value: float,
    name: str,
) -> float:
    result = _finite_float(value, name)

    if result < 0.0:
        raise ValueError(f"{name} must be nonnegative")

    return result


def _positive_finite(
    value: float,
    name: str,
) -> float:
    result = _finite_float(value, name)

    if result <= 0.0:
        raise ValueError(f"{name} must be positive")

    return result


def _clamp(
    value: float,
    lower: float,
    upper: float,
) -> float:
    return max(lower, min(value, upper))


@dataclass(slots=True)
class VelocityAltitudePIDConfig:
    """
    Configuration for horizontal velocity and altitude control.

    Horizontal gains:
        kp_vx, kp_vy:
            Velocity-error to acceleration gain [1/s].

    Vertical gains:
        kp_z:
            Position-error to acceleration gain [1/s^2].

        ki_z:
            Integrated-position-error to acceleration gain [1/s^3].

        kd_z:
            Velocity-error to acceleration gain [1/s].
    """

    kp_vx: float = 1.0
    kp_vy: float = 1.0

    kp_z: float = 0.8
    ki_z: float = 0.0
    kd_z: float = 0.25

    hover_command: int = 31500

    max_tilt: float = math.radians(8.0)

    min_thrust: float = 0.0
    max_thrust: float = 0.62

    integral_limit_z: float = 0.50
    gravity: float = STANDARD_GRAVITY

    def __post_init__(self) -> None:
        self.kp_vx = _nonnegative_finite(
            self.kp_vx,
            "kp_vx",
        )
        self.kp_vy = _nonnegative_finite(
            self.kp_vy,
            "kp_vy",
        )

        self.kp_z = _nonnegative_finite(
            self.kp_z,
            "kp_z",
        )
        self.ki_z = _nonnegative_finite(
            self.ki_z,
            "ki_z",
        )
        self.kd_z = _nonnegative_finite(
            self.kd_z,
            "kd_z",
        )

        self.hover_command = int(self.hover_command)

        if not 0 <= self.hover_command <= 65535:
            raise ValueError(
                "hover_command must be within [0, 65535]"
            )

        self.max_tilt = _positive_finite(
            self.max_tilt,
            "max_tilt",
        )

        if self.max_tilt >= math.pi / 2.0:
            raise ValueError(
                "max_tilt must be less than pi/2"
            )

        self.min_thrust = _finite_float(
            self.min_thrust,
            "min_thrust",
        )
        self.max_thrust = _finite_float(
            self.max_thrust,
            "max_thrust",
        )

        if not 0.0 <= self.min_thrust <= 1.0:
            raise ValueError(
                "min_thrust must be within [0, 1]"
            )

        if not 0.0 <= self.max_thrust <= 1.0:
            raise ValueError(
                "max_thrust must be within [0, 1]"
            )

        if self.min_thrust > self.max_thrust:
            raise ValueError(
                "min_thrust must not exceed max_thrust"
            )

        self.integral_limit_z = _nonnegative_finite(
            self.integral_limit_z,
            "integral_limit_z",
        )

        self.gravity = _positive_finite(
            self.gravity,
            "gravity",
        )

    @property
    def hover_thrust(self) -> float:
        return self.hover_command / UINT16_MAX


class VelocityAltitudePIDController:
    """
    Control horizontal velocity and vertical altitude.

    Public state velocities are assumed to be world-frame ENU.

    The controller:
        - tracks vx and vy through roll/pitch
        - tracks z and vz through collective thrust
        - passes reference.yaw_rate to the firmware
    """

    def __init__(
        self,
        config: VelocityAltitudePIDConfig | None = None,
    ) -> None:
        self.config = (
            config
            or VelocityAltitudePIDConfig()
        )

        self._integral_error_z = 0.0

    def reset(self) -> None:
        self._integral_error_z = 0.0

    def update(
        self,
        state: DroneState,
        reference: Reference,
        dt: float,
    ) -> AttitudeThrustCommand:
        dt = _positive_finite(dt, "dt")

        roll, pitch = self._horizontal_control(
            state=state,
            reference=reference,
        )

        thrust = self._vertical_control(
            state=state,
            reference=reference,
            dt=dt,
        )

        return AttitudeThrustCommand(
            roll=roll,
            pitch=pitch,
            yaw_rate=reference.yaw_rate,
            thrust=thrust,
        )

    def _horizontal_control(
        self,
        state: DroneState,
        reference: Reference,
    ) -> tuple[float, float]:
        """
        Convert world-frame horizontal velocity error into
        Crazyflie roll and pitch commands.
        """
        vx_error_world = (
            float(reference.velocity[0])
            - float(state.velocity[0])
        )
        vy_error_world = (
            float(reference.velocity[1])
            - float(state.velocity[1])
        )

        ax_world = (
            self.config.kp_vx
            * vx_error_world
        )
        ay_world = (
            self.config.kp_vy
            * vy_error_world
        )

        # Rotate the desired world-frame acceleration into the
        # body-yaw-aligned frame.
        yaw = float(state.yaw)

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        ax_body = (
            cos_yaw * ax_world
            + sin_yaw * ay_world
        )

        ay_body = (
            -sin_yaw * ax_world
            + cos_yaw * ay_world
        )

        # Crazyflie attitude convention:
        # negative pitch produces forward acceleration;
        # negative roll produces leftward acceleration.
        pitch_command = math.atan2(
            ax_body,
            self.config.gravity,
        )

        roll_command = math.atan2(
            -ay_body,
            self.config.gravity,
        )

        pitch_command = _clamp(
            pitch_command,
            -self.config.max_tilt,
            self.config.max_tilt,
        )

        roll_command = _clamp(
            roll_command,
            -self.config.max_tilt,
            self.config.max_tilt,
        )

        return roll_command, pitch_command

    def _vertical_control(
        self,
        state: DroneState,
        reference: Reference,
        dt: float,
    ) -> float:
        z = float(state.position[2])
        vz = float(state.velocity[2])

        z_ref = float(reference.position[2])
        vz_ref = float(reference.velocity[2])

        position_error = z_ref - z
        velocity_error = vz_ref - vz

        candidate_integral = (
            self._integral_error_z
            + position_error * dt
        )

        candidate_integral = _clamp(
            candidate_integral,
            -self.config.integral_limit_z,
            self.config.integral_limit_z,
        )

        acceleration_command = (
            self.config.kp_z * position_error
            + self.config.ki_z * candidate_integral
            + self.config.kd_z * velocity_error
        )

        raw_thrust = (
            self.config.hover_thrust
            * (
                1.0
                + acceleration_command
                / self.config.gravity
            )
        )

        limited_thrust = _clamp(
            raw_thrust,
            self.config.min_thrust,
            self.config.max_thrust,
        )

        if self._should_integrate_z(
            raw_thrust=raw_thrust,
            position_error=position_error,
        ):
            self._integral_error_z = (
                candidate_integral
            )

        return limited_thrust

    def _should_integrate_z(
        self,
        raw_thrust: float,
        position_error: float,
    ) -> bool:
        if self.config.ki_z == 0.0:
            return False

        if (
            self.config.min_thrust
            < raw_thrust
            < self.config.max_thrust
        ):
            return True

        if (
            raw_thrust >= self.config.max_thrust
            and position_error < 0.0
        ):
            return True

        if (
            raw_thrust <= self.config.min_thrust
            and position_error > 0.0
        ):
            return True

        return False

    @property
    def integral_error_z(self) -> float:
        return self._integral_error_z