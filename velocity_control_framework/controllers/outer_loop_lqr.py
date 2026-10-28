"""D-FaLL-inspired horizontal LQR with independent altitude/yaw PID.

The horizontal models use measured-minus-reference body-frame errors.  In the
repository's measured conventions, body-X and body-Y acceleration are both
approximately ``-g`` times pitch and roll, respectively.  A positive pitch
rate packet produces decreasing measured pitch, while roll-rate packets and
measured roll have the same sign.  Consequently::

    A_x = [[0, 1, 0], [0, 0, -g], [0, 0, 0]], B_x = [[0], [0], [-1]]
    A_y = [[0, 1, 0], [0, 0, -g], [0, 0, 0]], B_y = [[0], [0], [ 1]]

The control law is ``u = -K e``.  The D-FaLL example uses the full state order
``[x,y,z,xdot,ydot,zdot,roll,pitch,yaw]``, angular-rate inputs, continuous CARE,
and identity Q/R.  This controller uses the corresponding three-state axis
subproblems; its different signs are local hardware convention adaptations.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math

import numpy as np
from scipy.linalg import solve_continuous_are

from velocity_control_framework.interfaces import (
    BodyRateThrustCommand,
    DroneState,
    Reference,
)


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


@dataclass(frozen=True, slots=True)
class LQRPIDConfig:
    """Independent PID channel parameters owned by the LQR controller."""

    kp: float
    ki: float
    kd: float
    output_min: float
    output_max: float

    def __post_init__(self) -> None:
        for name in ("kp", "ki", "kd", "output_min", "output_max"):
            object.__setattr__(self, name, _finite(getattr(self, name), name))
        if self.output_min > self.output_max:
            raise ValueError("output_min must not exceed output_max")


@dataclass(frozen=True, slots=True)
class HorizontalLQRAxisConfig:
    """Continuous three-state model and quadratic cost for one axis."""

    acceleration_per_attitude_m_s2_rad: float
    attitude_rate_per_command: float
    q_diagonal: tuple[float, float, float] = (1.0, 1.0, 1.0)
    r: float = 1.0
    gain: tuple[float, float, float] = field(init=False)
    closed_loop_eigenvalues: tuple[tuple[float, float], ...] = field(init=False)

    def __post_init__(self) -> None:
        acceleration = _finite(
            self.acceleration_per_attitude_m_s2_rad,
            "acceleration_per_attitude_m_s2_rad",
        )
        rate_sign = _finite(
            self.attitude_rate_per_command,
            "attitude_rate_per_command",
        )
        if acceleration == 0.0 or rate_sign == 0.0:
            raise ValueError(
                "model acceleration and attitude-rate coefficients must be nonzero"
            )
        q = np.asarray(self.q_diagonal, dtype=np.float64)
        r = _finite(self.r, "r")
        if q.shape != (3,) or not np.all(np.isfinite(q)) or np.any(q <= 0.0):
            raise ValueError("q_diagonal must contain three finite positive values")
        if r <= 0.0:
            raise ValueError("r must be positive")
        a = np.array(
            [[0.0, 1.0, 0.0], [0.0, 0.0, acceleration], [0.0, 0.0, 0.0]]
        )
        b = np.array([[0.0], [0.0], [rate_sign]])
        controllability = np.hstack((b, a @ b, a @ a @ b))
        if np.linalg.matrix_rank(controllability) != 3:
            raise ValueError("horizontal model is not controllable")
        p = solve_continuous_are(a, b, np.diag(q), np.array([[r]]))
        gain_array = (b.T @ p / r).reshape(3)
        eigenvalues = np.linalg.eigvals(a - b @ gain_array.reshape(1, 3))
        if not np.all(np.isfinite(gain_array)) or not np.all(np.isfinite(eigenvalues)):
            raise ValueError("LQR synthesis produced non-finite values")
        if np.any(np.real(eigenvalues) >= 0.0):
            raise ValueError("LQR closed-loop model is not asymptotically stable")
        object.__setattr__(self, "gain", tuple(float(value) for value in gain_array))
        object.__setattr__(
            self,
            "closed_loop_eigenvalues",
            tuple((float(value.real), float(value.imag)) for value in eigenvalues),
        )


@dataclass(frozen=True, slots=True)
class OuterLoopLQRConfig:
    """Complete, independent parameter set for :class:`OuterLoopLQRController`."""

    x_axis: HorizontalLQRAxisConfig
    y_axis: HorizontalLQRAxisConfig
    altitude: LQRPIDConfig
    heading: LQRPIDConfig
    altitude_velocity_reference_tracking: bool = False
    equilibrium_thrust: float = 0.425
    min_thrust: float = 0.38
    max_thrust: float = 0.45
    max_roll_rate_rad_s: float = 0.35
    max_pitch_rate_rad_s: float = 0.35
    model_provenance: str = ""
    parameter_provenance: str = ""
    flight_validated: bool = False

    def __post_init__(self) -> None:
        for name in (
            "equilibrium_thrust",
            "min_thrust",
            "max_thrust",
            "max_roll_rate_rad_s",
            "max_pitch_rate_rad_s",
        ):
            object.__setattr__(self, name, _finite(getattr(self, name), name))
        if not (
            0.0
            <= self.min_thrust
            <= self.equilibrium_thrust
            <= self.max_thrust
            <= 1.0
        ):
            raise ValueError("thrust limits must contain equilibrium thrust within [0, 1]")
        if self.max_roll_rate_rad_s <= 0.0 or self.max_pitch_rate_rad_s <= 0.0:
            raise ValueError("horizontal body-rate limits must be positive")
        if not isinstance(self.altitude_velocity_reference_tracking, bool):
            raise TypeError("altitude_velocity_reference_tracking must be bool")

    @classmethod
    def dfall_initial(cls) -> "OuterLoopLQRConfig":
        gravity = 9.81
        return cls(
            x_axis=HorizontalLQRAxisConfig(-gravity, -1.0),
            y_axis=HorizontalLQRAxisConfig(-gravity, 1.0),
            altitude=LQRPIDConfig(0.06, 0.0, 0.06, -0.045, 0.025),
            heading=LQRPIDConfig(2.0, 0.0, 0.0, -0.35, 0.35),
            model_provenance=(
                "ETH D-FaLL exercise04 LQR example, continuous angular-rate-input "
                "hover model; acceleration and command signs adapted to repository "
                "hardware identification"
            ),
            parameter_provenance=(
                "Horizontal Q=diag(1,1,1), R=1 are the corresponding axis sub-costs "
                "of the official example's identity Q/R; Z/yaw and limits are copied "
                "into this independent config from stage5-hardware-candidate"
            ),
        )


class _IndependentPID:
    """PID with the same conditional-integration behavior as the PID controller."""

    def __init__(self, config: LQRPIDConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error: float | None = None

    def update(
        self,
        error: float,
        dt: float,
        *,
        derivative_error: float | None = None,
    ) -> float:
        derivative = derivative_error
        if derivative is None:
            derivative = 0.0
            if self.previous_error is not None and self.config.kd != 0.0:
                derivative = (error - self.previous_error) / dt
        candidate_integral = (
            self.integral + error * dt
            if self.config.ki != 0.0
            else self.integral
        )
        candidate = (
            self.config.kp * error
            + self.config.ki * candidate_integral
            + self.config.kd * derivative
        )
        saturated = _clamp(candidate, self.config.output_min, self.config.output_max)
        integral_drive = self.config.ki * error
        if self.config.ki != 0.0 and (
            candidate == saturated
            or (candidate > self.config.output_max and integral_drive < 0.0)
            or (candidate < self.config.output_min and integral_drive > 0.0)
        ):
            self.integral = candidate_integral
        self.previous_error = error
        return saturated


class OuterLoopLQRController:
    """Horizontal LQR plus independent altitude and wrapped-yaw PID channels."""

    def __init__(self, config: OuterLoopLQRConfig) -> None:
        self.config = config
        altitude_min = max(
            config.altitude.output_min,
            config.min_thrust - config.equilibrium_thrust,
        )
        altitude_max = min(
            config.altitude.output_max,
            config.max_thrust - config.equilibrium_thrust,
        )
        if altitude_min > altitude_max:
            raise ValueError("altitude output limits do not overlap available thrust authority")
        self._altitude = _IndependentPID(
            replace(
                config.altitude,
                output_min=altitude_min,
                output_max=altitude_max,
            )
        )
        self._heading = _IndependentPID(config.heading)

    def reset(self) -> None:
        self._altitude.reset()
        self._heading.reset()

    def update(
        self,
        state: DroneState,
        reference: Reference,
        dt: float,
    ) -> BodyRateThrustCommand:
        dt = _finite(dt, "dt")
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        position_error = self._world_to_body(
            float(state.position[0] - reference.position[0]),
            float(state.position[1] - reference.position[1]),
            state.yaw,
        )
        velocity_error = self._world_to_body(
            float(state.velocity[0] - reference.velocity[0]),
            float(state.velocity[1] - reference.velocity[1]),
            state.yaw,
        )
        pitch_rate = -float(
            np.dot(
                self.config.x_axis.gain,
                (position_error[0], velocity_error[0], state.pitch),
            )
        )
        roll_rate = -float(
            np.dot(
                self.config.y_axis.gain,
                (position_error[1], velocity_error[1], state.roll),
            )
        )
        pitch_rate = _clamp(
            pitch_rate,
            -self.config.max_pitch_rate_rad_s,
            self.config.max_pitch_rate_rad_s,
        )
        roll_rate = _clamp(
            roll_rate,
            -self.config.max_roll_rate_rad_s,
            self.config.max_roll_rate_rad_s,
        )

        desired_vz = (
            float(reference.velocity[2])
            if self.config.altitude_velocity_reference_tracking
            else 0.0
        )
        altitude_feedback = self._altitude.update(
            float(reference.position[2] - state.position[2]), dt,
            derivative_error=desired_vz - float(state.velocity[2]),
        )
        thrust = _clamp(
            self.config.equilibrium_thrust + altitude_feedback,
            self.config.min_thrust,
            self.config.max_thrust,
        )
        yaw_delta = reference.yaw - state.yaw
        yaw_error = math.atan2(math.sin(yaw_delta), math.cos(yaw_delta))
        yaw_rate = self._heading.update(yaw_error, dt)
        return BodyRateThrustCommand(roll_rate, pitch_rate, yaw_rate, thrust)

    @staticmethod
    def _world_to_body(
        x_world: float,
        y_world: float,
        yaw: float,
    ) -> tuple[float, float]:
        cosine, sine = math.cos(yaw), math.sin(yaw)
        return cosine * x_world + sine * y_world, -sine * x_world + cosine * y_world
