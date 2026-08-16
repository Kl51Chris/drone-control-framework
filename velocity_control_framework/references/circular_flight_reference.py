from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import math

import numpy as np
from numpy.typing import ArrayLike

from velocity_control_framework.interfaces import Reference


class CircularFlightState(Enum):
    """States used by the circular-flight reference."""

    ARMED_IDLE = auto()
    TAKEOFF = auto()
    CIRCLE = auto()
    LANDING = auto()
    COMPLETE = auto()


@dataclass(slots=True)
class CircularFlightConfig:
    """
    Configuration for one takeoff-circle-landing experiment.

    takeoff_height:
        Height gained relative to the measured initial altitude.

    circle_radius:
        Radius of the horizontal circular trajectory in meters.

    circle_revolutions:
        Number of complete circles.

    circle_duration:
        Total time used to complete the circular trajectory.
    """

    takeoff_height: float = 0.30

    circle_radius: float = 0.25
    circle_revolutions: float = 1.0

    armed_idle_duration: float = 0.50
    takeoff_duration: float = 4.00
    circle_duration: float = 15.00
    landing_duration: float = 4.00

    def __post_init__(self) -> None:
        positive_values = {
            "takeoff_height": self.takeoff_height,
            "circle_radius": self.circle_radius,
            "circle_revolutions": self.circle_revolutions,
            "armed_idle_duration": self.armed_idle_duration,
            "takeoff_duration": self.takeoff_duration,
            "circle_duration": self.circle_duration,
            "landing_duration": self.landing_duration,
        }

        for name, value in positive_values.items():
            converted = float(value)

            if not math.isfinite(converted):
                raise ValueError(f"{name} must be finite")

            if converted <= 0.0:
                raise ValueError(f"{name} must be positive")

            setattr(self, name, converted)


class CircularFlightReference:
    """
    Generate references for idle, takeoff, circular motion, and landing.

    The circular phase uses a cubic smoothstep angular trajectory.
    Angular velocity is zero at the start and end of the circle,
    avoiding an instantaneous horizontal-velocity step.
    """

    def __init__(
        self,
        initial_position: ArrayLike,
        initial_yaw: float,
        start_time: float,
        config: CircularFlightConfig | None = None,
    ) -> None:
        self.config = config or CircularFlightConfig()

        position = np.asarray(
            initial_position,
            dtype=np.float64,
        )

        if position.shape != (3,):
            raise ValueError(
                "initial_position must have shape (3,)"
            )

        if not np.all(np.isfinite(position)):
            raise ValueError(
                "initial_position must contain only finite values"
            )

        self._initial_position = position.copy()
        self._initial_yaw = float(initial_yaw)
        self._start_time = float(start_time)

        if not math.isfinite(self._initial_yaw):
            raise ValueError("initial_yaw must be finite")

        if not math.isfinite(self._start_time):
            raise ValueError("start_time must be finite")

        self._hover_altitude = (
            float(self._initial_position[2])
            + self.config.takeoff_height
        )

        # Choose the center so theta = 0 starts exactly at the
        # measured initial x/y position.
        self._circle_center = np.array(
            [
                float(self._initial_position[0])
                - self.config.circle_radius,
                float(self._initial_position[1]),
            ],
            dtype=np.float64,
        )

    @property
    def initial_altitude(self) -> float:
        return float(self._initial_position[2])

    @property
    def hover_altitude(self) -> float:
        return self._hover_altitude

    @property
    def circle_center(self) -> np.ndarray:
        return self._circle_center.copy()

    @property
    def total_duration(self) -> float:
        return (
            self.config.armed_idle_duration
            + self.config.takeoff_duration
            + self.config.circle_duration
            + self.config.landing_duration
        )

    def get_state(
        self,
        current_time: float,
    ) -> CircularFlightState:
        """Return the current flight state."""
        elapsed = self._elapsed(current_time)

        idle_end = self.config.armed_idle_duration

        takeoff_end = (
            idle_end
            + self.config.takeoff_duration
        )

        circle_end = (
            takeoff_end
            + self.config.circle_duration
        )

        landing_end = (
            circle_end
            + self.config.landing_duration
        )

        if elapsed < idle_end:
            return CircularFlightState.ARMED_IDLE

        if elapsed < takeoff_end:
            return CircularFlightState.TAKEOFF

        if elapsed < circle_end:
            return CircularFlightState.CIRCLE

        if elapsed < landing_end:
            return CircularFlightState.LANDING

        return CircularFlightState.COMPLETE

    def get_reference(
        self,
        current_time: float,
    ) -> Reference:
        """Return the position and velocity reference for this time."""
        elapsed = self._elapsed(current_time)
        flight_state = self.get_state(current_time)

        x_ref = float(self._initial_position[0])
        y_ref = float(self._initial_position[1])
        z_ref = self.initial_altitude

        vx_ref = 0.0
        vy_ref = 0.0
        vz_ref = 0.0

        if flight_state is CircularFlightState.ARMED_IDLE:
            pass

        elif flight_state is CircularFlightState.TAKEOFF:
            phase_time = (
                elapsed
                - self.config.armed_idle_duration
            )

            z_ref, vz_ref = self._smooth_segment(
                start_value=self.initial_altitude,
                end_value=self.hover_altitude,
                phase_time=phase_time,
                duration=self.config.takeoff_duration,
            )

        elif flight_state is CircularFlightState.CIRCLE:
            phase_start = (
                self.config.armed_idle_duration
                + self.config.takeoff_duration
            )

            phase_time = elapsed - phase_start

            (
                x_ref,
                y_ref,
                vx_ref,
                vy_ref,
            ) = self._circle_reference(
                phase_time=phase_time,
            )

            z_ref = self.hover_altitude

        elif flight_state is CircularFlightState.LANDING:
            phase_start = (
                self.config.armed_idle_duration
                + self.config.takeoff_duration
                + self.config.circle_duration
            )

            phase_time = elapsed - phase_start

            # The smooth circular trajectory completes at its starting
            # x/y position, so landing begins from the original x/y.
            x_ref = float(self._initial_position[0])
            y_ref = float(self._initial_position[1])

            z_ref, vz_ref = self._smooth_segment(
                start_value=self.hover_altitude,
                end_value=self.initial_altitude,
                phase_time=phase_time,
                duration=self.config.landing_duration,
            )

        return Reference(
            position=np.array(
                [x_ref, y_ref, z_ref],
                dtype=np.float64,
            ),
            velocity=np.array(
                [vx_ref, vy_ref, vz_ref],
                dtype=np.float64,
            ),
            yaw=self._initial_yaw,
            yaw_rate=0.0,
            timestamp=elapsed,
        )

    def _circle_reference(
        self,
        phase_time: float,
    ) -> tuple[float, float, float, float]:
        """
        Evaluate a smooth circular position and velocity reference.

        The angular position follows a cubic smoothstep, so angular
        velocity starts and ends at zero.
        """
        duration = self.config.circle_duration

        s = min(
            max(phase_time / duration, 0.0),
            1.0,
        )

        smooth_position = (
            3.0 * s**2
            - 2.0 * s**3
        )

        smooth_velocity = (
            6.0 * s
            - 6.0 * s**2
        )

        total_angle = (
            2.0
            * math.pi
            * self.config.circle_revolutions
        )

        theta = total_angle * smooth_position

        theta_rate = (
            total_angle
            / duration
            * smooth_velocity
        )

        radius = self.config.circle_radius

        x_ref = (
            float(self._circle_center[0])
            + radius * math.cos(theta)
        )

        y_ref = (
            float(self._circle_center[1])
            + radius * math.sin(theta)
        )

        vx_ref = (
            -radius
            * math.sin(theta)
            * theta_rate
        )

        vy_ref = (
            radius
            * math.cos(theta)
            * theta_rate
        )

        if s >= 1.0:
            vx_ref = 0.0
            vy_ref = 0.0

        return x_ref, y_ref, vx_ref, vy_ref

    def _elapsed(
        self,
        current_time: float,
    ) -> float:
        current_time = float(current_time)

        if not math.isfinite(current_time):
            raise ValueError("current_time must be finite")

        return max(
            0.0,
            current_time - self._start_time,
        )

    @staticmethod
    def _smooth_segment(
        start_value: float,
        end_value: float,
        phase_time: float,
        duration: float,
    ) -> tuple[float, float]:
        """Evaluate cubic smoothstep position and velocity."""
        s = min(
            max(phase_time / duration, 0.0),
            1.0,
        )

        position_scale = (
            3.0 * s**2
            - 2.0 * s**3
        )

        velocity_scale = (
            6.0 * s
            - 6.0 * s**2
        )

        value_change = end_value - start_value

        value = (
            start_value
            + value_change * position_scale
        )

        value_rate = (
            value_change
            / duration
            * velocity_scale
        )

        if s >= 1.0:
            value_rate = 0.0

        return value, value_rate