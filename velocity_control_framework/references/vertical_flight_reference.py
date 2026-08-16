from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import math

import numpy as np

from velocity_control_framework.interfaces import Reference


class FlightState(Enum):
    """States used by the vertical-flight runner."""

    ARMED_IDLE = auto()
    TAKEOFF = auto()
    HOVER = auto()
    LANDING = auto()
    COMPLETE = auto()


@dataclass(slots=True)
class VerticalFlightConfig:
    """
    Timing and altitude configuration for one vertical flight.

    takeoff_height:
        Height gained relative to the measured initial altitude.

    armed_idle_duration:
        Time spent sending zero-thrust setpoints after arming.

    takeoff_duration:
        Duration of the smooth ascent.

    hover_duration:
        Duration at the target altitude.

    landing_duration:
        Duration of the smooth descent.
    """

    takeoff_height: float = 0.20

    armed_idle_duration: float = 0.50
    takeoff_duration: float = 3.00
    hover_duration: float = 3.00
    landing_duration: float = 3.00

    def __post_init__(self) -> None:
        values = {
            "takeoff_height": self.takeoff_height,
            "armed_idle_duration": self.armed_idle_duration,
            "takeoff_duration": self.takeoff_duration,
            "hover_duration": self.hover_duration,
            "landing_duration": self.landing_duration,
        }

        for name, value in values.items():
            value = float(value)

            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")

            if value <= 0.0:
                raise ValueError(f"{name} must be positive")

            setattr(self, name, value)


class VerticalFlightReference:
    """
    Generate references for idle, takeoff, hover, and landing.

    Takeoff and landing use cubic smoothstep trajectories. Position
    and velocity references are continuous at all phase boundaries.
    """

    def __init__(
        self,
        initial_position: np.ndarray,
        initial_yaw: float,
        start_time: float,
        config: VerticalFlightConfig | None = None,
    ) -> None:
        self.config = config or VerticalFlightConfig()

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
                "initial_position must contain finite values"
            )

        self._initial_position = position.copy()
        self._initial_yaw = float(initial_yaw)
        self._start_time = float(start_time)

        if not math.isfinite(self._initial_yaw):
            raise ValueError("initial_yaw must be finite")

        if not math.isfinite(self._start_time):
            raise ValueError("start_time must be finite")

        self._target_altitude = (
            float(self._initial_position[2])
            + self.config.takeoff_height
        )

    @property
    def initial_altitude(self) -> float:
        return float(self._initial_position[2])

    @property
    def target_altitude(self) -> float:
        return self._target_altitude

    @property
    def total_duration(self) -> float:
        return (
            self.config.armed_idle_duration
            + self.config.takeoff_duration
            + self.config.hover_duration
            + self.config.landing_duration
        )

    def get_state(
        self,
        current_time: float,
    ) -> FlightState:
        """Return the active flight state."""
        elapsed = self._elapsed(current_time)

        idle_end = self.config.armed_idle_duration

        takeoff_end = (
            idle_end
            + self.config.takeoff_duration
        )

        hover_end = (
            takeoff_end
            + self.config.hover_duration
        )

        landing_end = (
            hover_end
            + self.config.landing_duration
        )

        if elapsed < idle_end:
            return FlightState.ARMED_IDLE

        if elapsed < takeoff_end:
            return FlightState.TAKEOFF

        if elapsed < hover_end:
            return FlightState.HOVER

        if elapsed < landing_end:
            return FlightState.LANDING

        return FlightState.COMPLETE

    def get_reference(
        self,
        current_time: float,
    ) -> Reference:
        """Return the reference corresponding to the current state."""
        elapsed = self._elapsed(current_time)
        state = self.get_state(current_time)

        if state is FlightState.ARMED_IDLE:
            z_ref = self.initial_altitude
            vz_ref = 0.0

        elif state is FlightState.TAKEOFF:
            phase_time = (
                elapsed
                - self.config.armed_idle_duration
            )

            z_ref, vz_ref = self._smooth_vertical_segment(
                start_altitude=self.initial_altitude,
                end_altitude=self.target_altitude,
                phase_time=phase_time,
                duration=self.config.takeoff_duration,
            )

        elif state is FlightState.HOVER:
            z_ref = self.target_altitude
            vz_ref = 0.0

        elif state is FlightState.LANDING:
            phase_start = (
                self.config.armed_idle_duration
                + self.config.takeoff_duration
                + self.config.hover_duration
            )

            phase_time = elapsed - phase_start

            z_ref, vz_ref = self._smooth_vertical_segment(
                start_altitude=self.target_altitude,
                end_altitude=self.initial_altitude,
                phase_time=phase_time,
                duration=self.config.landing_duration,
            )

        else:
            z_ref = self.initial_altitude
            vz_ref = 0.0

        return Reference(
            position=np.array(
                [
                    self._initial_position[0],
                    self._initial_position[1],
                    z_ref,
                ],
                dtype=np.float64,
            ),
            velocity=np.array(
                [0.0, 0.0, vz_ref],
                dtype=np.float64,
            ),
            yaw=self._initial_yaw,
            yaw_rate=0.0,
            timestamp=elapsed,
        )

    def _elapsed(
        self,
        current_time: float,
    ) -> float:
        current_time = float(current_time)

        if not math.isfinite(current_time):
            raise ValueError("current_time must be finite")

        return max(0.0, current_time - self._start_time)

    @staticmethod
    def _smooth_vertical_segment(
        start_altitude: float,
        end_altitude: float,
        phase_time: float,
        duration: float,
    ) -> tuple[float, float]:
        """
        Evaluate a cubic smoothstep position and velocity.

        The vertical velocity is zero at the beginning and end.
        """
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

        altitude_change = (
            end_altitude
            - start_altitude
        )

        position = (
            start_altitude
            + altitude_change * position_scale
        )

        velocity = (
            altitude_change
            / duration
            * velocity_scale
        )

        if s >= 1.0:
            velocity = 0.0

        return position, velocity