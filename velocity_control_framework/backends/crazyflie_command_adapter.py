from __future__ import annotations

import math
from typing import Any

from velocity_control_framework.interfaces import AttitudeThrustCommand


class CrazyflieCommandAdapter:
    """
    Convert AttitudeThrustCommand into Crazyflie commander calls.

    Controller-side units:
        roll: radians/second
        pitch: radians/second
        yaw_rate: radians/second
        thrust: normalized [0.0, 1.0]

    Crazyflie-specific unit conversion and sign conventions
    remain inside this adapter.
    """

    def __init__(self, crazyflie: Any) -> None:
        self._cf = crazyflie

    def send(
        self,
        command: AttitudeThrustCommand,
    ) -> None:
        """Send one body-rate and thrust setpoint to Crazyflie."""
        roll_rate = self._convert_roll_rate(command.roll)
        pitch_rate = self._convert_pitch_rate(command.pitch)
        yaw_rate = self._convert_yaw_rate(
            command.yaw_rate
        )
        thrust = self._encode_thrust(command.thrust)

        self._cf.commander.send_setpoint_manual(
            roll_rate,
            pitch_rate,
            yaw_rate,
            thrust,
            rate=True,
        )

    def stop(self) -> None:
        """Send the Crazyflie stop-setpoint command."""
        self._cf.commander.send_stop_setpoint()

    @staticmethod
    def _convert_roll_rate(roll_rate_rad_s: float) -> float:
        """Convert roll rate from rad/s to deg/s."""
        return math.degrees(roll_rate_rad_s)

    @staticmethod
    def _convert_pitch_rate(pitch_rate_rad_s: float) -> float:
        """
        Convert pitch rate from rad/s to deg/s.

        Any required sign adjustment belongs here.
        """
        return math.degrees(pitch_rate_rad_s)

    @staticmethod
    def _convert_yaw_rate(
        yaw_rate_rad_s: float,
    ) -> float:
        """Convert yaw rate from rad/s to deg/s."""
        return math.degrees(yaw_rate_rad_s)

    @staticmethod
    def _encode_thrust(thrust: float) -> float:
        """
        Convert normalized thrust [0, 1] to Crazyflie percentage.
        """
        limited = min(max(float(thrust), 0.0), 1.0)
        return limited * 100.0
