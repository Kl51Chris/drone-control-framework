from __future__ import annotations

import math
from typing import Any

from velocity_control_framework.interfaces import AttitudeThrustCommand


class CrazyflieCommandAdapter:
    """
    Convert AttitudeThrustCommand into Crazyflie commander calls.

    Controller-side units:
        roll: radians
        pitch: radians
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
        """Send one attitude-thrust setpoint to Crazyflie."""
        roll = self._convert_roll(command.roll)
        pitch = self._convert_pitch(command.pitch)
        yaw_rate = self._convert_yaw_rate(
            command.yaw_rate
        )
        thrust = self._encode_thrust(command.thrust)

        self._cf.commander.send_setpoint(
            roll,
            pitch,
            yaw_rate,
            thrust,
        )

    def stop(self) -> None:
        """Send the Crazyflie stop-setpoint command."""
        self._cf.commander.send_stop_setpoint()

    @staticmethod
    def _convert_roll(roll_rad: float) -> float:
        """Convert controller roll from radians to Crazyflie units."""
        return math.degrees(roll_rad)

    @staticmethod
    def _convert_pitch(pitch_rad: float) -> float:
        """
        Convert controller pitch from radians to Crazyflie units.

        Any required sign adjustment belongs here.
        """
        return math.degrees(pitch_rad)

    @staticmethod
    def _convert_yaw_rate(
        yaw_rate_rad_s: float,
    ) -> float:
        """Convert yaw rate from rad/s to deg/s."""
        return math.degrees(yaw_rate_rad_s)

    @staticmethod
    def _encode_thrust(thrust: float) -> int:
        """
        Convert normalized thrust [0, 1] to Crazyflie uint16 encoding.
        """
        limited = min(max(float(thrust), 0.0), 1.0)
        return int(round(limited * 65535.0))