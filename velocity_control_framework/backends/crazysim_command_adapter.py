from __future__ import annotations

import math
import time
from typing import Any, Callable

from velocity_control_framework.interfaces import BodyRateThrustCommand


class CrazySimCommandAdapter:
    """Legacy RPYT adapter for the firmware bundled with CrazySim.

    The bundled firmware does not decode CFLib's generic TYPE_MANUAL packet.
    It does support the legacy RPYT packet when roll and pitch are explicitly
    configured for rate mode. Construction verifies those modes; ``unlock()``
    sends the legacy zero-thrust packet immediately after confirmed arming.
    """

    RATE_MODE = 0
    ROLL_MODE_PARAMETER = "flightmode.stabModeRoll"
    PITCH_MODE_PARAMETER = "flightmode.stabModePitch"
    YAW_MODE_PARAMETER = "flightmode.stabModeYaw"
    RAW_THRUST_MIN = 10001
    RAW_THRUST_MAX = 60000

    def __init__(
        self,
        crazyflie: Any,
        *,
        verification_timeout_s: float = 1.0,
        poll_interval_s: float = 0.02,
        clock: Callable[[], float] = time.monotonic,
        sleeper: Callable[[float], None] = time.sleep,
    ) -> None:
        self._cf = crazyflie
        self._initialized = False
        self._configure_rate_modes(
            verification_timeout_s, poll_interval_s, clock, sleeper
        )

    def _configure_rate_modes(
        self,
        timeout_s: float,
        poll_interval_s: float,
        clock: Callable[[], float],
        sleeper: Callable[[float], None],
    ) -> None:
        if timeout_s <= 0.0:
            raise ValueError("verification_timeout_s must be positive")
        if poll_interval_s < 0.0:
            raise ValueError("poll_interval_s must be nonnegative")

        parameters = (
            self.ROLL_MODE_PARAMETER,
            self.PITCH_MODE_PARAMETER,
            self.YAW_MODE_PARAMETER,
        )
        try:
            for name in parameters:
                self._cf.param.set_value(name, str(self.RATE_MODE))

            deadline = clock() + timeout_s
            while True:
                values = {name: self._cf.param.get_value(name) for name in parameters}
                if all(int(value) == self.RATE_MODE for value in values.values()):
                    return
                if clock() >= deadline:
                    raise RuntimeError(
                        "CrazySim firmware did not confirm roll/pitch/yaw rate mode: "
                        f"{values}"
                    )
                sleeper(poll_interval_s)
        except BaseException as error:
            self._fail_closed()
            if isinstance(error, RuntimeError):
                raise
            raise RuntimeError(
                "Failed to configure and verify CrazySim roll/pitch/yaw rate mode"
            ) from error

    def unlock(self) -> None:
        """Release the legacy thrust lock immediately after confirmed arming."""
        try:
            self._cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
        except BaseException as error:
            self._fail_closed()
            raise RuntimeError("Failed to send CrazySim legacy unlock setpoint") from error
        self._initialized = True

    def send(self, command: BodyRateThrustCommand) -> None:
        if not self._initialized:
            raise RuntimeError("CrazySim command adapter is not initialized")
        self._cf.commander.send_setpoint(
            math.degrees(command.roll_rate),
            math.degrees(command.pitch_rate),
            math.degrees(command.yaw_rate),
            self._encode_thrust(command.thrust),
        )

    def stop(self) -> None:
        """Use the generic STOP packet; this is distinct from legacy unlock."""
        self._cf.commander.send_stop_setpoint()

    def emergency_stop(self) -> None:
        """Request the firmware supervisor's latched emergency stop."""
        self._cf.supervisor.send_emergency_stop()

    def _fail_closed(self) -> None:
        self._initialized = False
        try:
            self._cf.commander.send_stop_setpoint()
        except BaseException:
            pass

    @classmethod
    def _encode_thrust(cls, thrust: float) -> int:
        """Match CFLib send_setpoint_manual's percentage-to-raw mapping."""
        limited = min(max(float(thrust), 0.0), 1.0)
        return int(cls.RAW_THRUST_MIN + limited * (cls.RAW_THRUST_MAX - cls.RAW_THRUST_MIN))
