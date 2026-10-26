from __future__ import annotations

import math
import time
from typing import Any

from velocity_control_framework.backends import CrazyflieCommandAdapter, CrazyflieStateProvider
from velocity_control_framework.interfaces import BodyRateThrustCommand

from .state import AltitudeState, ExternalAltitudeReceiver


class CrazyflieHardware:
    """Thin lifecycle wrapper around the repository's verified CF adapters."""

    def __init__(self, uri: str, external_altitude_port: int | None = None) -> None:
        self.uri = uri
        self.external_altitude_port = external_altitude_port
        self._link: Any = None
        self._cf: Any = None
        self._provider: Any = None
        self._commands: Any = None
        self._external: ExternalAltitudeReceiver | None = None

    def connect(self) -> None:
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

        cflib.crtp.init_drivers()
        self._link = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache="./cache"))
        self._link.open_link()
        self._cf = self._link.cf
        self._provider = CrazyflieStateProvider(self._cf)
        self._commands = CrazyflieCommandAdapter(self._cf)
        self._provider.start()
        if self.external_altitude_port is not None:
            self._external = ExternalAltitudeReceiver(self.external_altitude_port)
            self._external.start()

    def state(self) -> AltitudeState:
        state = self._provider.get_state()
        altitude, velocity, timestamp, source = float(state.position[2]), float(state.velocity[2]), state.timestamp, "crazyflie"
        if self._external is not None:
            altitude, velocity, timestamp = self._external.latest()
            source = "external_udp"
        return AltitudeState(altitude, velocity, state.roll, state.pitch, state.yaw, timestamp, source)

    def supervisor_status(self) -> dict[str, bool]:
        supervisor = self._cf.supervisor
        return {
            "can_be_armed": bool(supervisor.can_be_armed),
            "can_fly": bool(supervisor.can_fly),
            "is_armed": bool(supervisor.is_armed),
            "is_auto_armed": bool(supervisor.is_auto_armed),
            "is_flying": bool(supervisor.is_flying),
            "is_tumbled": bool(supervisor.is_tumbled),
            "is_locked": bool(supervisor.is_locked),
            "is_crashed": bool(supervisor.is_crashed),
            "hl_control_active": bool(supervisor.hl_control_active),
        }

    def arm(self) -> None:
        self._cf.supervisor.send_arming_request(True)

    def disarm(self) -> None:
        self._cf.supervisor.send_arming_request(False)

    def send(self, roll_rate: float, pitch_rate: float, yaw_rate: float, thrust: float) -> None:
        self._commands.send(BodyRateThrustCommand(roll_rate, pitch_rate, yaw_rate, thrust))

    @staticmethod
    def encode_thrust(thrust: float) -> float:
        return CrazyflieCommandAdapter._encode_thrust(thrust)

    def emergency_stop(self) -> None:
        if self._commands is not None:
            for _ in range(10):
                try:
                    self._commands.stop()
                except Exception:
                    pass
                time.sleep(0.01)

    def close(self) -> None:
        self.emergency_stop()
        try:
            self.disarm()
        except Exception:
            pass
        if self._external is not None:
            self._external.stop()
        if self._provider is not None:
            self._provider.stop()
        if self._link is not None:
            self._link.close_link()


def attitude_rates(state: AltitudeState, initial_yaw: float, settings: Any) -> tuple[float, float, float]:
    limit = settings.max_rate_rad_s
    clamp = lambda value: max(-limit, min(limit, value))
    yaw_error = math.atan2(math.sin(initial_yaw - state.yaw_rad), math.cos(initial_yaw - state.yaw_rad))
    return (
        clamp(settings.roll_sign * settings.roll_kp * state.roll_rad),
        clamp(settings.pitch_sign * settings.pitch_kp * state.pitch_rad),
        clamp(settings.yaw_sign * settings.yaw_kp * -yaw_error),
    )
