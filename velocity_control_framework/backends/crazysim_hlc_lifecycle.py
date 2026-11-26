"""CrazySim commander ownership transitions, with no controller dependencies."""

from __future__ import annotations

from enum import Enum, auto
import math
from typing import Any

from velocity_control_framework.backends.crazysim_command_adapter import (
    CrazySimCommandAdapter,
)
from velocity_control_framework.interfaces import BodyRateThrustCommand


class CommandOwner(Enum):
    GROUND = auto()
    OFFICIAL = auto()
    BODY_RATE = auto()
    LANDING = auto()
    STOPPED = auto()


class CrazySimHLCLifecycle:
    """Issue mode transitions on one connection.

    The experiment remains responsible for verifying safe startup, a stable
    hover, fresh telemetry, and touchdown. Owner describes commands requested
    by the host, not an acknowledgement from firmware.
    """

    def __init__(self, cf: Any, body_rate: CrazySimCommandAdapter) -> None:
        self._cf = cf
        self._body_rate = body_rate
        self.owner = CommandOwner.GROUND

    def takeoff(self, height_m: float, duration_s: float) -> None:
        if self.owner is not CommandOwner.GROUND:
            raise RuntimeError("HLC takeoff requires ground ownership")
        self._validate_target(height_m, duration_s)
        self._body_rate.unlock()
        self._cf.commander.send_notify_setpoint_stop()
        self._cf.high_level_commander.takeoff(height_m, duration_s, yaw=None)
        self.owner = CommandOwner.OFFICIAL

    def send_tracking(self, command: BodyRateThrustCommand) -> None:
        if self.owner not in (CommandOwner.OFFICIAL, CommandOwner.BODY_RATE):
            raise RuntimeError("body-rate tracking requires an active flight")
        self._body_rate.send(command)
        self.owner = CommandOwner.BODY_RATE

    def land(self, height_m: float, duration_s: float) -> None:
        if self.owner not in (CommandOwner.OFFICIAL, CommandOwner.BODY_RATE):
            raise RuntimeError("HLC landing requires an active flight")
        self._validate_target(height_m, duration_s)
        if self.owner is CommandOwner.BODY_RATE:
            self._cf.commander.send_notify_setpoint_stop()
        self._cf.high_level_commander.land(height_m, duration_s, yaw=None)
        self.owner = CommandOwner.LANDING

    def stop(self) -> None:
        self.owner = CommandOwner.STOPPED
        try:
            self._cf.high_level_commander.stop()
        finally:
            self._body_rate.stop()

    @staticmethod
    def _validate_target(height_m: float, duration_s: float) -> None:
        if not math.isfinite(height_m):
            raise ValueError("HLC target height must be finite")
        if not math.isfinite(duration_s) or duration_s <= 0:
            raise ValueError("HLC duration must be finite and positive")
