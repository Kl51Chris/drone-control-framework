from __future__ import annotations

import math
import threading
import time
from typing import Any

from velocity_control_framework.backends import CrazyflieCommandAdapter, CrazyflieStateProvider
from velocity_control_framework.interfaces import BodyRateThrustCommand

from .state import AltitudeState, ExternalAltitudeReceiver


class CrazyflieHardware:
    """Thin lifecycle wrapper around the repository's verified CF adapters."""

    PARAMETER_READY_TIMEOUT_S = 10.0

    def __init__(self, uri: str, external_altitude_port: int | None = None) -> None:
        self.uri = uri
        self.external_altitude_port = external_altitude_port
        self._link: Any = None
        self._cf: Any = None
        self._provider: Any = None
        self._commands: Any = None
        self._external: ExternalAltitudeReceiver | None = None
        self._control_log: Any = None
        self._control_log_lock = threading.Lock()
        self._onboard_thrust: tuple[float, float] | None = None

    def connect(self) -> None:
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

        cflib.crtp.init_drivers()
        self._link = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache="./cache"))
        self._link.open_link()
        self._cf = self._link.cf
        self._wait_for_parameters()
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
        return AltitudeState(
            altitude, velocity, state.roll, state.pitch, state.yaw, timestamp, source,
            float(state.position[0]), float(state.position[1]),
            float(state.velocity[0]), float(state.velocity[1]),
        )

    def send_hover_setpoint(self, height_m: float) -> None:
        """Use the official firmware hover controller; height is metres."""
        self._cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, float(height_m))

    def notify_setpoint_stop(self) -> None:
        self._cf.commander.send_notify_setpoint_stop()

    def supervisor_emergency_stop(self) -> None:
        self._cf.supervisor.send_emergency_stop()

    def emergency_stop_available(self) -> bool:
        return callable(getattr(self._cf.supervisor, "send_emergency_stop", None))

    def protocol_version(self) -> int:
        return int(self._cf.platform.get_protocol_version())

    def _wait_for_parameters(self, timeout_s: float | None = None, poll_interval_s: float = 0.01) -> None:
        """Wait until cflib has fetched parameter values, with a bounded failure."""
        if self._link is None:
            raise RuntimeError("Crazyflie link is not initialized")
        timeout = self.PARAMETER_READY_TIMEOUT_S if timeout_s is None else timeout_s
        deadline = time.monotonic() + timeout
        try:
            while not self._link.is_params_updated():
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"timed out waiting {timeout:g}s for Crazyflie parameter values")
                time.sleep(min(poll_interval_s, max(0.0, deadline-time.monotonic())))
        except RuntimeError:
            raise
        except Exception as exc:
            raise RuntimeError(f"failed while waiting for Crazyflie parameter values: {exc}") from exc

    @staticmethod
    def _parameter_flag(value: Any, complete_name: str) -> bool:
        """Parse cflib string flags while also accepting integer values."""
        if isinstance(value, bool):
            return value
        if isinstance(value, int):
            return value != 0
        if isinstance(value, str):
            try:
                return int(value.strip(), 0) != 0
            except ValueError as exc:
                raise RuntimeError(f"invalid numeric value for {complete_name}: {value!r}") from exc
        raise RuntimeError(f"invalid value type for {complete_name}: {type(value).__name__}")

    def flow_deck_available(self) -> bool:
        """Return Flow Deck presence from fully fetched firmware parameters."""
        try:
            values = self._cf.param.values
            if not isinstance(values, dict):
                raise RuntimeError("Crazyflie parameter values are not a mapping")
            deck = values.get("deck", {})
            if deck is None:
                deck = {}
            if not isinstance(deck, dict):
                raise RuntimeError("Crazyflie deck parameter group is not a mapping")
            return any(
                self._parameter_flag(deck[name], f"deck.{name}")
                for name in ("bcFlow2", "bcFlow")
                if name in deck
            )
        except RuntimeError:
            raise
        except Exception as exc:
            raise RuntimeError(f"failed to retrieve Flow Deck parameters: {exc}") from exc

    def available_log_variables(self) -> set[str]:
        toc = getattr(getattr(self._cf.log, "toc", None), "toc", {})
        if not isinstance(toc, dict):
            return set()
        return {f"{group}.{name}" for group, variables in toc.items() if isinstance(variables, dict) for name in variables}

    def start_onboard_thrust_log(self, period_ms: int = 20) -> bool:
        """Log firmware `stabilizer.thrust` when present (raw control units)."""
        if "stabilizer.thrust" not in self.available_log_variables():
            return False
        from cflib.crazyflie.log import LogConfig
        config = LogConfig(name="CalibrationOnboardControl", period_in_ms=period_ms)
        config.add_variable("stabilizer.thrust")
        config.data_received_cb.add_callback(self._on_control_log)
        self._cf.log.add_config(config)
        if not config.valid:
            raise RuntimeError("onboard thrust log configuration is invalid")
        config.start()
        self._control_log = config
        return True

    def _on_control_log(self, timestamp: int, data: dict[str, float], log_config: Any) -> None:
        del timestamp, log_config
        value = data.get("stabilizer.thrust")
        if value is not None:
            with self._control_log_lock:
                self._onboard_thrust = (float(value), time.monotonic())

    def onboard_thrust_raw(self) -> tuple[float | None, float | None]:
        with self._control_log_lock:
            if self._onboard_thrust is None:
                return None, None
            return self._onboard_thrust

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
        if self._control_log is not None:
            try:
                self._control_log.stop()
            except Exception:
                pass
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
