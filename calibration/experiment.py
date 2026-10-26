from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Protocol

from .config import CalibrationConfig
from .hardware import attitude_rates
from .logger import ExperimentLogger
from .pid import AltitudePID
from .state import AltitudeState, validate_fresh_state


class Mode(str, Enum):
    PREFLIGHT = "preflight"
    HOVER_BASELINE = "hover-baseline"
    P_ONLY = "p-only"
    PID = "pid"
    TAKEOFF_HOVER_LAND = "takeoff-hover-land"

    @property
    def is_flight(self) -> bool:
        return self is not Mode.PREFLIGHT


class Hardware(Protocol):
    def state(self) -> AltitudeState: ...
    def supervisor_status(self) -> dict[str, bool]: ...
    def arm(self) -> None: ...
    def disarm(self) -> None: ...
    def send(self, roll_rate: float, pitch_rate: float, yaw_rate: float, thrust: float) -> None: ...
    def emergency_stop(self) -> None: ...


@dataclass(frozen=True, slots=True)
class ReferencePoint:
    phase: str
    altitude_m: float
    complete: bool = False


def reference_for(mode: Mode, elapsed: float, initial_altitude: float, config: CalibrationConfig) -> ReferencePoint:
    p = config.profile
    height = p.takeoff_height_m
    up_time = height / p.takeoff_rate_m_s
    if elapsed < up_time:
        return ReferencePoint("TAKEOFF", initial_altitude + elapsed * p.takeoff_rate_m_s)
    if elapsed < up_time + p.hover_duration_s:
        return ReferencePoint("HOVER", initial_altitude + height)
    landing_elapsed = elapsed - up_time - p.hover_duration_s
    down_time = height / p.landing_rate_m_s
    if landing_elapsed < down_time:
        return ReferencePoint("LANDING", initial_altitude + max(0.0, height - landing_elapsed * p.landing_rate_m_s))
    if landing_elapsed + 1e-12 < down_time + p.settle_duration_s:
        return ReferencePoint("SETTLE", initial_altitude)
    return ReferencePoint("COMPLETE", initial_altitude, True)


class CalibrationRunner:
    def __init__(self, hardware: Hardware, config: CalibrationConfig, mode: Mode, logger: ExperimentLogger, clock=time.monotonic, sleeper=time.sleep) -> None:
        self.hardware, self.config, self.mode, self.logger = hardware, config, mode, logger
        self.clock, self.sleeper = clock, sleeper

    def run(self) -> None:
        if self.mode is Mode.PREFLIGHT:
            state = self.hardware.state()
            validate_fresh_state(state, self.clock(), self.config.safety.max_state_age_s)
            status = self.hardware.supervisor_status()
            unsafe = (
                status["is_armed"]
                or status["is_auto_armed"]
                or status["is_flying"]
                or status["is_tumbled"]
                or status["is_locked"]
                or status["is_crashed"]
                or status["hl_control_active"]
                or not status["can_be_armed"]
            )
            if unsafe:
                raise RuntimeError(f"supervisor rejects flight readiness: {status}")
            self.logger.close("valid", "connectivity and state checks passed; no flight command sent")
            return
        self._fly()

    def _fly(self) -> None:
        pid = AltitudePID(self.config.pid)
        first = self.hardware.state()
        now = self.clock()
        validate_fresh_state(first, now, self.config.safety.max_state_age_s)
        initial_altitude, initial_yaw = first.altitude_m, first.yaw_rad
        status = self.hardware.supervisor_status()
        if not status["can_be_armed"] or not status["can_fly"] or status["is_locked"] or status["is_crashed"]:
            raise RuntimeError(f"supervisor safety check failed: {status}")
        period = 1 / self.config.control_frequency_hz
        start = previous = deadline = now
        final_status, reason = "complete", "profile completed"
        try:
            self.hardware.arm()
            while True:
                now = self.clock()
                dt = now - previous
                previous = now
                if dt <= 0:
                    dt = period
                if dt > self.config.safety.max_loop_dt_s:
                    raise TimeoutError(f"control loop dt exceeded limit: {dt:.3f} s")
                state = self.hardware.state()
                age = validate_fresh_state(state, now, self.config.safety.max_state_age_s)
                if abs(state.roll_rad) > self.config.safety.max_tilt_rad or abs(state.pitch_rad) > self.config.safety.max_tilt_rad:
                    raise RuntimeError("tilt safety limit exceeded")
                if state.altitude_m > self.config.safety.max_altitude_m or abs(state.vertical_velocity_m_s) > self.config.safety.max_vertical_speed_m_s:
                    raise RuntimeError("altitude or vertical-speed safety limit exceeded")
                ref = reference_for(self.mode, now - start, initial_altitude, self.config)
                if ref.complete:
                    break
                result = pid.update(ref.altitude_m, state.altitude_m, dt)
                correction = 0.0 if self.mode is Mode.HOVER_BASELINE else result.output
                thrust = min(max(self.config.equilibrium_thrust + correction, self.config.safety.min_thrust), self.config.safety.max_thrust)
                roll, pitch, yaw = attitude_rates(state, initial_yaw, self.config.attitude)
                self.hardware.send(roll, pitch, yaw, thrust)
                self.logger.row({"elapsed_s": now-start, "monotonic_s": now, "phase": ref.phase, "state_source": state.source, "state_age_s": age, "dt_s": dt, "altitude_reference_m": ref.altitude_m, "altitude_m": state.altitude_m, "vertical_velocity_m_s": state.vertical_velocity_m_s, "roll_rad": state.roll_rad, "pitch_rad": state.pitch_rad, "yaw_rad": state.yaw_rad, "error_m": result.error, "p_term": result.proportional, "i_term": result.integral, "d_term": result.derivative, "pid_unsaturated": result.unsaturated, "pid_output": result.output, "thrust_normalized": thrust, "thrust_percent": thrust*100, "saturated": int(result.saturated or thrust in (self.config.safety.min_thrust, self.config.safety.max_thrust))})
                deadline += period
                delay = deadline - self.clock()
                if delay > 0:
                    self.sleeper(delay)
                else:
                    deadline = self.clock()
        except BaseException as exc:
            final_status, reason = "invalid", f"{type(exc).__name__}: {exc}"
            raise
        finally:
            self.hardware.emergency_stop()
            self.hardware.disarm()
            self.logger.close(final_status, reason)
