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
    STATE_MONITOR = "state-monitor"
    ATTITUDE_SIGN_MONITOR = "attitude-sign-monitor"
    OFFICIAL_HOVER_BASELINE = "official-hover-baseline"
    HOVER_BASELINE = "hover-baseline"
    P_ONLY = "p-only"
    PID = "pid"
    TAKEOFF_HOVER_LAND = "takeoff-hover-land"
    CASCADED_5B = "cascaded-5b"
    CASCADED_5C = "cascaded-5c"
    CASCADED_XY = "cascaded-xy"
    PITCH_RESPONSE_ID = "pitch-response-id"
    ROLL_RESPONSE_ID = "roll-response-id"

    @property
    def is_flight(self) -> bool:
        return self not in (Mode.PREFLIGHT, Mode.STATE_MONITOR, Mode.ATTITUDE_SIGN_MONITOR)


class Hardware(Protocol):
    def state(self) -> AltitudeState: ...
    def supervisor_status(self) -> dict[str, bool]: ...
    def arm(self) -> None: ...
    def disarm(self) -> None: ...
    def send(self, roll_rate: float, pitch_rate: float, yaw_rate: float, thrust: float) -> None: ...
    def emergency_stop(self) -> None: ...
    def emergency_stop_available(self) -> bool: ...
    def flow_deck_available(self) -> bool: ...
    def protocol_version(self) -> int: ...
    def supervisor_emergency_stop(self) -> None: ...


class HardSafetyError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class ReferencePoint:
    phase: str
    altitude_m: float
    complete: bool = False
    vertical_velocity_m_s: float = 0.0


def reference_for(mode: Mode, elapsed: float, initial_altitude: float, config: CalibrationConfig) -> ReferencePoint:
    p = config.profile
    height = p.takeoff_height_m
    up_time = height / p.takeoff_rate_m_s
    if elapsed < up_time:
        return ReferencePoint("TAKEOFF", initial_altitude + elapsed * p.takeoff_rate_m_s,
                              vertical_velocity_m_s=p.takeoff_rate_m_s)
    if elapsed < up_time + p.hover_duration_s:
        return ReferencePoint("HOVER", initial_altitude + height)
    landing_elapsed = elapsed - up_time - p.hover_duration_s
    down_time = height / p.landing_rate_m_s
    if landing_elapsed < down_time:
        return ReferencePoint("LANDING", initial_altitude + max(0.0, height - landing_elapsed * p.landing_rate_m_s),
                              vertical_velocity_m_s=-p.landing_rate_m_s)
    if landing_elapsed + 1e-12 < down_time + p.settle_duration_s:
        return ReferencePoint("SETTLE", initial_altitude)
    return ReferencePoint("COMPLETE", initial_altitude, True)


class CalibrationRunner:
    def __init__(self, hardware: Hardware, config: CalibrationConfig, mode: Mode, logger: ExperimentLogger, clock=time.monotonic, sleeper=time.sleep) -> None:
        self.hardware, self.config, self.mode, self.logger = hardware, config, mode, logger
        self.clock, self.sleeper = clock, sleeper

    def run(self) -> None:
        if self.mode in (Mode.STATE_MONITOR, Mode.ATTITUDE_SIGN_MONITOR):
            raise ValueError("monitor modes must use their non-flight monitor runner")
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
        if not self.hardware.emergency_stop_available():
            raise RuntimeError("supervisor emergency stop unavailable")
        if self.hardware.protocol_version() < 12:
            raise RuntimeError("custom flight requires Crazyflie protocol version 12 or newer")
        if not self.hardware.flow_deck_available():
            raise RuntimeError("Flow Deck bcFlow/bcFlow2 is not active")
        pid = AltitudePID(self.config.pid)
        first = self.hardware.state()
        now = self.clock()
        validate_fresh_state(first, now, self.config.safety.max_state_age_s)
        initial_altitude, initial_yaw = first.altitude_m, first.yaw_rad
        origin_x, origin_y = first.position_x_m, first.position_y_m
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
                    self.logger.safety_event("loop_deadline_exceeded", {"dt_s": dt})
                    raise HardSafetyError(f"control loop dt exceeded limit: {dt:.3f} s")
                state = self.hardware.state()
                try:
                    age = validate_fresh_state(state, now, self.config.safety.max_state_age_s)
                except (RuntimeError, TimeoutError) as exc:
                    self.logger.safety_event("invalid_or_stale_state", {
                        "error": str(exc), "state_timestamp_s": state.timestamp_s,
                        "altitude_m": state.altitude_m,
                    })
                    raise HardSafetyError(str(exc)) from exc
                status = self.hardware.supervisor_status()
                if (not status["can_fly"] or not status["is_armed"] or status["is_tumbled"] or
                        status["is_locked"] or status["is_crashed"] or status["hl_control_active"]):
                    self.logger.safety_event("unsafe_supervisor_state", {"supervisor": status})
                    raise HardSafetyError(f"unsafe in-flight supervisor state: {status}")
                if abs(state.roll_rad) > self.config.safety.max_tilt_rad or abs(state.pitch_rad) > self.config.safety.max_tilt_rad:
                    self.logger.safety_event("tilt_limit_exceeded", {"roll_rad": state.roll_rad, "pitch_rad": state.pitch_rad})
                    raise HardSafetyError("tilt safety limit exceeded")
                if state.altitude_m > self.config.safety.max_altitude_m or abs(state.vertical_velocity_m_s) > self.config.safety.max_vertical_speed_m_s:
                    self.logger.safety_event("vertical_envelope_exceeded", {"altitude_m": state.altitude_m, "vertical_velocity_m_s": state.vertical_velocity_m_s})
                    raise HardSafetyError("altitude or vertical-speed safety limit exceeded")
                displacement = math.hypot(state.position_x_m-origin_x, state.position_y_m-origin_y)
                horizontal_speed = math.hypot(state.velocity_x_m_s, state.velocity_y_m_s)
                if (displacement > self.config.safety.max_horizontal_displacement_m or
                        horizontal_speed > self.config.safety.max_horizontal_speed_m_s):
                    self.logger.safety_event("horizontal_envelope_exceeded", {
                        "horizontal_displacement_m": displacement,
                        "horizontal_speed_m_s": horizontal_speed,
                        "origin_x_m": origin_x, "origin_y_m": origin_y,
                    })
                    raise HardSafetyError("horizontal displacement or speed safety limit exceeded")
                ref = reference_for(self.mode, now - start, initial_altitude, self.config)
                if ref.complete:
                    break
                result = pid.update(ref.altitude_m, state.altitude_m, dt)
                correction = 0.0 if self.mode is Mode.HOVER_BASELINE else result.output
                thrust = min(max(self.config.equilibrium_thrust + correction, self.config.safety.min_thrust), self.config.safety.max_thrust)
                roll, pitch, yaw = attitude_rates(state, initial_yaw, self.config.attitude)
                self.hardware.send(roll, pitch, yaw, thrust)
                self.logger.row({"elapsed_s": now-start, "monotonic_s": now, "phase": ref.phase, "state_source": state.source, "state_age_s": age, "dt_s": dt, "altitude_reference_m": ref.altitude_m, "altitude_m": state.altitude_m, "vertical_velocity_m_s": state.vertical_velocity_m_s, "x_m": state.position_x_m, "y_m": state.position_y_m, "vx_m_s": state.velocity_x_m_s, "vy_m_s": state.velocity_y_m_s, "horizontal_displacement_m": displacement, "horizontal_speed_m_s": horizontal_speed, "roll_rad": state.roll_rad, "pitch_rad": state.pitch_rad, "yaw_rad": state.yaw_rad, "error_m": result.error, "p_term": result.proportional, "i_term": result.integral, "d_term": result.derivative, "pid_unsaturated": result.unsaturated, "pid_output": result.output, "thrust_normalized": thrust, "thrust_percent": thrust*100, "saturated": int(result.saturated or thrust in (self.config.safety.min_thrust, self.config.safety.max_thrust))})
                deadline += period
                delay = deadline - self.clock()
                if delay > 0:
                    self.sleeper(delay)
                else:
                    deadline = self.clock()
        except HardSafetyError as exc:
            final_status, reason = "invalid", f"hard safety: {exc}"
            self.hardware.supervisor_emergency_stop()
            raise
        except BaseException as exc:
            final_status, reason = "invalid", f"{type(exc).__name__}: {exc}"
            raise
        finally:
            self.hardware.emergency_stop()
            self.hardware.disarm()
            self.logger.close(final_status, reason)
