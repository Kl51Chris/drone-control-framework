from __future__ import annotations

import csv
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from statistics import median
import time
from typing import Any, Protocol

from velocity_control_framework.interfaces import BodyRateThrustCommand

from .cascaded_pid import CascadedPIDConfig, CascadedPIDController, CascadedReference
from .config import CalibrationConfig
from .experiment import reference_for, Mode
from .state import AltitudeState, validate_fresh_state


SUPERVISOR_FIELDS = ("can_be_armed", "can_fly", "is_armed", "is_auto_armed",
                     "is_flying", "is_tumbled", "is_locked", "is_crashed",
                     "hl_control_active")
BASE_FIELDS = [
    "elapsed_s", "monotonic_s", "dt_s", "phase", "state_source", "state_age_s",
    "reference_x_m", "reference_y_m", "reference_z_m", "reference_vx_m_s",
    "reference_vy_m_s", "reference_vz_m_s", "reference_yaw_rad",
    "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s", "roll_rad",
    "pitch_rad", "yaw_rad", "p_rad_s", "q_rad_s", "r_rad_s",
    "horizontal_displacement_m", "horizontal_speed_m_s", "tilt_rad",
]
CONTRIBUTION_FIELDS = [
    "altitude_error_m", "vertical_velocity_error_m_s", "altitude_p",
    "altitude_velocity", "altitude_integral", "altitude_correction_raw",
    "altitude_correction", "altitude_saturated", "integral_update_accepted",
    "desired_roll_rad", "desired_pitch_rad", "desired_yaw_rad",
    "horizontal_acceleration_x_world_m_s2", "horizontal_acceleration_y_world_m_s2",
    "horizontal_acceleration_x_body_m_s2", "horizontal_acceleration_y_body_m_s2",
    "roll_error_rad", "pitch_error_rad", "yaw_error_rad",
    "roll_rate_raw_rad_s", "pitch_rate_raw_rad_s", "yaw_rate_raw_rad_s",
    "roll_rate_saturated", "pitch_rate_saturated", "yaw_rate_saturated",
]
COMMAND_FIELDS = ["command_roll_rate_rad_s", "command_pitch_rate_rad_s",
                  "command_yaw_rate_rad_s", "command_thrust_normalized"]
LIMIT_FIELDS = ["limit_max_tilt_rad", "limit_max_altitude_m",
                "limit_max_vertical_speed_m_s", "limit_max_horizontal_displacement_m",
                "limit_max_horizontal_speed_m_s", "limit_max_state_age_s",
                "limit_max_loop_dt_s", "limit_min_thrust", "limit_max_thrust",
                "limit_max_roll_rate_rad_s", "limit_max_pitch_rate_rad_s",
                "limit_max_yaw_rate_rad_s"]
FIELDS = BASE_FIELDS + CONTRIBUTION_FIELDS + COMMAND_FIELDS + list(SUPERVISOR_FIELDS) + LIMIT_FIELDS


class CascadedHardware(Protocol):
    def state(self) -> AltitudeState: ...
    def supervisor_status(self) -> dict[str, bool]: ...
    def flow_deck_available(self) -> bool: ...
    def emergency_stop_available(self) -> bool: ...
    def protocol_version(self) -> int: ...
    def arm(self) -> None: ...
    def disarm(self) -> None: ...
    def send_command(self, command: BodyRateThrustCommand) -> None: ...
    def emergency_stop(self) -> None: ...
    def supervisor_emergency_stop(self) -> None: ...


class CascadedFlightLogger:
    def __init__(self, root: str | Path, config: CalibrationConfig,
                 controller_config: CascadedPIDConfig) -> None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        self.directory = Path(root)/f"{stamp}_cascaded-5b"
        self.directory.mkdir(parents=True, exist_ok=False)
        self._stream = (self.directory/"flight_telemetry.csv").open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._stream, fieldnames=FIELDS)
        self._writer.writeheader(); self._stream.flush()
        self.metadata = {
            "schema_version": 1, "mode": "cascaded-5b", "status": "running",
            "started_utc": datetime.now(timezone.utc).isoformat(),
            "flight_config": asdict(config), "controller_config": asdict(controller_config),
            "controller": "calibration.cascaded_pid.CascadedPIDController",
            "command_interface": "BodyRateThrustCommand; firmware body-rate PID and mixer retained",
        }
        self._write_metadata()

    def _write_metadata(self) -> None:
        (self.directory/"metadata.json").write_text(
            json.dumps(self.metadata, indent=2, sort_keys=True)+"\n", encoding="utf-8")

    def row(self, values: dict[str, Any]) -> None:
        self._writer.writerow(values); self._stream.flush()

    def diagnostic_event(self, condition: str, snapshot: dict[str, Any]) -> None:
        self.metadata["safety_event"] = {
            "condition": condition, "recorded_utc": datetime.now(timezone.utc).isoformat(),
            **snapshot,
        }
        self._write_metadata()

    def close(self, status: str, reason: str, shutdown: dict[str, Any]) -> None:
        if not self._stream.closed: self._stream.close()
        self.metadata.update(status=status, termination_reason=reason,
                             shutdown=shutdown, ended_utc=datetime.now(timezone.utc).isoformat())
        self._write_metadata()


class HardSafetyError(RuntimeError):
    def __init__(self, condition: str, message: str, snapshot: dict[str, Any]):
        super().__init__(message); self.condition = condition; self.snapshot = snapshot


class CascadedStage5BRunner:
    LANDING_CONFIRMATION_S = 0.5
    LANDING_TIMEOUT_S = 3.0
    LANDING_MAX_RELATIVE_HEIGHT_M = 0.06
    LANDING_MAX_VERTICAL_SPEED_M_S = 0.08
    def __init__(self, hardware: CascadedHardware, config: CalibrationConfig,
                 controller_config: CascadedPIDConfig, logger: CascadedFlightLogger,
                 clock=time.monotonic, sleeper=time.sleep) -> None:
        self.hardware, self.config, self.controller_config, self.logger = hardware, config, controller_config, logger
        self.clock, self.sleeper = clock, sleeper
        if controller_config.stage != "5B": raise ValueError("Stage 5B runner requires a 5B controller config")
        if (controller_config.max_thrust > config.safety.max_thrust or
                controller_config.min_thrust < config.safety.min_thrust):
            raise ValueError("controller thrust bounds exceed flight safety bounds")

    def run(self) -> None:
        status, reason = "aborted", "flight did not start"
        shutdown: dict[str, Any] = {}
        try:
            origin = self._preflight()
            controller = CascadedPIDController(self.controller_config)
            self.hardware.arm(); self.sleeper(.25)
            armed = self.hardware.supervisor_status()
            if not armed["is_armed"] or not armed["can_fly"]:
                raise HardSafetyError("arming_not_confirmed", "supervisor did not confirm armed readiness", {"supervisor": armed})
            period = 1/self.config.control_frequency_hz
            start = previous = deadline = self.clock()
            landing_confirmation_start = None
            landing_timeout_start = None
            while True:
                now = self.clock(); dt = now-previous; previous = now
                if dt <= 0: dt = period
                if dt > self.config.safety.max_loop_dt_s:
                    raise HardSafetyError("loop_deadline_exceeded", f"loop dt {dt:.3f}s", {"dt_s": dt})
                state = self.hardware.state()
                supervisor = self.hardware.supervisor_status()
                snapshot = self._snapshot(state, supervisor, origin, now)
                try: age = validate_fresh_state(state, now, self.config.safety.max_state_age_s)
                except (RuntimeError, TimeoutError) as exc:
                    raise HardSafetyError("invalid_or_stale_state", str(exc), snapshot) from exc
                self._check_safety(state, supervisor, snapshot)
                profile = reference_for(Mode.CASCADED_5B, now-start, origin[2], self.config)
                phase, reference_altitude = profile.phase, profile.altitude_m
                if profile.complete:
                    phase, reference_altitude = "LANDING_CONFIRM", origin[2]
                    if landing_timeout_start is None: landing_timeout_start = now
                    landed = (state.altitude_m-origin[2] <= self.LANDING_MAX_RELATIVE_HEIGHT_M and
                              abs(state.vertical_velocity_m_s) <= self.LANDING_MAX_VERTICAL_SPEED_M_S)
                    if landed:
                        if landing_confirmation_start is None: landing_confirmation_start = now
                        if now-landing_confirmation_start >= self.LANDING_CONFIRMATION_S: break
                    else: landing_confirmation_start = None
                    if now-landing_timeout_start >= self.LANDING_TIMEOUT_S:
                        raise HardSafetyError("touchdown_not_confirmed", "touchdown was not confirmed before timeout", snapshot)
                reference = CascadedReference(reference_altitude, yaw_rad=origin[3],
                                              position_x_m=origin[0], position_y_m=origin[1])
                result = controller.update(state, reference, dt)
                row = self._row(now-start, now, dt, phase, age, state,
                                reference, result, supervisor, origin)
                self.logger.row(row)
                self.hardware.send_command(result.command)
                deadline += period
                delay = deadline-self.clock()
                if delay > 0: self.sleeper(delay)
                else: deadline = self.clock()
            status, reason = "complete", "profile completed"
        except KeyboardInterrupt:
            status, reason = "interrupted", "KeyboardInterrupt/operator stop"
            shutdown = self._shutdown(supervisor_estop=False)
            raise
        except HardSafetyError as exc:
            status, reason = "aborted", f"hard safety: {exc}"
            shutdown = self._shutdown(supervisor_estop=True)
            self.logger.diagnostic_event(exc.condition, exc.snapshot)
            raise
        except BaseException as exc:
            status, reason = "aborted", f"unexpected {type(exc).__name__}: {exc}"
            shutdown = self._shutdown(supervisor_estop=False)
            self.logger.diagnostic_event("unexpected_exception", {"type": type(exc).__name__, "message": str(exc)})
            raise
        finally:
            if not shutdown: shutdown = self._shutdown(supervisor_estop=False)
            self.logger.close(status, reason, shutdown)

    def _preflight(self) -> tuple[float, float, float, float]:
        if not self.hardware.emergency_stop_available(): raise RuntimeError("supervisor emergency stop unavailable")
        if self.hardware.protocol_version() < 12: raise RuntimeError("protocol version 12 or newer required")
        if not self.hardware.flow_deck_available(): raise RuntimeError("Flow Deck bcFlow/bcFlow2 is not active")
        status = self.hardware.supervisor_status()
        if (status["is_armed"] or status["is_flying"] or status["is_tumbled"] or status["is_locked"] or
                status["is_crashed"] or status["hl_control_active"] or not status["can_be_armed"]):
            raise HardSafetyError("preflight_supervisor_rejected", f"supervisor rejects flight: {status}", {"supervisor":status})
        samples=[]; end=self.clock()+.5
        while self.clock() < end:
            state=self.hardware.state()
            try: validate_fresh_state(state,self.clock(),self.config.safety.max_state_age_s)
            except (RuntimeError,TimeoutError) as exc:
                raise HardSafetyError("preflight_invalid_or_stale_state",str(exc),{"state":asdict(state),"supervisor":status}) from exc
            samples.append(state); self.sleeper(min(.02,max(0,end-self.clock())))
        if len(samples)<2: raise RuntimeError("insufficient preflight state samples")
        if max(s.altitude_m for s in samples)-min(s.altitude_m for s in samples)>.10:
            raise HardSafetyError("preflight_estimator_unsettled","estimator altitude is not settled",{"state":asdict(samples[-1]),"supervisor":status})
        if max(max(abs(s.roll_rad),abs(s.pitch_rad)) for s in samples)>.20:
            raise HardSafetyError("preflight_not_level","vehicle is not level",{"state":asdict(samples[-1]),"supervisor":status})
        origin=(median(s.position_x_m for s in samples),median(s.position_y_m for s in samples),
                median(s.altitude_m for s in samples),median(s.yaw_rad for s in samples))
        self.logger.metadata["captured_reference"]={"x_m":origin[0],"y_m":origin[1],"ground_z_m":origin[2],"yaw_rad":origin[3],"samples":len(samples)}
        self.logger._write_metadata(); return origin

    def _snapshot(self,state,supervisor,origin,now):
        return {"monotonic_s":now,"state":asdict(state),"supervisor":supervisor,
                "horizontal_displacement_m":math.hypot(state.position_x_m-origin[0],state.position_y_m-origin[1]),
                "horizontal_speed_m_s":math.hypot(state.velocity_x_m_s,state.velocity_y_m_s)}

    def _check_safety(self,state,supervisor,snapshot):
        s=self.config.safety
        checks=[("unsafe_supervisor_state",not supervisor["can_fly"] or not supervisor["is_armed"] or supervisor["is_tumbled"] or supervisor["is_locked"] or supervisor["is_crashed"] or supervisor["hl_control_active"]),
                ("tilt_limit_exceeded",max(abs(state.roll_rad),abs(state.pitch_rad))>s.max_tilt_rad),
                ("vertical_envelope_exceeded",state.altitude_m>s.max_altitude_m or abs(state.vertical_velocity_m_s)>s.max_vertical_speed_m_s),
                ("horizontal_envelope_exceeded",snapshot["horizontal_displacement_m"]>s.max_horizontal_displacement_m or snapshot["horizontal_speed_m_s"]>s.max_horizontal_speed_m_s)]
        for name,failed in checks:
            if failed: raise HardSafetyError(name,name.replace("_"," "),snapshot)

    def _row(self,elapsed,now,dt,phase,age,state,reference,result,supervisor,origin):
        c=result.command;s=self.config.safety;a=self.controller_config.attitude
        return {"elapsed_s":elapsed,"monotonic_s":now,"dt_s":dt,"phase":phase,"state_source":state.source,"state_age_s":age,
            "reference_x_m":reference.position_x_m,"reference_y_m":reference.position_y_m,"reference_z_m":reference.altitude_m,"reference_vx_m_s":reference.velocity_x_m_s,"reference_vy_m_s":reference.velocity_y_m_s,"reference_vz_m_s":reference.vertical_velocity_m_s,"reference_yaw_rad":reference.yaw_rad,
            "x_m":state.position_x_m,"y_m":state.position_y_m,"z_m":state.altitude_m,"vx_m_s":state.velocity_x_m_s,"vy_m_s":state.velocity_y_m_s,"vz_m_s":state.vertical_velocity_m_s,"roll_rad":state.roll_rad,"pitch_rad":state.pitch_rad,"yaw_rad":state.yaw_rad,"p_rad_s":state.body_rate_p_rad_s,"q_rad_s":state.body_rate_q_rad_s,"r_rad_s":state.body_rate_r_rad_s,
            "horizontal_displacement_m":math.hypot(state.position_x_m-origin[0],state.position_y_m-origin[1]),"horizontal_speed_m_s":math.hypot(state.velocity_x_m_s,state.velocity_y_m_s),"tilt_rad":max(abs(state.roll_rad),abs(state.pitch_rad)),**result.contributions,
            "command_roll_rate_rad_s":c.roll_rate,"command_pitch_rate_rad_s":c.pitch_rate,"command_yaw_rate_rad_s":c.yaw_rate,"command_thrust_normalized":c.thrust,**supervisor,
            "limit_max_tilt_rad":s.max_tilt_rad,"limit_max_altitude_m":s.max_altitude_m,"limit_max_vertical_speed_m_s":s.max_vertical_speed_m_s,"limit_max_horizontal_displacement_m":s.max_horizontal_displacement_m,"limit_max_horizontal_speed_m_s":s.max_horizontal_speed_m_s,"limit_max_state_age_s":s.max_state_age_s,"limit_max_loop_dt_s":s.max_loop_dt_s,"limit_min_thrust":self.controller_config.min_thrust,"limit_max_thrust":self.controller_config.max_thrust,"limit_max_roll_rate_rad_s":a.max_roll_rate_rad_s,"limit_max_pitch_rate_rad_s":a.max_pitch_rate_rad_s,"limit_max_yaw_rate_rad_s":a.max_yaw_rate_rad_s}

    def _shutdown(self,supervisor_estop):
        result={"supervisor_emergency_stop_requested":supervisor_estop,"supervisor_emergency_stop_succeeded":False,"stop_setpoint_attempted":True,"stop_setpoint_succeeded":False,"disarm_attempted":True,"disarm_succeeded":False}
        if supervisor_estop:
            try:self.hardware.supervisor_emergency_stop();result["supervisor_emergency_stop_succeeded"]=True
            except BaseException as exc:result["supervisor_emergency_stop_error"]=f"{type(exc).__name__}: {exc}"
        try:self.hardware.emergency_stop();result["stop_setpoint_succeeded"]=True
        except BaseException as exc:result["stop_setpoint_error"]=f"{type(exc).__name__}: {exc}"
        try:self.hardware.disarm();result["disarm_succeeded"]=True
        except BaseException as exc:result["disarm_error"]=f"{type(exc).__name__}: {exc}"
        return result
