from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
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
from .experiment import reference_for, Mode, ReferencePoint
from .state import AltitudeState, validate_fresh_state


SUPERVISOR_FIELDS = ("can_be_armed", "can_fly", "is_armed", "is_auto_armed",
                     "is_flying", "is_tumbled", "is_locked", "is_crashed",
                     "hl_control_active")
BASE_FIELDS = [
    "elapsed_s", "monotonic_s", "dt_s", "phase", "state_source", "state_age_s",
    "pitch_identification_phase", "pitch_excitation_reference_rad", "pitch_identification_ready",
    "roll_identification_phase", "roll_excitation_reference_rad", "roll_identification_ready",
    "reference_x_m", "reference_y_m", "reference_z_m", "reference_vx_m_s",
    "reference_vy_m_s", "reference_vz_m_s", "reference_yaw_rad",
    "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s", "roll_rad",
    "pitch_rad", "yaw_rad", "p_rad_s", "q_rad_s", "r_rad_s",
    "horizontal_displacement_m", "horizontal_speed_m_s", "tilt_rad",
]
CONTRIBUTION_FIELDS = [
    "altitude_error_m", "altitude_velocity_reference_m_s", "vertical_velocity_error_m_s", "altitude_p",
    "altitude_velocity", "altitude_integral", "altitude_correction_raw",
    "altitude_correction", "altitude_saturated", "integral_update_accepted",
    "desired_roll_rad", "desired_pitch_rad", "desired_yaw_rad",
    "horizontal_position_error_x_m", "horizontal_position_error_y_m",
    "horizontal_velocity_error_x_m_s", "horizontal_velocity_error_y_m_s",
    "horizontal_position_x_contribution_m_s2", "horizontal_position_y_contribution_m_s2",
    "horizontal_velocity_x_contribution_m_s2", "horizontal_velocity_y_contribution_m_s2",
    "horizontal_acceleration_x_world_raw_m_s2", "horizontal_acceleration_y_world_raw_m_s2",
    "horizontal_acceleration_x_world_m_s2", "horizontal_acceleration_y_world_m_s2",
    "horizontal_acceleration_x_body_m_s2", "horizontal_acceleration_y_body_m_s2",
    "horizontal_acceleration_saturated", "tilt_reference_saturated",
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
                 controller_config: CascadedPIDConfig, experiment_mode: str | None = None) -> None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        mode = experiment_mode or ("cascaded-xy" if controller_config.stage == "5D" else "cascaded-5b")
        self.directory = Path(root)/f"{stamp}_{mode}"
        self.directory.mkdir(parents=True, exist_ok=False)
        self._stream = (self.directory/"flight_telemetry.csv").open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._stream, fieldnames=FIELDS)
        self._writer.writeheader(); self._stream.flush()
        self.metadata = {
            "schema_version": 1, "mode": mode, "status": "running",
            "started_utc": datetime.now(timezone.utc).isoformat(),
            "flight_config": asdict(config), "controller_config": asdict(controller_config),
            "controller": "calibration.cascaded_pid.CascadedPIDController",
            "command_interface": "BodyRateThrustCommand; firmware body-rate PID and mixer retained",
        }
        self._write_metadata()

    def _write_metadata(self) -> None:
        (self.directory/"metadata.json").write_text(
            json.dumps(self.metadata, indent=2, sort_keys=True)+"\n", encoding="utf-8")

    def persist_metadata(self) -> None:
        self._write_metadata()

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


@dataclass(frozen=True,slots=True)
class PitchIdentificationSettings:
    excitation_rad: float = 0.02
    maximum_excitation_rad: float = 0.02
    takeoff_end_s: float = 1.0
    safety_assessment_end_s: float = 1.2
    neutral_baseline_end_s: float = 1.4
    positive_pulse_end_s: float = 1.65
    positive_recovery_end_s: float = 1.9
    negative_pulse_end_s: float = 2.15
    landing_start_s: float = 2.5
    envelope_reserve_m: float = 0.05
    horizontal_speed_reserve_m_s: float = 0.10

    def __post_init__(self):
        if not 0 < self.excitation_rad <= self.maximum_excitation_rad:
            raise ValueError("pitch excitation exceeds conservative validated bound")


@dataclass(frozen=True,slots=True)
class RollIdentificationSettings(PitchIdentificationSettings):
    def __post_init__(self):
        if not 0 < self.excitation_rad <= self.maximum_excitation_rad:
            raise ValueError("roll excitation exceeds conservative validated bound")


class EstimatorReadinessSettings:
    """Stationary-estimator gates, separate from the in-flight safety envelope."""
    preflight_duration_s = 0.5
    max_preflight_position_radius_m = 0.05
    max_preflight_horizontal_speed_m_s = 0.15
    max_position_velocity_residual_m = 0.10


class CascadedStage5BRunner:
    LANDING_CONFIRMATION_S = 0.5
    LANDING_TIMEOUT_S = 4.0
    LANDING_MAX_RELATIVE_HEIGHT_M = 0.06
    LANDING_MAX_VERTICAL_SPEED_M_S = 0.08
    def __init__(self, hardware: CascadedHardware, config: CalibrationConfig,
                 controller_config: CascadedPIDConfig, logger: CascadedFlightLogger,
                 clock=time.monotonic, sleeper=time.sleep,
                 pitch_identification: PitchIdentificationSettings | None = None,
                 roll_identification: RollIdentificationSettings | None = None) -> None:
        self.hardware, self.config, self.controller_config, self.logger = hardware, config, controller_config, logger
        self.clock, self.sleeper = clock, sleeper
        self.pitch_identification = pitch_identification
        self.roll_identification = roll_identification
        if pitch_identification is not None and roll_identification is not None:
            raise ValueError("only one attitude-identification axis may be active")
        self.attitude_identification = pitch_identification or roll_identification
        self.identification_axis = "pitch" if pitch_identification is not None else (
            "roll" if roll_identification is not None else None)
        self.estimator_readiness = EstimatorReadinessSettings()
        self._preflight_last_state: AltitudeState | None = None
        if controller_config.stage not in {"5B", "5C", "5D"}:
            raise ValueError("flight runner requires a Stage 5B, 5C, or 5D controller config")
        if (controller_config.max_thrust > config.safety.max_thrust or
                controller_config.min_thrust < config.safety.min_thrust):
            raise ValueError("controller thrust bounds exceed flight safety bounds")
        if self.attitude_identification is not None:
            if controller_config.horizontal.enabled:
                raise ValueError("attitude identification requires XY control disabled")

    def run(self) -> None:
        status, reason = "aborted", "flight did not start"
        shutdown: dict[str, Any] = {}
        try:
            if self.attitude_identification is not None:
                settings = self.attitude_identification
                key = f"{self.identification_axis}_identification"
                self.logger.metadata[key] = {
                    "status":"not_started","excitation_performed":False,
                    "excitation_rad":settings.excitation_rad,
                    "maximum_excitation_rad":settings.maximum_excitation_rad,
                    "timeline_s":{"takeoff_end":settings.takeoff_end_s,
                        "safety_assessment_end":settings.safety_assessment_end_s,
                        "neutral_baseline_end":settings.neutral_baseline_end_s,
                        "positive_pulse_end":settings.positive_pulse_end_s,
                        "positive_recovery_end":settings.positive_recovery_end_s,
                        "negative_pulse_end":settings.negative_pulse_end_s,
                        "landing_start":settings.landing_start_s},
                    "pulse_margin_checks":{},"phases_performed":[]}
                self.logger._write_metadata()
            origin = self._preflight()
            controller = CascadedPIDController(self.controller_config)
            self.hardware.arm(); self.sleeper(.25)
            armed = self.hardware.supervisor_status()
            if not armed["is_armed"] or not armed["can_fly"]:
                raise HardSafetyError("arming_not_confirmed", "supervisor did not confirm armed readiness", {"supervisor": armed})
            previous_state = self._validate_post_arm_estimator(origin, armed)
            period = 1/self.config.control_frequency_hz
            start = previous = deadline = self.clock()
            landing_confirmation_start = None
            landing_timeout_start = None
            positive_allowed = None
            negative_allowed = None
            identification_phases_performed: set[str] = set()
            if self.attitude_identification is not None:
                key = f"{self.identification_axis}_identification"
                self.logger.metadata[key]["status"]="fixed_timeline_running"
                self.logger._write_metadata()
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
                self._check_estimator_continuity(previous_state, state, snapshot)
                previous_state = state
                self._check_safety(state, supervisor, snapshot)
                elapsed=now-start
                profile = (self._attitude_identification_profile(elapsed,origin[2])
                           if self.attitude_identification is not None else
                           reference_for(Mode.CASCADED_5B,elapsed,origin[2],self.config))
                phase, reference_altitude = profile.phase, profile.altitude_m
                if profile.complete:
                    if landing_timeout_start is None: landing_timeout_start = now
                    landing_confirmation_elapsed = now-landing_timeout_start
                    phase = "LANDING_CONFIRM"
                    # Continue the virtual descent below the measured ground plane.
                    # A ground-level reference left the P/PD loop at a positive
                    # airborne equilibrium in the first XY/Z flight.
                    reference_altitude = (origin[2] -
                        self.config.profile.landing_rate_m_s*landing_confirmation_elapsed)
                    landed = (state.altitude_m-origin[2] <= self.LANDING_MAX_RELATIVE_HEIGHT_M and
                              abs(state.vertical_velocity_m_s) <= self.LANDING_MAX_VERTICAL_SPEED_M_S)
                    if landed:
                        if landing_confirmation_start is None: landing_confirmation_start = now
                        if now-landing_confirmation_start >= self.LANDING_CONFIRMATION_S: break
                    else: landing_confirmation_start = None
                    if now-landing_timeout_start >= self.LANDING_TIMEOUT_S:
                        raise HardSafetyError("touchdown_not_confirmed", "touchdown was not confirmed before timeout", snapshot)
                pitch_reference = 0.0
                roll_reference = 0.0
                identification_phase = "DISABLED"
                identification_ready = 0
                if self.attitude_identification is not None:
                    settings = self.attitude_identification
                    axis = self.identification_axis
                    metadata = self.logger.metadata[f"{axis}_identification"]
                    positive_phase=f"{axis.upper()}_POSITIVE";negative_phase=f"{axis.upper()}_NEGATIVE"
                    if elapsed<settings.takeoff_end_s: identification_phase="DISABLED"
                    elif elapsed<settings.safety_assessment_end_s: identification_phase="SAFETY_ASSESSMENT"
                    elif elapsed<settings.neutral_baseline_end_s: identification_phase="NEUTRAL_BEFORE"
                    elif elapsed<settings.positive_pulse_end_s:
                        if positive_allowed is None:
                            positive_allowed=self._check_attitude_pulse_margin(snapshot,elapsed,
                                settings.positive_pulse_end_s-settings.neutral_baseline_end_s+
                                settings.positive_recovery_end_s-settings.positive_pulse_end_s,"positive")
                        identification_phase=positive_phase if positive_allowed else "SKIPPED_POSITIVE"
                        if positive_allowed:
                            if axis=="pitch": pitch_reference=settings.excitation_rad
                            else: roll_reference=settings.excitation_rad
                            identification_ready=1
                    elif elapsed<settings.positive_recovery_end_s: identification_phase="RECOVERY_POSITIVE"
                    elif elapsed<settings.negative_pulse_end_s:
                        if negative_allowed is None:
                            negative_allowed=self._check_attitude_pulse_margin(snapshot,elapsed,
                                settings.negative_pulse_end_s-settings.positive_recovery_end_s+
                                settings.landing_start_s-settings.negative_pulse_end_s,"negative")
                        identification_phase=negative_phase if negative_allowed else "SKIPPED_NEGATIVE"
                        if negative_allowed:
                            if axis=="pitch": pitch_reference=-settings.excitation_rad
                            else: roll_reference=-settings.excitation_rad
                            identification_ready=1
                    elif elapsed<settings.landing_start_s: identification_phase="RECOVERY_NEGATIVE"
                    else:
                        identification_phase="COMPLETE"
                    if identification_phase in {positive_phase,negative_phase}:
                        identification_phases_performed.add(identification_phase)
                    metadata["phases_performed"]=sorted(identification_phases_performed)
                    metadata["excitation_performed"]={positive_phase,negative_phase}.issubset(identification_phases_performed)
                    if elapsed>=settings.landing_start_s and metadata["status"] not in {"complete","inconclusive"}:
                        metadata.update(status="complete" if metadata["excitation_performed"] else "inconclusive",
                            completed_elapsed_s=elapsed)
                        self.logger._write_metadata()
                reference_velocity = (-self.config.profile.landing_rate_m_s
                                      if profile.complete else profile.vertical_velocity_m_s)
                reference = CascadedReference(reference_altitude, vertical_velocity_m_s=reference_velocity,
                                              roll_rad=roll_reference,pitch_rad=pitch_reference, yaw_rad=origin[3],
                                              position_x_m=origin[0], position_y_m=origin[1])
                result = controller.update(state, reference, dt)
                row = self._row(now-start, now, dt, phase, age, state,
                                reference, result, supervisor, origin,
                                identification_phase, pitch_reference, identification_ready)
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

    def _attitude_identification_profile(self,elapsed,initial_altitude):
        settings=self.attitude_identification
        if settings is None: raise RuntimeError("attitude identification settings are unavailable")
        height=self.config.profile.takeoff_height_m
        if elapsed<settings.takeoff_end_s:
            return ReferencePoint("TAKEOFF",initial_altitude+height*elapsed/settings.takeoff_end_s,
                                  vertical_velocity_m_s=height/settings.takeoff_end_s)
        if elapsed<settings.landing_start_s:
            return ReferencePoint("HOVER",initial_altitude+height)
        landing_elapsed=elapsed-settings.landing_start_s
        down_time=height/self.config.profile.landing_rate_m_s
        if landing_elapsed<down_time:
            return ReferencePoint("LANDING",initial_altitude+max(0,height-landing_elapsed*self.config.profile.landing_rate_m_s),
                                  vertical_velocity_m_s=-self.config.profile.landing_rate_m_s)
        if landing_elapsed<down_time+self.config.profile.settle_duration_s:
            return ReferencePoint("SETTLE",initial_altitude)
        return ReferencePoint("COMPLETE",initial_altitude,True)

    def _check_attitude_pulse_margin(self,snapshot,elapsed_s,horizon_s,label):
        settings=self.attitude_identification
        if settings is None: raise RuntimeError("attitude identification settings are unavailable")
        safety=self.config.safety
        displacement=snapshot["horizontal_displacement_m"];speed=snapshot["horizontal_speed_m_s"]
        remaining=safety.max_horizontal_displacement_m-displacement
        required=speed*horizon_s+settings.envelope_reserve_m
        speed_margin=safety.max_horizontal_speed_m_s-speed
        reasons=[]
        if remaining<required: reasons.append("insufficient_displacement_margin")
        if speed_margin<settings.horizontal_speed_reserve_m_s: reasons.append("insufficient_speed_margin")
        check={"elapsed_s":elapsed_s,"monotonic_s":snapshot["monotonic_s"],"horizontal_displacement_m":displacement,
            "horizontal_speed_m_s":speed,"planned_pulse_and_recovery_s":horizon_s,
            "remaining_displacement_m":remaining,"required_displacement_margin_m":required,
            "remaining_speed_margin_m_s":speed_margin,"passed":not reasons,"failure_reasons":reasons}
        self.logger.metadata[f"{self.identification_axis}_identification"]["pulse_margin_checks"][label]=check
        self.logger._write_metadata()
        return not reasons

    def _preflight(self) -> tuple[float, float, float, float]:
        if not self.hardware.emergency_stop_available(): raise RuntimeError("supervisor emergency stop unavailable")
        if self.hardware.protocol_version() < 12: raise RuntimeError("protocol version 12 or newer required")
        if not self.hardware.flow_deck_available(): raise RuntimeError("Flow Deck bcFlow/bcFlow2 is not active")
        status = self.hardware.supervisor_status()
        if (status["is_armed"] or status["is_flying"] or status["is_tumbled"] or status["is_locked"] or
                status["is_crashed"] or status["hl_control_active"] or not status["can_be_armed"]):
            raise HardSafetyError("preflight_supervisor_rejected", f"supervisor rejects flight: {status}", {"supervisor":status})
        samples=[]; end=self.clock()+self.estimator_readiness.preflight_duration_s
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
        radii=[math.hypot(s.position_x_m-origin[0],s.position_y_m-origin[1]) for s in samples]
        speeds=[math.hypot(s.velocity_x_m_s,s.velocity_y_m_s) for s in samples]
        readiness=self.estimator_readiness
        self.logger.metadata["captured_reference"]={"x_m":origin[0],"y_m":origin[1],"ground_z_m":origin[2],"yaw_rad":origin[3],"samples":len(samples)}
        self.logger.metadata["horizontal_estimator_readiness"]={
            "origin_method":"component-wise median of stationary preflight samples",
            "sample_count":len(samples),"duration_s":readiness.preflight_duration_s,
            "first_x_m":samples[0].position_x_m,"first_y_m":samples[0].position_y_m,
            "last_x_m":samples[-1].position_x_m,"last_y_m":samples[-1].position_y_m,
            "maximum_position_radius_m":max(radii),"maximum_horizontal_speed_m_s":max(speeds),
            "limit_position_radius_m":readiness.max_preflight_position_radius_m,
            "limit_horizontal_speed_m_s":readiness.max_preflight_horizontal_speed_m_s,
            "continuity_residual_limit_m":readiness.max_position_velocity_residual_m}
        self.logger._write_metadata()
        snapshot=self._snapshot(samples[-1],status,origin,self.clock())
        if max(radii)>readiness.max_preflight_position_radius_m:
            raise HardSafetyError("preflight_horizontal_position_unsettled",
                "preflight horizontal position is not settled",snapshot)
        if max(speeds)>readiness.max_preflight_horizontal_speed_m_s:
            raise HardSafetyError("preflight_horizontal_velocity_unsettled",
                "preflight horizontal velocity is not settled",snapshot)
        self._preflight_last_state=samples[-1]
        return origin

    def _validate_post_arm_estimator(self,origin,supervisor):
        state=self.hardware.state(); now=self.clock()
        snapshot=self._snapshot(state,supervisor,origin,now)
        try: validate_fresh_state(state,now,self.config.safety.max_state_age_s)
        except (RuntimeError,TimeoutError) as exc:
            raise HardSafetyError("post_arm_invalid_or_stale_state",str(exc),snapshot) from exc
        speed=snapshot["horizontal_speed_m_s"]
        if speed>self.estimator_readiness.max_preflight_horizontal_speed_m_s:
            raise HardSafetyError("post_arm_horizontal_velocity_unsettled",
                "horizontal estimator velocity changed before the first command",snapshot)
        if self._preflight_last_state is None:
            raise RuntimeError("missing preflight estimator state")
        self._check_estimator_continuity(self._preflight_last_state,state,snapshot,
                                         condition="post_arm_estimator_discontinuity")
        self._check_safety(state,supervisor,snapshot)
        self.logger.metadata["post_arm_estimator_check"]={
            "x_m":state.position_x_m,"y_m":state.position_y_m,
            "vx_m_s":state.velocity_x_m_s,"vy_m_s":state.velocity_y_m_s,
            "horizontal_speed_m_s":speed,"origin_preserved":True}
        self.logger._write_metadata()
        return state

    def _check_estimator_continuity(self,previous,state,snapshot,
                                    condition="estimator_position_discontinuity"):
        dt=state.timestamp_s-previous.timestamp_s
        if dt<=0:
            return
        predicted_x=.5*(previous.velocity_x_m_s+state.velocity_x_m_s)*dt
        predicted_y=.5*(previous.velocity_y_m_s+state.velocity_y_m_s)*dt
        observed_x=state.position_x_m-previous.position_x_m
        observed_y=state.position_y_m-previous.position_y_m
        residual=math.hypot(observed_x-predicted_x,observed_y-predicted_y)
        snapshot["estimator_continuity"]={
            "dt_s":dt,"observed_dx_m":observed_x,"observed_dy_m":observed_y,
            "velocity_integrated_dx_m":predicted_x,"velocity_integrated_dy_m":predicted_y,
            "position_velocity_residual_m":residual,
            "limit_m":self.estimator_readiness.max_position_velocity_residual_m}
        if residual>self.estimator_readiness.max_position_velocity_residual_m:
            raise HardSafetyError(condition,
                "horizontal position changed inconsistently with measured velocity",snapshot)

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

    def _row(self,elapsed,now,dt,phase,age,state,reference,result,supervisor,origin,
             identification_phase="DISABLED",pitch_excitation=0.0,identification_ready=0):
        c=result.command;s=self.config.safety;a=self.controller_config.attitude
        return {"elapsed_s":elapsed,"monotonic_s":now,"dt_s":dt,"phase":phase,"state_source":state.source,"state_age_s":age,
            "pitch_identification_phase":identification_phase if self.identification_axis!="roll" else "DISABLED","pitch_excitation_reference_rad":pitch_excitation if self.identification_axis!="roll" else 0.0,"pitch_identification_ready":identification_ready if self.identification_axis!="roll" else 0,
            "roll_identification_phase":identification_phase if self.identification_axis=="roll" else "DISABLED","roll_excitation_reference_rad":reference.roll_rad if self.identification_axis=="roll" else 0.0,"roll_identification_ready":identification_ready if self.identification_axis=="roll" else 0,
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
