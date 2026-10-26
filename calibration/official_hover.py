from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean, median
import time
from typing import Any, Protocol
from importlib.metadata import version

from .config import CalibrationConfig
from .state import AltitudeState, validate_fresh_state


FIELDS = ["elapsed_s", "monotonic_s", "dt_s", "phase", "reference_altitude_m",
          "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s", "roll_rad",
          "pitch_rad", "yaw_rad", "state_timestamp_s", "state_age_s",
          "onboard_thrust_raw", "onboard_thrust_age_s", "can_be_armed", "can_fly",
          "is_armed", "is_auto_armed", "is_flying", "is_tumbled", "is_locked",
          "is_crashed", "hl_control_active"]


@dataclass(frozen=True, slots=True)
class OfficialHoverSettings:
    target_altitude_m: float = 0.30
    hover_duration_s: float = 3.0
    takeoff_rate_m_s: float = 0.15
    landing_rate_m_s: float = 0.12
    command_frequency_hz: float = 50.0
    start_height_m: float = 0.05
    landing_height_m: float = 0.03
    landing_confirmation_s: float = 0.5
    landing_timeout_s: float = 3.0
    landing_max_height_m: float = 0.06
    landing_max_vertical_speed_m_s: float = 0.08
    estimator_check_s: float = 0.5
    max_horizontal_displacement_m: float = 0.50
    max_horizontal_speed_m_s: float = 0.75

    def validate(self, config: CalibrationConfig) -> None:
        values = asdict(self)
        if any(not math.isfinite(value) or value <= 0 for value in values.values()):
            raise ValueError("official hover settings must be finite and positive")
        if self.target_altitude_m > config.safety.max_altitude_m:
            raise ValueError("target altitude exceeds configured altitude limit")
        if max(self.takeoff_rate_m_s, self.landing_rate_m_s) > config.safety.max_vertical_speed_m_s:
            raise ValueError("profile velocity exceeds configured vertical-speed limit")
        if 1/self.command_frequency_hz > config.safety.max_loop_dt_s:
            raise ValueError("command period exceeds configured loop-dt limit")
        if self.landing_height_m > self.start_height_m:
            raise ValueError("landing height must not exceed start height")
        if self.landing_max_height_m < self.landing_height_m:
            raise ValueError("landing confirmation height must cover landing height")


class OfficialHardware(Protocol):
    def state(self) -> AltitudeState: ...
    def supervisor_status(self) -> dict[str, bool]: ...
    def flow_deck_available(self) -> bool: ...
    def emergency_stop_available(self) -> bool: ...
    def protocol_version(self) -> int: ...
    def start_onboard_thrust_log(self, period_ms: int = 20) -> bool: ...
    def onboard_thrust_raw(self) -> tuple[float | None, float | None]: ...
    def arm(self) -> None: ...
    def disarm(self) -> None: ...
    def send_hover_setpoint(self, height_m: float) -> None: ...
    def notify_setpoint_stop(self) -> None: ...
    def emergency_stop(self) -> None: ...
    def supervisor_emergency_stop(self) -> None: ...


class HardSafetyError(RuntimeError):
    pass


class OfficialHoverLogger:
    def __init__(self, root: str | Path, config: CalibrationConfig, settings: OfficialHoverSettings) -> None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        self.directory = Path(root)/f"{stamp}_official-hover-baseline"
        self.directory.mkdir(parents=True, exist_ok=False)
        self._stream = (self.directory/"flight_telemetry.csv").open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._stream, fieldnames=FIELDS); self._writer.writeheader()
        self.metadata = {"schema_version": 1, "mode": "official-hover-baseline", "status": "running",
                         "started_utc": datetime.now(timezone.utc).isoformat(), "config": asdict(config),
                         "official_hover": asdict(settings),
                         "cflib_version": version("cflib"),
                         "onboard_thrust_semantics": "stabilizer.thrust raw firmware collective-control value; not normalized thrust, force, or independently calibrated hover thrust"}
        self._write_metadata()
    def _write_metadata(self) -> None:
        (self.directory/"metadata.json").write_text(json.dumps(self.metadata, indent=2, sort_keys=True)+"\n", encoding="utf-8")
    def persist_metadata(self) -> None:
        self._write_metadata()
    def row(self, row: dict[str, Any]) -> None:
        self._writer.writerow(row); self._stream.flush()
    def close(self, status: str, reason: str, landing_status: str) -> None:
        if self._stream.closed: return
        self._stream.close(); self.metadata.update(status=status, termination_reason=reason,
                                                    landing_status=landing_status,
                                                    ended_utc=datetime.now(timezone.utc).isoformat()); self._write_metadata()


class OfficialHoverRunner:
    def __init__(self, hardware: OfficialHardware, config: CalibrationConfig, logger: OfficialHoverLogger,
                 settings: OfficialHoverSettings | None = None, clock=time.monotonic, sleeper=time.sleep,
                 output=print) -> None:
        self.hardware, self.config, self.logger = hardware, config, logger
        self.settings = settings or OfficialHoverSettings(); self.settings.validate(config)
        self.clock, self.sleeper, self.output = clock, sleeper, output
        self._last_state: AltitudeState | None = None; self._armed = False
        self._origin_xy = (0.0, 0.0)
        self._ground_z = 0.0

    @staticmethod
    def _horizontal_values(state: AltitudeState, origin_xy: tuple[float, float]) -> dict[str, float]:
        dx = state.position_x_m-origin_xy[0]
        dy = state.position_y_m-origin_xy[1]
        return {
            "x_m": state.position_x_m, "y_m": state.position_y_m,
            "origin_x_m": origin_xy[0], "origin_y_m": origin_xy[1],
            "displacement_x_m": dx, "displacement_y_m": dy,
            "horizontal_displacement_m": math.hypot(dx, dy),
            "vx_m_s": state.velocity_x_m_s, "vy_m_s": state.velocity_y_m_s,
            "horizontal_speed_m_s": math.hypot(state.velocity_x_m_s, state.velocity_y_m_s),
        }

    def _record_safety_event(self, condition: str, state: AltitudeState,
                             values: dict[str, Any]) -> None:
        self.logger.metadata["safety_event"] = {
            "condition": condition,
            "recorded_utc": datetime.now(timezone.utc).isoformat(),
            "state": {
                "z_m": state.altitude_m, "vz_m_s": state.vertical_velocity_m_s,
                "roll_rad": state.roll_rad, "pitch_rad": state.pitch_rad,
                "state_timestamp_s": state.timestamp_s,
                **values,
            },
        }
        # Persist before raising: an emergency stop or link loss may follow immediately.
        self.logger._write_metadata()

    def run(self) -> dict[str, Any]:
        self._preflight()
        thrust_available = self.hardware.start_onboard_thrust_log(period_ms=20)
        self.logger.metadata["onboard_thrust_available"] = thrust_available
        start = self.clock(); previous = start
        status, reason, landing_status = "complete", "normal completion", "not_started"
        try:
            self.hardware.arm(); self._armed = True
            self.sleeper(0.25)
            armed_status = self.hardware.supervisor_status()
            if not armed_status["is_armed"] or not armed_status["can_fly"]:
                raise HardSafetyError(f"supervisor did not confirm armed flight readiness: {armed_status}")
            previous = self.clock()
            takeoff_s = (self.settings.target_altitude_m-self.settings.start_height_m)/self.settings.takeoff_rate_m_s
            previous = self._run_phase("TAKEOFF", start, previous, takeoff_s,
                lambda elapsed: self.settings.start_height_m+elapsed*self.settings.takeoff_rate_m_s)
            previous = self._run_phase("HOVER", start, previous, self.settings.hover_duration_s,
                lambda _: self.settings.target_altitude_m)
            self._controlled_landing(start, previous, self.settings.target_altitude_m)
            landing_status = "controlled_landing_complete"
            self._normal_stop_and_disarm(); self._armed = False
        except KeyboardInterrupt:
            status, reason, landing_status = "interrupted", "Ctrl-C/operator stop", "immediate_lift_termination"
            self._immediate_stop()
        except HardSafetyError as exc:
            status, reason, landing_status = "aborted", f"hard safety: {exc}", "emergency_motor_shutdown"
            self._supervisor_estop()
        except BaseException as exc:
            status, reason = "aborted", f"recoverable fault: {type(exc).__name__}: {exc}"
            if self._can_attempt_landing():
                try:
                    relative_height = max(self.settings.start_height_m, self._last_state.altitude_m-self._ground_z)
                    self._controlled_landing(start, previous, relative_height)
                    landing_status = "recoverable_controlled_landing_complete"
                    self._normal_stop_and_disarm(); self._armed = False
                except BaseException as landing_exc:
                    reason += f"; landing failed: {type(landing_exc).__name__}: {landing_exc}"
                    landing_status = "emergency_motor_shutdown_after_landing_failure"; self._supervisor_estop()
            else:
                landing_status = "immediate_motor_shutdown"; self._immediate_stop()
        finally:
            self.logger.close(status, reason, landing_status)
        return analyze_official_hover(self.logger.directory)

    def _preflight(self) -> None:
        if not self.hardware.emergency_stop_available(): raise RuntimeError("supervisor emergency stop unavailable")
        if self.hardware.protocol_version() < 12: raise RuntimeError("official hover baseline requires Crazyflie protocol version 12 or newer for supervisor safety")
        if not self.hardware.flow_deck_available(): raise RuntimeError("Flow Deck bcFlow/bcFlow2 is not active")
        status = self.hardware.supervisor_status()
        unsafe = status["is_armed"] or status["is_flying"] or status["is_tumbled"] or status["is_locked"] or status["is_crashed"] or status["hl_control_active"] or not status["can_be_armed"]
        if unsafe: raise RuntimeError(f"supervisor rejects flight: {status}")
        samples=[]; end=self.clock()+self.settings.estimator_check_s
        while self.clock() < end:
            state=self.hardware.state(); validate_fresh_state(state, self.clock(), self.config.safety.max_state_age_s)
            samples.append(state); self.sleeper(min(0.02, max(0, end-self.clock())))
        if len(samples)<2: raise RuntimeError("insufficient estimator readiness samples")
        if max(s.altitude_m for s in samples)-min(s.altitude_m for s in samples)>0.10: raise RuntimeError("estimator altitude is not settled")
        if max(max(abs(s.roll_rad),abs(s.pitch_rad)) for s in samples)>0.20: raise RuntimeError("vehicle is not level for takeoff")
        self._origin_xy = (median(s.position_x_m for s in samples),
                           median(s.position_y_m for s in samples))
        self._ground_z = median(s.altitude_m for s in samples)
        self.logger.metadata["ground_altitude_m"] = self._ground_z
        horizontal = [self._horizontal_values(s, self._origin_xy) for s in samples]
        self.logger.metadata["horizontal_preflight"] = {
            "sample_count": len(samples),
            "origin_method": "component-wise median of stationary preflight samples",
            "origin_x_m": self._origin_xy[0], "origin_y_m": self._origin_xy[1],
            "latest_x_m": samples[-1].position_x_m, "latest_y_m": samples[-1].position_y_m,
            "latest_vx_m_s": samples[-1].velocity_x_m_s,
            "latest_vy_m_s": samples[-1].velocity_y_m_s,
            "latest_horizontal_speed_m_s": horizontal[-1]["horizontal_speed_m_s"],
            "maximum_horizontal_displacement_m": max(v["horizontal_displacement_m"] for v in horizontal),
            "maximum_horizontal_speed_m_s": max(v["horizontal_speed_m_s"] for v in horizontal),
        }
        self.logger.metadata["height_reference_semantics"] = "hover zDistance is absolute world-frame Z; profile heights are offsets from the preflight median ground Z"
        self.logger._write_metadata()
        worst_displacement = max(horizontal, key=lambda value: value["horizontal_displacement_m"])
        if worst_displacement["horizontal_displacement_m"] > self.settings.max_horizontal_displacement_m:
            self._record_safety_event("horizontal_displacement_limit_exceeded_preflight", samples[horizontal.index(worst_displacement)], {
                **worst_displacement, "limit_m": self.settings.max_horizontal_displacement_m})
            raise RuntimeError("preflight horizontal displacement limit exceeded: "
                               f'{worst_displacement["horizontal_displacement_m"]:.3f} m > {self.settings.max_horizontal_displacement_m:.3f} m')
        worst_speed = max(horizontal, key=lambda value: value["horizontal_speed_m_s"])
        if worst_speed["horizontal_speed_m_s"] > self.settings.max_horizontal_speed_m_s:
            self._record_safety_event("horizontal_speed_limit_exceeded_preflight", samples[horizontal.index(worst_speed)], {
                **worst_speed, "limit_m_s": self.settings.max_horizontal_speed_m_s})
            raise RuntimeError("preflight horizontal speed limit exceeded: "
                               f'{worst_speed["horizontal_speed_m_s"]:.3f} m/s > {self.settings.max_horizontal_speed_m_s:.3f} m/s')

    def _run_phase(self, phase: str, experiment_start: float, previous: float, duration: float, reference) -> float:
        phase_start=self.clock(); deadline=phase_start
        while True:
            now=self.clock(); elapsed=now-phase_start
            if elapsed>=duration: return previous
            dt=now-previous; previous=now
            if dt>self.config.safety.max_loop_dt_s: raise HardSafetyError(f"control loop dt {dt:.3f}s")
            state=self.hardware.state(); self._last_state=state
            try: age=validate_fresh_state(state,now,self.config.safety.max_state_age_s)
            except (RuntimeError,TimeoutError) as exc: raise HardSafetyError(str(exc)) from exc
            supervisor=self.hardware.supervisor_status(); self._validate_inflight(state,supervisor)
            target=self._ground_z+min(reference(elapsed),self.settings.target_altitude_m)
            self.hardware.send_hover_setpoint(target)
            thrust,thrust_time=self.hardware.onboard_thrust_raw()
            row={"elapsed_s":now-experiment_start,"monotonic_s":now,"dt_s":dt,"phase":phase,
                 "reference_altitude_m":target,"x_m":state.position_x_m,"y_m":state.position_y_m,"z_m":state.altitude_m,
                 "vx_m_s":state.velocity_x_m_s,"vy_m_s":state.velocity_y_m_s,"vz_m_s":state.vertical_velocity_m_s,
                 "roll_rad":state.roll_rad,"pitch_rad":state.pitch_rad,"yaw_rad":state.yaw_rad,
                 "state_timestamp_s":state.timestamp_s,"state_age_s":age,"onboard_thrust_raw":thrust if thrust is not None else "",
                 "onboard_thrust_age_s":now-thrust_time if thrust_time is not None else ""}
            row.update({name:int(bool(supervisor[name])) for name in FIELDS if name in supervisor}); self.logger.row(row)
            deadline+=1/self.settings.command_frequency_hz; delay=deadline-self.clock()
            if delay>0:self.sleeper(delay)
            else:deadline=self.clock()

    def _controlled_landing(self, start: float, previous: float, from_height: float) -> None:
        duration=max(0.01,(from_height-self.settings.landing_height_m)/self.settings.landing_rate_m_s)
        previous=self._run_phase("LANDING",start,previous,duration,
            lambda elapsed:max(self.settings.landing_height_m,from_height-elapsed*self.settings.landing_rate_m_s))
        self._confirm_touchdown(start, previous)

    def _confirm_touchdown(self, experiment_start: float, previous: float) -> None:
        phase_start=self.clock(); deadline=phase_start; stable_since:float|None=None
        while self.clock()-phase_start < self.settings.landing_timeout_s:
            now=self.clock(); dt=now-previous; previous=now
            if dt>self.config.safety.max_loop_dt_s: raise HardSafetyError(f"control loop dt {dt:.3f}s")
            state=self.hardware.state(); self._last_state=state
            try: age=validate_fresh_state(state,now,self.config.safety.max_state_age_s)
            except (RuntimeError,TimeoutError) as exc: raise HardSafetyError(str(exc)) from exc
            supervisor=self.hardware.supervisor_status(); self._validate_inflight(state,supervisor)
            target=self._ground_z+self.settings.landing_height_m
            self.hardware.send_hover_setpoint(target)
            thrust,thrust_time=self.hardware.onboard_thrust_raw()
            row={"elapsed_s":now-experiment_start,"monotonic_s":now,"dt_s":dt,"phase":"LANDING_CONFIRM",
                 "reference_altitude_m":target,"x_m":state.position_x_m,"y_m":state.position_y_m,"z_m":state.altitude_m,
                 "vx_m_s":state.velocity_x_m_s,"vy_m_s":state.velocity_y_m_s,"vz_m_s":state.vertical_velocity_m_s,
                 "roll_rad":state.roll_rad,"pitch_rad":state.pitch_rad,"yaw_rad":state.yaw_rad,
                 "state_timestamp_s":state.timestamp_s,"state_age_s":age,"onboard_thrust_raw":thrust if thrust is not None else "",
                 "onboard_thrust_age_s":now-thrust_time if thrust_time is not None else ""}
            row.update({name:int(bool(supervisor[name])) for name in FIELDS if name in supervisor}); self.logger.row(row)
            landed=(state.altitude_m-self._ground_z<=self.settings.landing_max_height_m and
                    abs(state.vertical_velocity_m_s)<=self.settings.landing_max_vertical_speed_m_s)
            stable_since = stable_since if landed and stable_since is not None else (now if landed else None)
            if stable_since is not None and now-stable_since>=self.settings.landing_confirmation_s:return
            deadline+=1/self.settings.command_frequency_hz; delay=deadline-self.clock()
            if delay>0:self.sleeper(delay)
            else:deadline=self.clock()
        raise HardSafetyError("touchdown was not confirmed before landing timeout")

    def _validate_inflight(self,state:AltitudeState,status:dict[str,bool])->None:
        if status["is_tumbled"] or status["is_locked"] or status["is_crashed"] or not status["is_armed"]: raise HardSafetyError(f"unsafe supervisor state: {status}")
        if abs(state.roll_rad)>self.config.safety.max_tilt_rad or abs(state.pitch_rad)>self.config.safety.max_tilt_rad: raise HardSafetyError("tilt limit exceeded")
        relative_altitude=state.altitude_m-self._ground_z
        if relative_altitude>self.config.safety.max_altitude_m or relative_altitude < -0.10 or abs(state.vertical_velocity_m_s)>self.config.safety.max_vertical_speed_m_s: raise HardSafetyError("altitude/vertical-speed limit exceeded")
        horizontal = self._horizontal_values(state, self._origin_xy)
        displacement = horizontal["horizontal_displacement_m"]
        horizontal_speed = horizontal["horizontal_speed_m_s"]
        if displacement>self.settings.max_horizontal_displacement_m:
            self._record_safety_event("horizontal_displacement_limit_exceeded", state, {
                **horizontal, "limit_m": self.settings.max_horizontal_displacement_m})
            raise HardSafetyError(f"horizontal displacement limit exceeded: {displacement:.3f} m > {self.settings.max_horizontal_displacement_m:.3f} m")
        if horizontal_speed>self.settings.max_horizontal_speed_m_s:
            self._record_safety_event("horizontal_speed_limit_exceeded", state, {
                **horizontal, "limit_m_s": self.settings.max_horizontal_speed_m_s})
            raise HardSafetyError(f"horizontal speed limit exceeded: {horizontal_speed:.3f} m/s > {self.settings.max_horizontal_speed_m_s:.3f} m/s")

    def _can_attempt_landing(self)->bool:
        if not self._armed or self._last_state is None:return False
        age=self.clock()-self._last_state.timestamp_s
        relative_altitude=self._last_state.altitude_m-self._ground_z
        return age<=self.config.safety.max_state_age_s and abs(self._last_state.roll_rad)<=self.config.safety.max_tilt_rad and abs(self._last_state.pitch_rad)<=self.config.safety.max_tilt_rad and -.10<=relative_altitude<=self.config.safety.max_altitude_m
    def _normal_stop_and_disarm(self)->None:
        self.hardware.emergency_stop(); self.hardware.notify_setpoint_stop(); self.hardware.disarm()
    def _immediate_stop(self)->None:
        self.hardware.emergency_stop(); self.hardware.disarm(); self._armed=False
    def _supervisor_estop(self)->None:
        for _ in range(3):
            try:self.hardware.supervisor_emergency_stop()
            except Exception:pass
        self._armed=False


def analyze_official_hover(run_directory: str|Path)->dict[str,Any]:
    run=Path(run_directory); metadata=json.loads((run/"metadata.json").read_text(encoding="utf-8"))
    with (run/"flight_telemetry.csv").open(newline="",encoding="utf-8") as stream:rows=list(csv.DictReader(stream))
    hover=[r for r in rows if r["phase"]=="HOVER"]
    errors=[float(r["z_m"])-float(r["reference_altitude_m"]) for r in hover]
    target=metadata.get("ground_altitude_m",0.0)+metadata["official_hover"]["target_altitude_m"]
    metrics={"software_execution_status":metadata["status"],"physically_validated":False,
             "sample_count":len(rows),"hover_sample_count":len(hover),
             "hover_tracking_rmse_m":math.sqrt(mean(e*e for e in errors)) if errors else None,
             "maximum_overshoot_m":max((float(r["z_m"])-target for r in rows),default=None),
             "maximum_observed_tilt_rad":max((max(abs(float(r["roll_rad"])),abs(float(r["pitch_rad"]))) for r in rows),default=None),
             "landing_status":metadata.get("landing_status"),"termination_reason":metadata.get("termination_reason"),
             "performance_pass":metadata["status"]=="complete" and metadata.get("landing_status")=="controlled_landing_complete" and bool(hover)}
    (run/"metrics.json").write_text(json.dumps(metrics,indent=2,sort_keys=True)+"\n",encoding="utf-8")
    if rows:_plot(run,rows)
    return metrics


def _plot(run:Path,rows:list[dict[str,str]])->None:
    panels=[("altitude [m]",[("reference_altitude_m","#2563eb"),("z_m","#111827")]),("vertical velocity [m/s]",[("vz_m_s","#059669")]),("attitude [rad]",[("roll_rad","#dc2626"),("pitch_rad","#2563eb"),("yaw_rad","#7c3aed")])]
    t=[float(r["elapsed_s"]) for r in rows]; width,height,margin=1000,570,70; duration=max(t[-1]-t[0],1e-9); svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">','<rect width="100%" height="100%" fill="white"/>']
    for i,(label,series) in enumerate(panels):
        top=20+i*175;bottom=top+140; values=[float(r[k]) for k,_ in series for r in rows];lo,hi=min(values),max(values)
        if lo==hi:lo,hi=lo-.5,hi+.5
        svg += [f'<rect x="{margin}" y="{top}" width="900" height="140" fill="none" stroke="#ccc"/>',f'<text x="5" y="{top+15}" font-family="sans-serif" font-size="11">{label}</text>']
        for key,color in series:
            pts=[f'{margin+(x-t[0])/duration*900:.1f},{bottom-(float(r[key])-lo)/(hi-lo)*140:.1f}' for x,r in zip(t,rows)];svg.append(f'<polyline points="{" ".join(pts)}" fill="none" stroke="{color}"/>')
        previous=None
        for x,r in zip(t,rows):
            if r["phase"]!=previous:
                px=margin+(x-t[0])/duration*900;svg.append(f'<line x1="{px:.1f}" y1="{top}" x2="{px:.1f}" y2="{bottom}" stroke="#999" stroke-dasharray="3 3"/>');previous=r["phase"]
    svg.append('</svg>');(run/"flight_response.svg").write_text("\n".join(svg)+"\n",encoding="utf-8")
