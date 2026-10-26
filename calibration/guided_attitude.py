from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import median
import time
from typing import Any, Callable

from .config import CalibrationConfig
from .state import validate_fresh_state
from .state_monitor import MONITOR_FIELDS, MonitorHardware, StateMonitorLogger, StateMonitorSettings, _write_monitor_plot


@dataclass(frozen=True, slots=True)
class GuidedPhase:
    name: str
    prompt: str
    axis: str
    expected_sign: int


GUIDED_PHASES = (
    GuidedPhase("neutral", "Keep the Crazyflie level and still.", "neutral", 0),
    GuidedPhase("left-side-down-roll", "Lower the LEFT side, hold briefly, then return to neutral.", "roll_rad", -1),
    GuidedPhase("right-side-down-roll", "Lower the RIGHT side, hold briefly, then return to neutral.", "roll_rad", 1),
    GuidedPhase("nose-up-pitch", "Raise the NOSE, hold briefly, then return to neutral.", "pitch_rad", 1),
    GuidedPhase("counterclockwise-yaw", "Rotate COUNTERCLOCKWISE viewed from above, hold briefly, then return.", "yaw_rad", 1),
    GuidedPhase("clockwise-yaw", "Rotate CLOCKWISE viewed from above, hold briefly, then return.", "yaw_rad", -1),
)


class GuidedAttitudeRunner:
    """Prompted read-only monitor; deliberately has no command or arm path."""

    def __init__(self, hardware: MonitorHardware, config: CalibrationConfig, logger: StateMonitorLogger,
                 settings: StateMonitorSettings, clock=time.monotonic, sleeper=time.sleep,
                 prompt: Callable[[str], None] = print) -> None:
        self.hardware, self.config, self.logger, self.settings = hardware, config, logger, settings
        self.clock, self.sleeper, self.prompt = clock, sleeper, prompt
        self.settings.validate()
        expected_duration = len(GUIDED_PHASES) * 5.0
        if not math.isclose(settings.duration_s, expected_duration):
            raise ValueError(f"guided attitude monitor duration must be {expected_duration:.1f} seconds")

    def run(self) -> dict[str, Any]:
        period, phase_duration = 1/self.settings.frequency_hz, 5.0
        start = previous = deadline = self.clock()
        previous_phase = -1
        status, reason = "complete", "guided attitude sequence completed"
        try:
            while True:
                now = self.clock(); elapsed = now-start
                if elapsed >= self.settings.duration_s:
                    break
                phase_index = min(int(elapsed/phase_duration), len(GUIDED_PHASES)-1)
                phase = GUIDED_PHASES[phase_index]
                if phase_index != previous_phase:
                    self.prompt(f"[PHASE {phase_index+1}/{len(GUIDED_PHASES)} | {phase_index*5:02d}-{(phase_index+1)*5:02d}s] {phase.prompt}")
                    previous_phase = phase_index
                interval = now-previous; previous = now
                row: dict[str, Any] = dict.fromkeys(MONITOR_FIELDS, "")
                row.update(elapsed_s=elapsed, sample_interval_s=interval, phase_index=phase_index,
                           phase=phase.name, physical_action=phase.prompt, expected_axis=phase.axis,
                           expected_sign=phase.expected_sign, valid=0, error="")
                try:
                    supervisor = self.hardware.supervisor_status()
                    row.update(**{name: int(bool(supervisor[name])) for name in MONITOR_FIELDS if name in supervisor})
                    state = self.hardware.state(); age = now-state.timestamp_s
                    row.update(altitude_m=state.altitude_m, vertical_velocity_m_s=state.vertical_velocity_m_s,
                               roll_rad=state.roll_rad, pitch_rad=state.pitch_rad, yaw_rad=state.yaw_rad,
                               state_timestamp_s=state.timestamp_s, state_age_s=age)
                    validate_fresh_state(state, now, self.config.safety.max_state_age_s)
                    row["valid"] = 1
                except (RuntimeError, TimeoutError, KeyError, ValueError) as exc:
                    row["error"] = f"{type(exc).__name__}: {exc}"
                self.logger.row(row)
                deadline += period
                delay = deadline-self.clock()
                if delay > 0: self.sleeper(delay)
                else: deadline = self.clock()
        except KeyboardInterrupt:
            status, reason = "interrupted", "operator interrupted with Ctrl-C"
        except BaseException as exc:
            status, reason = "invalid", f"{type(exc).__name__}: {exc}"
            raise
        finally:
            self.logger.close(status, reason)
        return analyze_guided_attitude(self.logger.directory)


def _angle_delta(value: float, baseline: float, yaw: bool) -> float:
    delta = value-baseline
    return math.atan2(math.sin(delta), math.cos(delta)) if yaw else delta


def analyze_guided_attitude(run_directory: str | Path, movement_threshold_rad: float = math.radians(5)) -> dict[str, Any]:
    run = Path(run_directory)
    metadata = json.loads((run/"metadata.json").read_text(encoding="utf-8"))
    with (run/"state_telemetry.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    valid = [row for row in rows if row["valid"] == "1"]
    neutral = [row for row in valid if row["phase"] == "neutral" and float(row["elapsed_s"]) >= 1.0]
    baselines = {axis: median(float(row[axis]) for row in neutral) for axis in ("roll_rad", "pitch_rad", "yaw_rad")} if neutral else {}
    summaries = []
    for phase in GUIDED_PHASES:
        phase_rows = [row for row in valid if row["phase"] == phase.name]
        if phase.name == "neutral":
            maximum = max((abs(float(row[axis])-baselines[axis]) for row in phase_rows for axis in baselines), default=None)
            summaries.append({"physical_action": phase.name, "axis": "neutral", "expected_sign": "near-zero",
                              "measured_sign": "near-zero" if maximum is not None and maximum < movement_threshold_rad else "movement-detected",
                              "peak_angle_rad": maximum, "peak_angle_deg": math.degrees(maximum) if maximum is not None else None,
                              "returned_to_neutral": None, "result": "PASS" if maximum is not None and maximum < movement_threshold_rad else "FAIL"})
            continue
        axis = phase.axis; yaw = axis == "yaw_rad"
        deltas = [_angle_delta(float(row[axis]), baselines[axis], yaw) for row in phase_rows] if baselines and phase_rows else []
        expected_peak = max((phase.expected_sign*value for value in deltas), default=float("-inf"))
        opposite_peak = max((-phase.expected_sign*value for value in deltas), default=float("-inf"))
        dominant_index = max(range(len(deltas)), key=lambda index: abs(deltas[index])) if deltas else None
        peak_delta = deltas[dominant_index] if dominant_index is not None else None
        peak_angle = float(phase_rows[dominant_index][axis]) if dominant_index is not None else None
        measured_sign = "positive" if peak_delta is not None and peak_delta > movement_threshold_rad else "negative" if peak_delta is not None and peak_delta < -movement_threshold_rad else "not-significant"
        expected_name = "positive" if phase.expected_sign > 0 else "negative"
        match = measured_sign == expected_name
        tail = [row for row in phase_rows if float(row["elapsed_s"]) >= (GUIDED_PHASES.index(phase)+1)*5-1]
        tail_delta = median([abs(_angle_delta(float(row[axis]), baselines[axis], yaw)) for row in tail]) if tail else None
        summaries.append({"physical_action": phase.name, "axis": axis, "expected_sign": expected_name,
                          "measured_sign": measured_sign, "peak_angle_rad": peak_angle,
                          "peak_angle_deg": math.degrees(peak_angle) if peak_angle is not None else None,
                          "peak_delta_from_neutral_rad": peak_delta,
                          "opposite_direction_peak_rad": opposite_peak if opposite_peak != float("-inf") else None,
                          "returned_to_neutral": tail_delta is not None and tail_delta < movement_threshold_rad,
                          "result": "PASS" if match else "FAIL"})
    complete = metadata.get("status") == "complete" and len(summaries) == len(GUIDED_PHASES)
    report = {"schema_version": 1, "source_run": run.name, "status": metadata.get("status"),
              "coordinate_conventions": {"left-side-down-roll": "negative", "right-side-down-roll": "positive",
                "nose-up-pitch": "positive", "counterclockwise-yaw-viewed-from-above": "positive",
                "clockwise-yaw-viewed-from-above": "negative"},
              "movement_threshold_rad": movement_threshold_rad, "movement_threshold_deg": math.degrees(movement_threshold_rad),
              "valid_samples": len(valid), "total_samples": len(rows), "phase_results": summaries,
              "overall_result": "PASS" if complete and all(item["result"] == "PASS" for item in summaries) else "FAIL"}
    (run/"attitude_sign_summary.json").write_text(json.dumps(report, indent=2, sort_keys=True)+"\n", encoding="utf-8")
    if valid:
        _write_monitor_plot(run, valid)
    return report
