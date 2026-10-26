from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean
import time
from typing import Any, Protocol

from .config import CalibrationConfig
from .state import AltitudeState, validate_fresh_state


MONITOR_FIELDS = [
    "elapsed_s", "sample_interval_s", "phase_index", "phase", "physical_action",
    "expected_axis", "expected_sign", "altitude_m", "vertical_velocity_m_s",
    "roll_rad", "pitch_rad", "yaw_rad", "state_timestamp_s", "state_age_s",
    "valid", "error", "can_be_armed", "can_fly", "is_armed",
    "is_auto_armed", "is_flying", "is_tumbled", "is_locked", "is_crashed",
    "hl_control_active",
]


@dataclass(frozen=True, slots=True)
class StateMonitorSettings:
    frequency_hz: float = 50.0
    duration_s: float = 20.0
    stationary_vertical_speed_m_s: float = 0.05
    stationary_tilt_rad: float = 0.10
    minimum_stationary_interval_s: float = 2.0

    def validate(self) -> None:
        if self.frequency_hz <= 0 or self.duration_s <= 0:
            raise ValueError("monitor frequency and duration must be positive")
        if self.stationary_vertical_speed_m_s < 0 or self.stationary_tilt_rad < 0:
            raise ValueError("stationary thresholds must be nonnegative")
        if self.minimum_stationary_interval_s <= 0:
            raise ValueError("minimum stationary interval must be positive")


class MonitorHardware(Protocol):
    def state(self) -> AltitudeState: ...
    def supervisor_status(self) -> dict[str, bool]: ...


class StateMonitorLogger:
    def __init__(self, root: str | Path, config: CalibrationConfig, settings: StateMonitorSettings,
                 mode: str = "state-monitor") -> None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        self.directory = Path(root) / f"{stamp}_{mode}"
        self.directory.mkdir(parents=True, exist_ok=False)
        self._stream = (self.directory / "state_telemetry.csv").open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._stream, fieldnames=MONITOR_FIELDS)
        self._writer.writeheader()
        self.metadata: dict[str, Any] = {
            "schema_version": 1, "mode": mode,
            "started_utc": datetime.now(timezone.utc).isoformat(), "status": "running",
            "monitor": asdict(settings), "config": asdict(config),
        }
        self._write_metadata()

    def _write_metadata(self) -> None:
        (self.directory / "metadata.json").write_text(
            json.dumps(self.metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    def row(self, values: dict[str, Any]) -> None:
        self._writer.writerow(values)
        self._stream.flush()

    def close(self, status: str, reason: str) -> None:
        if self._stream.closed:
            return
        self._stream.close()
        self.metadata.update(status=status, reason=reason, ended_utc=datetime.now(timezone.utc).isoformat())
        self._write_metadata()


class StateMonitorRunner:
    """Read-only state sampler. It has no arm, disarm, or command methods."""

    def __init__(self, hardware: MonitorHardware, config: CalibrationConfig, logger: StateMonitorLogger,
                 settings: StateMonitorSettings | None = None, clock=time.monotonic, sleeper=time.sleep) -> None:
        self.hardware = hardware
        self.config = config
        self.logger = logger
        self.settings = settings or StateMonitorSettings()
        self.settings.validate()
        self.clock = clock
        self.sleeper = sleeper

    def run(self) -> dict[str, Any]:
        period = 1.0 / self.settings.frequency_hz
        start = previous = deadline = self.clock()
        status, reason = "complete", "monitor duration completed"
        try:
            while True:
                now = self.clock()
                elapsed = now - start
                if elapsed >= self.settings.duration_s:
                    break
                interval = now - previous
                previous = now
                row: dict[str, Any] = dict.fromkeys(MONITOR_FIELDS, "")
                row.update(elapsed_s=elapsed, sample_interval_s=interval, valid=0, error="")
                try:
                    supervisor = self.hardware.supervisor_status()
                    row.update(**{
                        name: int(bool(supervisor[name]))
                        for name in MONITOR_FIELDS if name in supervisor
                    })
                    state = self.hardware.state()
                    age = now - state.timestamp_s
                    row.update(
                        altitude_m=state.altitude_m,
                        vertical_velocity_m_s=state.vertical_velocity_m_s,
                        roll_rad=state.roll_rad, pitch_rad=state.pitch_rad, yaw_rad=state.yaw_rad,
                        state_timestamp_s=state.timestamp_s, state_age_s=age,
                    )
                    validate_fresh_state(state, now, self.config.safety.max_state_age_s)
                    row["valid"] = 1
                except (RuntimeError, TimeoutError, KeyError, ValueError) as exc:
                    row["error"] = f"{type(exc).__name__}: {exc}"
                self.logger.row(row)
                deadline += period
                delay = deadline - self.clock()
                if delay > 0:
                    self.sleeper(delay)
                else:
                    deadline = self.clock()
        except KeyboardInterrupt:
            status, reason = "interrupted", "operator interrupted with Ctrl-C"
        except BaseException as exc:
            status, reason = "invalid", f"{type(exc).__name__}: {exc}"
            raise
        finally:
            self.logger.close(status, reason)
        return analyze_state_monitor(self.logger.directory)


def analyze_state_monitor(run_directory: str | Path) -> dict[str, Any]:
    run = Path(run_directory)
    with (run / "state_telemetry.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    valid_rows = [row for row in rows if row["valid"] == "1"]
    intervals = [float(row["sample_interval_s"]) for row in rows[1:] if float(row["sample_interval_s"]) > 0]
    ages = [float(row["state_age_s"]) for row in rows if row["state_age_s"] != ""]
    metadata = json.loads((run / "metadata.json").read_text(encoding="utf-8"))
    settings = StateMonitorSettings(**metadata["monitor"])
    stationary = _stationary_intervals(rows, settings)
    metrics = {
        "status": metadata["status"],
        "sample_count": len(rows),
        "valid_measurement_count": len(valid_rows),
        "invalid_or_stale_measurement_count": len(rows) - len(valid_rows),
        "effective_sampling_frequency_hz": (1.0 / mean(intervals)) if intervals else None,
        "maximum_state_age_s": max(ages, default=None),
        "stationary_intervals": stationary,
        "maximum_stationary_altitude_drift_m": max((item["altitude_drift_m"] for item in stationary), default=None),
        "stationary_detection": "abs(vz) and abs(roll/pitch) thresholds; heuristic, not proof that the vehicle was stationary",
    }
    (run / "metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if valid_rows:
        _write_monitor_plot(run, valid_rows)
    return metrics


def _stationary_intervals(rows: list[dict[str, str]], settings: StateMonitorSettings) -> list[dict[str, float]]:
    groups: list[list[dict[str, str]]] = []
    current: list[dict[str, str]] = []
    for row in rows:
        stationary = (
            row["valid"] == "1"
            and abs(float(row["vertical_velocity_m_s"])) <= settings.stationary_vertical_speed_m_s
            and abs(float(row["roll_rad"])) <= settings.stationary_tilt_rad
            and abs(float(row["pitch_rad"])) <= settings.stationary_tilt_rad
        )
        if stationary:
            current.append(row)
        elif current:
            groups.append(current); current = []
    if current:
        groups.append(current)
    result = []
    for group in groups:
        start, end = float(group[0]["elapsed_s"]), float(group[-1]["elapsed_s"])
        if end - start >= settings.minimum_stationary_interval_s:
            altitudes = [float(row["altitude_m"]) for row in group]
            result.append({"start_s": start, "end_s": end, "duration_s": end-start,
                           "altitude_drift_m": max(altitudes)-min(altitudes)})
    return result


def _write_monitor_plot(run: Path, rows: list[dict[str, str]]) -> None:
    panels = [
        ("altitude [m]", [("altitude_m", "#111827")]),
        ("vertical velocity [m/s]", [("vertical_velocity_m_s", "#059669")]),
        ("attitude [rad]", [("roll_rad", "#dc2626"), ("pitch_rad", "#2563eb"), ("yaw_rad", "#7c3aed")]),
        ("state age [s]", [("state_age_s", "#ea580c")]),
        ("sample interval [s]", [("sample_interval_s", "#0891b2")]),
    ]
    width, panel_height, margin = 1000, 170, 70
    height = len(panels) * panel_height + 35
    times = [float(row["elapsed_s"]) for row in rows]
    duration = max(times[-1] - times[0], 1e-9)
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">', '<rect width="100%" height="100%" fill="white"/>']
    for index, (label, series) in enumerate(panels):
        top, bottom = 15 + index*panel_height, 15 + index*panel_height + panel_height-30
        values = [float(row[key]) for key, _ in series for row in rows]
        low, high = min(values), max(values)
        if low == high: low, high = low-.5, high+.5
        svg += [f'<rect x="{margin}" y="{top}" width="{width-margin-20}" height="{bottom-top}" fill="none" stroke="#d1d5db"/>', f'<text x="5" y="{top+14}" font-family="sans-serif" font-size="11">{label}</text>']
        for key, color in series:
            points = []
            for elapsed, row in zip(times, rows):
                x = margin + (elapsed-times[0])/duration*(width-margin-20)
                y = bottom-(float(row[key])-low)/(high-low)*(bottom-top)
                points.append(f"{x:.1f},{y:.1f}")
            svg.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" stroke-width="1.4"/>')
    svg.append(f'<text x="{width/2}" y="{height-5}" text-anchor="middle" font-family="sans-serif" font-size="12">elapsed time [s]</text></svg>')
    (run / "state_response.svg").write_text("\n".join(svg) + "\n", encoding="utf-8")
