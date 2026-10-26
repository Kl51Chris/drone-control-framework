from __future__ import annotations

import csv
from dataclasses import asdict, dataclass, replace
import json
import math
from pathlib import Path
from typing import Any

from .config import CalibrationConfig, PIDSettings
from .pid import AltitudePID


@dataclass(frozen=True, slots=True)
class ReplaySettings:
    equilibrium_thrust: float
    pid: PIDSettings
    min_thrust: float
    max_thrust: float

    def validate(self) -> None:
        values = (self.equilibrium_thrust, self.min_thrust, self.max_thrust,
                  *asdict(self.pid).values())
        if not all(math.isfinite(value) for value in values):
            raise ValueError("replay settings must be finite")
        if not 0 <= self.min_thrust <= self.equilibrium_thrust <= self.max_thrust <= 1:
            raise ValueError("replay thrust must satisfy 0 <= min <= equilibrium <= max <= 1")
        if self.pid.correction_min > self.pid.correction_max:
            raise ValueError("correction_min must not exceed correction_max")


REPLAY_FIELDS = [
    "elapsed_s", "phase", "dt_s", "reference_altitude_m", "measured_altitude_m",
    "measured_vertical_velocity_m_s", "error_m", "p_term", "i_term", "d_term",
    "pid_unsaturated", "pid_output", "pre_thrust_clamp", "predicted_thrust_normalized",
    "pid_saturated", "thrust_saturated", "saturated",
]


def replay_official_hover(run_directory: str | Path, output_directory: str | Path,
                          settings: ReplaySettings) -> dict[str, Any]:
    """Replay recorded measurements through a PID; this is not a plant simulation."""
    settings.validate()
    source = Path(run_directory)
    metadata = json.loads((source / "metadata.json").read_text(encoding="utf-8"))
    if metadata.get("mode") != "official-hover-baseline":
        raise ValueError("replay source must be an official-hover-baseline run")
    with (source / "flight_telemetry.csv").open(newline="", encoding="utf-8") as stream:
        input_rows = list(csv.DictReader(stream))
    if not input_rows:
        raise ValueError("replay source has no telemetry")

    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    pid = AltitudePID(settings.pid)
    output_rows: list[dict[str, Any]] = []
    previous_elapsed: float | None = None
    for row in input_rows:
        elapsed = float(row["elapsed_s"])
        recorded_dt = float(row["dt_s"])
        dt = recorded_dt if previous_elapsed is None else elapsed - previous_elapsed
        previous_elapsed = elapsed
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError(f"non-positive replay interval at elapsed {elapsed:g} s")
        reference = float(row["reference_altitude_m"])
        altitude = float(row["z_m"])
        result = pid.update(reference, altitude, dt)
        pre_clamp = settings.equilibrium_thrust + result.output
        thrust = min(max(pre_clamp, settings.min_thrust), settings.max_thrust)
        thrust_saturated = pre_clamp != thrust
        output_rows.append({
            "elapsed_s": elapsed, "phase": row["phase"], "dt_s": dt,
            "reference_altitude_m": reference, "measured_altitude_m": altitude,
            "measured_vertical_velocity_m_s": float(row["vz_m_s"]),
            "error_m": result.error, "p_term": result.proportional,
            "i_term": result.integral, "d_term": result.derivative,
            "pid_unsaturated": result.unsaturated, "pid_output": result.output,
            "pre_thrust_clamp": pre_clamp, "predicted_thrust_normalized": thrust,
            "pid_saturated": int(result.saturated),
            "thrust_saturated": int(thrust_saturated),
            "saturated": int(result.saturated or thrust_saturated),
        })

    with (destination / "pid_replay.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=REPLAY_FIELDS)
        writer.writeheader()
        writer.writerows(output_rows)
    thrusts = [float(row["predicted_thrust_normalized"]) for row in output_rows]
    errors = [float(row["error_m"]) for row in output_rows]
    report = {
        "schema_version": 1,
        "analysis": "open-loop-controller-replay",
        "source_run": str(source),
        "sample_count": len(output_rows),
        "settings": {"equilibrium_thrust": settings.equilibrium_thrust,
                     "pid": asdict(settings.pid), "min_thrust": settings.min_thrust,
                     "max_thrust": settings.max_thrust},
        "predicted_thrust": {
            "minimum": min(thrusts), "maximum": max(thrusts),
            "mean": sum(thrusts) / len(thrusts),
        },
        "error": {
            "minimum_m": min(errors), "maximum_m": max(errors),
            "rmse_m": math.sqrt(sum(error * error for error in errors) / len(errors)),
        },
        "pid_saturation_count": sum(int(row["pid_saturated"]) for row in output_rows),
        "thrust_saturation_count": sum(int(row["thrust_saturated"]) for row in output_rows),
        "any_saturation_count": sum(int(row["saturated"]) for row in output_rows),
        "interpretation": (
            "Recorded altitude is exogenous: predicted commands were not applied to a vehicle or "
            "plant model. This controller replay checks arithmetic and command ranges only; it "
            "does not predict or validate closed-loop stability or flight performance."
        ),
    }
    (destination / "pid_replay_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def settings_from_config(config: CalibrationConfig, *, equilibrium_thrust: float | None = None,
                         kp: float | None = None, ki: float | None = None,
                         kd: float | None = None, correction_min: float | None = None,
                         correction_max: float | None = None, min_thrust: float | None = None,
                         max_thrust: float | None = None) -> ReplaySettings:
    pid = replace(config.pid,
                  kp=config.pid.kp if kp is None else kp,
                  ki=config.pid.ki if ki is None else ki,
                  kd=config.pid.kd if kd is None else kd,
                  correction_min=config.pid.correction_min if correction_min is None else correction_min,
                  correction_max=config.pid.correction_max if correction_max is None else correction_max)
    return ReplaySettings(
        config.equilibrium_thrust if equilibrium_thrust is None else equilibrium_thrust,
        pid,
        config.safety.min_thrust if min_thrust is None else min_thrust,
        config.safety.max_thrust if max_thrust is None else max_thrust,
    )
