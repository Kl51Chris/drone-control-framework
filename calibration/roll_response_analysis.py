from __future__ import annotations

import csv
import json
from pathlib import Path
from statistics import mean
from typing import Any

from .pitch_response_analysis import (
    _best_lag, _bootstrap_slope, _local_acceleration, _slope,
)


PULSE_PHASES = {"ROLL_POSITIVE", "ROLL_NEGATIVE", "NEUTRAL_BEFORE",
                "RECOVERY_POSITIVE", "RECOVERY_NEGATIVE"}


def analyze_roll_response(run_directory: str | Path) -> dict[str, Any]:
    run = Path(run_directory)
    metadata = json.loads((run / "metadata.json").read_text())
    with (run / "flight_telemetry.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    active = [r for r in rows if r.get("roll_identification_phase") in PULSE_PHASES]
    report = {
        "schema_version": 1,
        "analysis": "roll-reference-to-body-y-response",
        "source_run": str(run),
        "software_status": metadata.get("status"),
        "sample_count": len(rows),
        "identification_sample_count": len(active),
        "safety_event": metadata.get("safety_event"),
    }
    counts = {phase: sum(r.get("roll_identification_phase") == phase for r in active)
              for phase in sorted(PULSE_PHASES)}
    report["phase_sample_counts"] = counts
    required = all(counts[p] >= 10 for p in (
        "NEUTRAL_BEFORE", "ROLL_POSITIVE", "ROLL_NEGATIVE",
        "RECOVERY_POSITIVE", "RECOVERY_NEGATIVE"))
    if not required:
        report.update(result="INCONCLUSIVE",
                      reason="required neutral, bipolar pulse, and recovery samples are incomplete")
        return _save(run, report)

    dt = mean(float(r["dt_s"]) for r in active)
    maximum_lag = max(1, round(.30 / dt))
    command = [float(r["roll_excitation_reference_rad"]) for r in active]
    roll = [float(r["roll_rad"]) for r in active]
    _, attitude_lag, attitude_corr = _best_lag(command, roll, maximum_lag)
    grouped = {phase: [] for phase in PULSE_PHASES}
    grouped_p = {phase: [] for phase in PULSE_PHASES}
    for i in range(len(active) - attitude_lag):
        phase = active[i]["roll_identification_phase"]
        grouped[phase].append(float(active[i + attitude_lag]["roll_rad"]))
        grouped_p[phase].append(float(active[i + attitude_lag]["p_rad_s"]))
    neutral = mean(grouped["NEUTRAL_BEFORE"])
    positive = mean(grouped["ROLL_POSITIVE"]) - neutral
    negative = mean(grouped["ROLL_NEGATIVE"]) - neutral
    command_pass = positive >= .005 and negative <= -.005

    lookup = {i: (ax, ay) for i, ax, ay in _local_acceleration(active)}
    valid = [i for i in range(len(active)) if i in lookup]
    roll_valid = [float(active[i]["roll_rad"]) for i in valid]
    ay = [lookup[i][1] for i in valid]
    _, acceleration_lag, attitude_accel_corr = _best_lag(
        roll_valid, ay, maximum_lag)
    x = roll_valid[:len(roll_valid) - acceleration_lag or None]
    y = ay[acceleration_lag:]
    physical_slope = _slope(x, y)
    interval = _bootstrap_slope(x, y)
    identified_sign = (1 if interval and interval[0] > 0 else
                       -1 if interval and interval[1] < 0 else None)
    saturation = sum(int(r.get("roll_rate_saturated", 0) or 0)
                     for r in active) / len(active)
    accepted = (command_pass and identified_sign is not None and
                metadata.get("safety_event") is None and saturation <= .05)
    report.update({
        "command_to_attitude": {
            "best_lag_s": attitude_lag * dt,
            "correlation": attitude_corr,
            "neutral_mean_roll_rad": neutral,
            "positive_pulse_delta_roll_rad": positive,
            "negative_pulse_delta_roll_rad": negative,
            "mean_p_rad_s": {
                "positive": mean(grouped_p["ROLL_POSITIVE"]),
                "negative": mean(grouped_p["ROLL_NEGATIVE"]),
            },
            "polarity_pass": command_pass,
        },
        "attitude_to_body_y_acceleration": {
            "method": "centered 0.30 s local velocity regression, yaw rotation, lag scan, 5-sample moving-block bootstrap",
            "best_lag_s": acceleration_lag * dt,
            "correlation": attitude_accel_corr,
            "slope_m_s2_per_rad": physical_slope,
            "slope_95_percent_block_bootstrap": interval,
            "identified_roll_mapping_sign": identified_sign,
        },
        "roll_rate_saturation_fraction": saturation,
        "result": "PASS" if accepted else "INCONCLUSIVE",
        "reason": ("both command-to-attitude polarity and physical attitude-to-acceleration sign identified without a safety event"
                   if accepted else
                   "polarity response, physical acceleration confidence, saturation, or safety criteria were not all satisfied"),
        "automatic_configuration_change": False,
    })
    return _save(run, report)


def _save(run: Path, report: dict[str, Any]) -> dict[str, Any]:
    (run / "roll_response_analysis.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")
    return report
