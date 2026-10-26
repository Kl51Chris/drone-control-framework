from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any


AXES = ("roll", "pitch", "yaw")


def _flight_phase(raw: str) -> str:
    return "LANDING" if raw in {"LANDING", "LANDING_CONFIRM"} else raw


def _runs(rows: list[dict[str, str]], axis: str) -> list[dict[str, Any]]:
    flag = f"{axis}_rate_saturated"
    runs: list[dict[str, Any]] = []
    start: int | None = None
    for index in range(len(rows)+1):
        active = index < len(rows) and int(rows[index][flag]) == 1
        if active and start is None:
            start = index
        if not active and start is not None:
            selected = rows[start:index]
            first, last = selected[0], selected[-1]
            start_s, end_s = float(first["elapsed_s"]), float(last["elapsed_s"])
            runs.append({
                "start_elapsed_s": start_s,
                "last_sample_elapsed_s": end_s,
                "estimated_command_end_s": end_s+float(last["dt_s"]),
                "estimated_duration_s": end_s-start_s+float(last["dt_s"]),
                "sample_count": len(selected),
                "phases": sorted({_flight_phase(row["phase"]) for row in selected}),
                "raw_phases": sorted({row["phase"] for row in selected}),
                "command_sign": "positive" if float(first[f"command_{axis}_rate_rad_s"]) > 0 else "negative",
                "maximum_absolute_error_rad": max(abs(float(row[f"{axis}_error_rad"])) for row in selected),
            })
            start = None
    return runs


def analyze_rate_saturation(replay_csv: str | Path, destination: str | Path) -> dict[str, Any]:
    source = Path(replay_csv)
    with source.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("cascaded replay has no rows")
    phases = ("TAKEOFF", "HOVER", "LANDING")
    breakdown = {
        axis: {
            phase: sum(int(row[f"{axis}_rate_saturated"]) for row in rows
                       if _flight_phase(row["phase"]) == phase)
            for phase in phases}
        for axis in AXES}
    runs = {axis: _runs(rows, axis) for axis in AXES}

    restoring: dict[str, Any] = {}
    firmware_sign = {"roll": 1.0, "pitch": -1.0, "yaw": 1.0}
    for axis in AXES:
        relevant = [row for row in rows if abs(float(row[f"{axis}_error_rad"])) > 1e-12]
        correct = sum(
            firmware_sign[axis]*float(row[f"command_{axis}_rate_rad_s"])*
            float(row[f"{axis}_error_rad"]) > 0 for row in relevant)
        restoring[axis] = {
            "nonzero_error_samples": len(relevant), "restoring_samples": correct,
            "all_restoring": correct == len(relevant),
            "firmware_rate_from_api": f"{firmware_sign[axis]:+g} * API {axis} rate",
        }

    yaw_reference = float(rows[0]["desired_yaw_rad"])
    yaw_errors = [float(row["yaw_error_rad"]) for row in rows]
    measured_yaw = [float(row["measured_yaw_rad"]) for row in rows]
    wrap_jumps = sum(abs(measured_yaw[i]-measured_yaw[i-1]) > math.pi for i in range(1, len(rows)))
    yaw_limit = max(abs(float(row["command_yaw_rate_rad_s"])) for row in rows)
    yaw_gain_values = [
        abs(float(row["yaw_rate_raw_rad_s"])/float(row["yaw_error_rad"]))
        for row in rows if abs(float(row["yaw_error_rad"])) > 1e-12]
    report = {
        "schema_version": 1,
        "analysis": "stage5b-rate-saturation-and-interface-review",
        "source_replay": str(source), "sample_count": len(rows),
        "phase_grouping": "LANDING includes raw LANDING and LANDING_CONFIRM rows",
        "saturation_sample_counts": breakdown,
        "consecutive_saturation_runs": runs,
        "yaw_analysis": {
            "captured_reference_yaw_rad": yaw_reference,
            "capture_elapsed_s": float(rows[0]["elapsed_s"]),
            "gain_rad_s_per_rad": max(yaw_gain_values) if yaw_gain_values else 0.0,
            "rate_limit_rad_s": yaw_limit,
            "implied_saturation_error_threshold_rad": yaw_limit/(max(yaw_gain_values) if yaw_gain_values else 1),
            "maximum_absolute_wrapped_error_rad": max(abs(value) for value in yaw_errors),
            "measured_yaw_wrap_boundary_crossings": wrap_jumps,
            "wrapped_error_within_principal_interval": all(-math.pi <= value <= math.pi for value in yaw_errors),
        },
        "restoring_direction_check": restoring,
        "interface": {
            "controller_output_order": ["roll_rate", "pitch_rate", "yaw_rate", "thrust"],
            "controller_units": ["rad/s", "rad/s", "rad/s", "normalized [0,1]"],
            "adapter_call_order": ["roll", "pitch", "yawrate", "thrust_percentage", "rate=True"],
            "adapter_units": ["deg/s", "deg/s", "deg/s", "percent [0,100]", "bool"],
            "installed_cflib_packet_order": ["TYPE_MANUAL", "roll", "-pitch", "yawrate", "thrust_uint16", "rate"],
            "pitch_inversion_locations": [
                "CascadedPIDController pitch_command_sign=-1 maps legacy measured pitch error to API command",
                "installed cflib send_setpoint_manual packs -pitch into the firmware packet",
            ],
            "duplicated_in_adapter": False,
            "ownership_warning": (
                "The adapter itself performs only rad/s-to-deg/s conversion, but its documentation says "
                "backend sign conventions belong there. The controller currently compensates for cflib's "
                "downstream pitch inversion, so sign ownership is split across layers and must be resolved "
                "before integration, without changing the verified net firmware-side polarity."
            ),
        },
    }
    output = Path(destination)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True)+"\n", encoding="utf-8")
    return report
