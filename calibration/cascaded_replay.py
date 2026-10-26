from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any

from .cascaded_pid import CascadedPIDConfig, CascadedPIDController, CascadedReference
from .state import AltitudeState


def replay_cascaded_official_hover(run_directory: str | Path, output_directory: str | Path,
                                    config: CascadedPIDConfig) -> dict[str, Any]:
    """Exercise the cascaded controller with recorded exogenous state samples."""
    config.validate()
    source, destination = Path(run_directory), Path(output_directory)
    metadata = json.loads((source / "metadata.json").read_text(encoding="utf-8"))
    if metadata.get("mode") != "official-hover-baseline":
        raise ValueError("replay source must be an official-hover-baseline run")
    with (source / "flight_telemetry.csv").open(newline="", encoding="utf-8") as stream:
        source_rows = list(csv.DictReader(stream))
    if not source_rows:
        raise ValueError("replay source has no telemetry")
    destination.mkdir(parents=True, exist_ok=True)
    controller = CascadedPIDController(config)
    first = source_rows[0]
    hold_x, hold_y, hold_yaw = float(first["x_m"]), float(first["y_m"]), float(first["yaw_rad"])
    output_rows: list[dict[str, Any]] = []
    previous_elapsed: float | None = None
    for row in source_rows:
        elapsed = float(row["elapsed_s"])
        dt = float(row["dt_s"]) if previous_elapsed is None else elapsed-previous_elapsed
        previous_elapsed = elapsed
        state = AltitudeState(
            float(row["z_m"]), float(row["vz_m_s"]), float(row["roll_rad"]),
            float(row["pitch_rad"]), float(row["yaw_rad"]), float(row["state_timestamp_s"]),
            "official_hover_replay", float(row["x_m"]), float(row["y_m"]),
            float(row["vx_m_s"]), float(row["vy_m_s"]),
        )
        reference = CascadedReference(
            altitude_m=float(row["reference_altitude_m"]), yaw_rad=hold_yaw,
            position_x_m=hold_x, position_y_m=hold_y,
        )
        result = controller.update(state, reference, dt)
        command = result.command
        output_rows.append({
            "elapsed_s": elapsed, "phase": row["phase"], "dt_s": dt,
            "reference_altitude_m": reference.altitude_m,
            "measured_altitude_m": state.altitude_m,
            "measured_vertical_velocity_m_s": state.vertical_velocity_m_s,
            "measured_roll_rad": state.roll_rad, "measured_pitch_rad": state.pitch_rad,
            "measured_yaw_rad": state.yaw_rad,
            **result.contributions,
            "command_roll_rate_rad_s": command.roll_rate,
            "command_pitch_rate_rad_s": command.pitch_rate,
            "command_yaw_rate_rad_s": command.yaw_rate,
            "command_thrust_normalized": command.thrust,
        })
    fields = list(output_rows[0])
    with (destination / "cascaded_replay.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(output_rows)
    thrust = [float(row["command_thrust_normalized"]) for row in output_rows]
    rate_keys = ["command_roll_rate_rad_s", "command_pitch_rate_rad_s", "command_yaw_rate_rad_s"]
    report = {
        "schema_version": 1, "analysis": "cascaded-controller-open-loop-replay",
        "controller_stage": config.stage, "source_run": str(source),
        "sample_count": len(output_rows),
        "predicted_thrust": {"minimum": min(thrust), "maximum": max(thrust),
                             "mean": sum(thrust)/len(thrust)},
        "maximum_absolute_body_rate_rad_s": {
            key.removeprefix("command_").removesuffix("_rad_s"):
                max(abs(float(row[key])) for row in output_rows) for key in rate_keys},
        "tracking_error": {
            name: {
                "maximum_absolute": max(abs(float(row[key])) for row in output_rows),
                "rmse": math.sqrt(sum(float(row[key])**2 for row in output_rows)/len(output_rows)),
            } for name, key in (
                ("altitude_m", "altitude_error_m"), ("roll_rad", "roll_error_rad"),
                ("pitch_rad", "pitch_error_rad"), ("yaw_rad", "yaw_error_rad"))},
        "saturation_counts": {
            name: sum(int(row[name]) for row in output_rows) for name in
            ("altitude_saturated", "roll_rate_saturated", "pitch_rate_saturated",
             "yaw_rate_saturated")},
        "interpretation": (
            "Recorded states came from the official firmware hover controller and are exogenous. "
            "This validates controller equations, signs, limits, logging, and deterministic replay; "
            "it is not a closed-loop simulation or evidence of custom-controller flight stability."
        ),
    }
    (destination / "cascaded_replay_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True)+"\n", encoding="utf-8")
    return report
