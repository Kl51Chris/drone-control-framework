from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from statistics import mean
from typing import Any


def analyze(run_directory: str | Path) -> dict[str, Any]:
    run = Path(run_directory)
    metadata = json.loads((run / "metadata.json").read_text(encoding="utf-8"))
    with (run / "telemetry.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    numeric = ["elapsed_s", "dt_s", "altitude_reference_m", "altitude_m", "vertical_velocity_m_s", "p_term", "i_term", "d_term", "thrust_normalized", "saturated"]
    values = {name: [float(row[name]) for row in rows] for name in numeric}
    errors = [reference - measured for reference, measured in zip(values["altitude_reference_m"], values["altitude_m"])]
    phases = []
    for row in rows:
        if not phases or phases[-1]["phase"] != row["phase"]:
            phases.append({"phase": row["phase"], "elapsed_s": float(row["elapsed_s"])})
    valid = metadata.get("status") == "complete" and len(rows) >= 2 and "HOVER" in {r["phase"] for r in rows}
    reasons = []
    if metadata.get("status") != "complete": reasons.append(f"run status is {metadata.get('status')}")
    if len(rows) < 2: reasons.append("insufficient telemetry")
    metrics = {
        "valid": valid,
        "invalid_reasons": reasons,
        "sample_count": len(rows),
        "duration_s": values["elapsed_s"][-1] if rows else 0.0,
        "rmse_m": math.sqrt(mean(e * e for e in errors)) if errors else None,
        "mean_absolute_error_m": mean(abs(e) for e in errors) if errors else None,
        "maximum_absolute_error_m": max((abs(e) for e in errors), default=None),
        "mean_loop_dt_s": mean(values["dt_s"]) if rows else None,
        "maximum_loop_dt_s": max(values["dt_s"], default=None),
        "saturation_fraction": mean(values["saturated"]) if rows else None,
        "phase_transitions": phases,
    }
    metrics_path = run / "metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if rows:
        _plot(run, rows)
    return metrics


def _plot(run: Path, rows: list[dict[str, str]]) -> None:
    t = [float(row["elapsed_s"]) for row in rows]
    panels = [
        ("altitude [m]", [("reference", "altitude_reference_m", "#2563eb"), ("measured", "altitude_m", "#111827")]),
        ("vertical velocity [m/s]", [("vz", "vertical_velocity_m_s", "#059669")]),
        ("PID contributions", [("P", "p_term", "#dc2626"), ("I", "i_term", "#7c3aed"), ("D", "d_term", "#ea580c")]),
        ("normalized thrust", [("thrust", "thrust_normalized", "#111827")]),
        ("control-loop dt [s]", [("dt", "dt_s", "#0891b2")]),
    ]
    width, panel_height, margin = 1000, 180, 60
    height = panel_height * len(panels) + 40
    duration = max(t[-1] - t[0], 1e-9)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">', '<rect width="100%" height="100%" fill="white"/>']
    transitions: list[tuple[float, str]] = []
    previous = None
    for row in rows:
        if row["phase"] != previous:
            transitions.append((float(row["elapsed_s"]), row["phase"])); previous = row["phase"]
    for index, (label, series) in enumerate(panels):
        top = 20 + index * panel_height; bottom = top + panel_height - 35
        all_values = [float(row[key]) for _, key, _ in series for row in rows]
        low, high = min(all_values), max(all_values)
        if low == high: low, high = low - .5, high + .5
        parts += [f'<rect x="{margin}" y="{top}" width="{width-margin-20}" height="{bottom-top}" fill="none" stroke="#d1d5db"/>', f'<text x="8" y="{top+15}" font-family="sans-serif" font-size="12">{label}</text>']
        for when, phase in transitions:
            x = margin + (when - t[0]) / duration * (width-margin-20)
            parts.append(f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{bottom}" stroke="#9ca3af" stroke-dasharray="3 3"/>')
            if index == 0: parts.append(f'<text x="{x+3:.1f}" y="{top+14}" font-family="sans-serif" font-size="10">{phase}</text>')
        for series_index, (name, key, color) in enumerate(series):
            points = []
            for when, row in zip(t, rows):
                x = margin + (when - t[0]) / duration * (width-margin-20)
                y = bottom - (float(row[key]) - low) / (high-low) * (bottom-top)
                points.append(f"{x:.1f},{y:.1f}")
            parts.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" stroke-width="1.5"/><text x="{width-100}" y="{top+15+14*series_index}" fill="{color}" font-family="sans-serif" font-size="11">{name}</text>')
        if index == 3:
            for when, row in zip(t, rows):
                if int(row["saturated"]):
                    x = margin + (when-t[0])/duration*(width-margin-20); y = bottom-(float(row["thrust_normalized"])-low)/(high-low)*(bottom-top)
                    parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="2.5" fill="#dc2626"/>')
    parts.append(f'<text x="{width/2}" y="{height-8}" text-anchor="middle" font-family="sans-serif" font-size="12">elapsed time [s]</text></svg>')
    (run / "response.svg").write_text("\n".join(parts) + "\n", encoding="utf-8")


def export_validated(run_directory: str | Path, destination: str | Path) -> dict[str, Any]:
    run = Path(run_directory)
    metrics_path = run / "metrics.json"
    metrics = json.loads(metrics_path.read_text(encoding="utf-8")) if metrics_path.exists() else analyze(run)
    if not metrics["valid"]:
        raise ValueError("only a complete, analyzed experiment can be exported")
    metadata = json.loads((run / "metadata.json").read_text(encoding="utf-8"))
    config = metadata["config"]
    exported = {
        "schema_version": 1,
        "source_run": run.name,
        "hardware_validated": False,
        "review_required": "A human must review telemetry, plots, safety events, and physical behavior before declaring hardware validation.",
        "altitude_pid": config["pid"],
        "equilibrium_thrust": config["equilibrium_thrust"],
        "control_frequency_hz": config["control_frequency_hz"],
        "thrust_limits": {"min": config["safety"]["min_thrust"], "max": config["safety"]["max_thrust"]},
        "assumptions": {"z_axis": "world positive-up metres", "thrust": "normalized collective [0,1]", "command": "body rates rad/s; Crazyflie firmware closes rate loops", "altitude_source": "see telemetry state_source"},
        "performance_metrics": metrics,
        "outer_loop_mapping": {"PIDConfig.kp": config["pid"]["kp"], "PIDConfig.ki": config["pid"]["ki"], "PIDConfig.kd": config["pid"]["kd"], "PIDConfig.output_min": config["pid"]["correction_min"], "PIDConfig.output_max": config["pid"]["correction_max"], "OuterLoopPIDConfig.equilibrium_thrust": config["equilibrium_thrust"], "OuterLoopPIDConfig.min_thrust": config["safety"]["min_thrust"], "OuterLoopPIDConfig.max_thrust": config["safety"]["max_thrust"]},
    }
    path = Path(destination); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(exported, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return exported
