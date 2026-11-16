"""Calculate tracking metrics from an existing FlightLogger run directory."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any, Iterable

import numpy as np


def _number(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name} in flight log")
    return value


def _phase_metrics(
    rows: list[dict[str, str]], *, min_thrust: float, max_thrust: float
) -> dict[str, Any]:
    if not rows:
        raise ValueError("cannot analyze an empty phase")
    errors = np.asarray(
        [
            [
                _number(row, f"estimated_position_{axis}_m")
                - _number(row, f"reference_position_{axis}_m")
                for axis in "xyz"
            ]
            for row in rows
        ],
        dtype=np.float64,
    )
    position_error = np.linalg.norm(errors, axis=1)
    horizontal_error = np.linalg.norm(errors[:, :2], axis=1)
    positions = np.asarray(
        [
            [_number(row, f"estimated_position_{axis}_m") for axis in "xyz"]
            for row in rows
        ],
        dtype=np.float64,
    )
    initial_xy = positions[0, :2]
    horizontal_displacement = np.linalg.norm(positions[:, :2] - initial_xy, axis=1)
    roll = np.asarray([_number(row, "estimated_roll_rad") for row in rows])
    pitch = np.asarray([_number(row, "estimated_pitch_rad") for row in rows])
    tilt = np.arccos(np.clip(np.cos(roll) * np.cos(pitch), -1.0, 1.0))
    vertical_velocity = np.asarray(
        [_number(row, "estimated_velocity_z_m_s") for row in rows]
    )
    thrust = np.asarray(
        [_number(row, "command_thrust_normalized") for row in rows]
    )
    thrust_span = max_thrust - min_thrust
    near_margin = 0.01 * thrust_span
    near_limit = (thrust <= min_thrust + near_margin) | (
        thrust >= max_thrust - near_margin
    )
    at_limit = (thrust <= min_thrust + 1.0e-9) | (
        thrust >= max_thrust - 1.0e-9
    )
    return {
        "samples": len(rows),
        "position_rmse_m": float(np.sqrt(np.mean(position_error**2))),
        "horizontal_rmse_m": float(np.sqrt(np.mean(horizontal_error**2))),
        "altitude_rmse_m": float(np.sqrt(np.mean(errors[:, 2] ** 2))),
        "maximum_position_error_m": float(np.max(position_error)),
        "maximum_horizontal_error_m": float(np.max(horizontal_error)),
        "maximum_horizontal_displacement_m": float(np.max(horizontal_displacement)),
        "maximum_tilt_rad": float(np.max(tilt)),
        "maximum_absolute_vertical_velocity_m_s": float(
            np.max(np.abs(vertical_velocity))
        ),
        "minimum_thrust_normalized": float(np.min(thrust)),
        "maximum_thrust_normalized": float(np.max(thrust)),
        "commands_near_thrust_limits_percent": float(100.0 * np.mean(near_limit)),
        "commands_at_thrust_limits_percent": float(100.0 * np.mean(at_limit)),
    }


def analyze_run(directory: Path) -> dict[str, Any]:
    """Return run identification, completion, and per-phase tracking metrics."""
    directory = directory.resolve()
    metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
    with (directory / "flight.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("flight.csv contains no controller records")
    controller = metadata.get("controller_configuration", {})
    min_thrust = float(controller["min_thrust"])
    max_thrust = float(controller["max_thrust"])
    if not 0.0 <= min_thrust < max_thrust <= 1.0:
        raise ValueError("metadata contains invalid thrust limits")
    phases = list(dict.fromkeys(row["phase"] for row in rows))
    metrics = {
        "all": _phase_metrics(rows, min_thrust=min_thrust, max_thrust=max_thrust)
    }
    for phase in phases:
        metrics[phase] = _phase_metrics(
            [row for row in rows if row["phase"] == phase],
            min_thrust=min_thrust,
            max_thrust=max_thrust,
        )
    return {
        "run_directory": str(directory),
        "timestamp": metadata.get("timestamp"),
        "status": metadata.get("status"),
        "termination_reason": metadata.get("termination_reason"),
        "controller": metadata.get("controller_type"),
        "configuration": metadata.get("parameter_set"),
        "reference": metadata.get("reference", {}).get("type"),
        "phases": phases,
        "metrics": metrics,
    }


def main(argv: Iterable[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    args = parser.parse_args(argv)
    print(json.dumps(analyze_run(args.run_directory), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
