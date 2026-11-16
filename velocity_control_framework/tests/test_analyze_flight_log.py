from __future__ import annotations

import csv
import json

import pytest

from tools.analyze_flight_log import analyze_run


def test_analyze_run_uses_logged_reference_and_reports_saturation(tmp_path) -> None:
    metadata = {
        "timestamp": "2026-01-01T00:00:00+00:00",
        "status": "completed",
        "termination_reason": None,
        "controller_type": "outer-loop-pid",
        "parameter_set": "test",
        "reference": {"type": "circular"},
        "controller_configuration": {"min_thrust": 0.4, "max_thrust": 0.6},
    }
    (tmp_path / "metadata.json").write_text(json.dumps(metadata))
    fields = [
        "phase",
        *[f"estimated_position_{axis}_m" for axis in "xyz"],
        *[f"reference_position_{axis}_m" for axis in "xyz"],
        "estimated_roll_rad",
        "estimated_pitch_rad",
        "estimated_velocity_z_m_s",
        "command_thrust_normalized",
    ]
    with (tmp_path / "flight.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(
            [
                {
                    "phase": "CIRCLE",
                    "estimated_position_x_m": 1.0,
                    "estimated_position_y_m": 0.0,
                    "estimated_position_z_m": 0.2,
                    "reference_position_x_m": 0.0,
                    "reference_position_y_m": 0.0,
                    "reference_position_z_m": 0.1,
                    "estimated_roll_rad": 0.0,
                    "estimated_pitch_rad": 0.0,
                    "estimated_velocity_z_m_s": -0.2,
                    "command_thrust_normalized": 0.4,
                },
                {
                    "phase": "CIRCLE",
                    "estimated_position_x_m": 1.0,
                    "estimated_position_y_m": 1.0,
                    "estimated_position_z_m": 0.1,
                    "reference_position_x_m": 1.0,
                    "reference_position_y_m": 0.0,
                    "reference_position_z_m": 0.1,
                    "estimated_roll_rad": 0.1,
                    "estimated_pitch_rad": 0.0,
                    "estimated_velocity_z_m_s": 0.1,
                    "command_thrust_normalized": 0.5,
                },
            ]
        )

    report = analyze_run(tmp_path)

    assert report["status"] == "completed"
    assert report["phases"] == ["CIRCLE"]
    metrics = report["metrics"]["CIRCLE"]
    assert metrics["position_rmse_m"] == pytest.approx((2.01 / 2.0) ** 0.5)
    assert metrics["altitude_rmse_m"] == pytest.approx((0.01 / 2.0) ** 0.5)
    assert metrics["maximum_horizontal_displacement_m"] == pytest.approx(1.0)
    assert metrics["maximum_tilt_rad"] == pytest.approx(0.1)
    assert metrics["maximum_absolute_vertical_velocity_m_s"] == pytest.approx(0.2)
    assert metrics["commands_at_thrust_limits_percent"] == pytest.approx(50.0)
    assert metrics["commands_near_thrust_limits_percent"] == pytest.approx(50.0)
