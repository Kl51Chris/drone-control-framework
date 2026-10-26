from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from statistics import mean
from typing import Any

from .config import _construct, load_config
from .hardware import CrazyflieHardware, attitude_rates
from .state import AltitudeState


DIAGNOSTIC_FIELDS = [
    "elapsed_s", "roll_rad", "pitch_rad", "yaw_rad",
    "roll_rate_command_rad_s", "pitch_rate_command_rad_s", "yaw_rate_command_rad_s",
    "firmware_roll_rate_rad_s", "firmware_pitch_rate_rad_s", "firmware_yaw_rate_rad_s",
    "roll_saturated", "pitch_saturated", "yaw_saturated",
]


def _check(name: str, passed: bool, evidence: dict[str, Any], classification: str = "verified") -> dict[str, Any]:
    return {"name": name, "result": "PASS" if passed else "FAIL", "classification": classification, "evidence": evidence}


def _correlation(left: list[float], right: list[float]) -> float | None:
    if len(left) < 2:
        return None
    left_mean, right_mean = mean(left), mean(right)
    numerator = sum((a-left_mean)*(b-right_mean) for a, b in zip(left, right))
    denominator = math.sqrt(sum((a-left_mean)**2 for a in left) * sum((b-right_mean)**2 for b in right))
    return numerator / denominator if denominator else None


def generate_motor_free_report(run_directory: str | Path, config_path: str | Path | None = None) -> dict[str, Any]:
    """Analyze recorded states and calculate commands without opening hardware."""
    run = Path(run_directory)
    metadata = json.loads((run / "metadata.json").read_text(encoding="utf-8"))
    config = load_config(config_path) if config_path is not None else _construct(metadata["config"])
    with (run / "state_telemetry.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    valid = [row for row in rows if row["valid"] == "1"]
    if len(valid) < 2:
        raise ValueError("at least two valid state-monitor samples are required")

    elapsed = [float(row["elapsed_s"]) for row in valid]
    altitude = [float(row["altitude_m"]) for row in valid]
    vertical_velocity = [float(row["vertical_velocity_m_s"]) for row in valid]
    finite_difference_velocity, midpoint_velocity = [], []
    for index in range(1, len(valid)):
        dt = elapsed[index] - elapsed[index-1]
        if dt > 0:
            finite_difference_velocity.append((altitude[index]-altitude[index-1])/dt)
            midpoint_velocity.append((vertical_velocity[index]+vertical_velocity[index-1])/2)
    correlation = _correlation(finite_difference_velocity, midpoint_velocity)
    significant = [(dz, vz) for dz, vz in zip(finite_difference_velocity, midpoint_velocity) if abs(vz) > 0.05]
    sign_matches = sum(dz*vz > 0 for dz, vz in significant)
    sign_fraction = sign_matches / len(significant) if significant else 0.0
    positive_motion = any(dz > 0 and vz > 0.05 for dz, vz in significant)
    negative_motion = any(dz < 0 and vz < -0.05 for dz, vz in significant)

    initial_yaw = float(valid[0]["yaw_rad"])
    diagnostic_path = run / "attitude_command_diagnostic.csv"
    max_rate = config.attitude.max_rate_rad_s
    attitude_corrective = True
    saturation_counts = [0, 0, 0]
    max_commands = [0.0, 0.0, 0.0]
    with diagnostic_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=DIAGNOSTIC_FIELDS)
        writer.writeheader()
        for row in valid:
            state = AltitudeState(
                float(row["altitude_m"]), float(row["vertical_velocity_m_s"]),
                float(row["roll_rad"]), float(row["pitch_rad"]), float(row["yaw_rad"]),
                float(row["state_timestamp_s"]),
            )
            commands = attitude_rates(state, initial_yaw, config.attitude)
            # Installed cflib send_setpoint_manual packs (roll, -pitch, yaw).
            firmware_commands = (commands[0], -commands[1], commands[2])
            errors = (-state.roll_rad, -state.pitch_rad,
                      math.atan2(math.sin(initial_yaw-state.yaw_rad), math.cos(initial_yaw-state.yaw_rad)))
            for index, (command, firmware_command, error) in enumerate(zip(commands, firmware_commands, errors)):
                max_commands[index] = max(max_commands[index], abs(command))
                saturated = math.isclose(abs(command), max_rate, rel_tol=0, abs_tol=1e-12)
                saturation_counts[index] += int(saturated)
                if abs(error) > 1e-9 and firmware_command*error < -1e-12:
                    attitude_corrective = False
            writer.writerow({
                "elapsed_s": row["elapsed_s"], "roll_rad": state.roll_rad,
                "pitch_rad": state.pitch_rad, "yaw_rad": state.yaw_rad,
                "roll_rate_command_rad_s": commands[0], "pitch_rate_command_rad_s": commands[1],
                "yaw_rate_command_rad_s": commands[2],
                "firmware_roll_rate_rad_s": firmware_commands[0],
                "firmware_pitch_rate_rad_s": firmware_commands[1],
                "firmware_yaw_rate_rad_s": firmware_commands[2],
                "roll_saturated": int(math.isclose(abs(commands[0]), max_rate, abs_tol=1e-12)),
                "pitch_saturated": int(math.isclose(abs(commands[1]), max_rate, abs_tol=1e-12)),
                "yaw_saturated": int(math.isclose(abs(commands[2]), max_rate, abs_tol=1e-12)),
            })

    thrust_inputs = [0.0, 0.3, 0.38, 0.45]
    thrust_encoded = {str(value): CrazyflieHardware.encode_thrust(value) for value in thrust_inputs}
    thrust_expected = {str(value): value*100.0 for value in thrust_inputs}
    thrust_uint16 = {str(value): int(10001 + value*(60000-10001)) for value in thrust_inputs}
    safety = config.safety
    sample_intervals = [float(row["sample_interval_s"]) for row in rows[1:]]
    state_ages = [float(row["state_age_s"]) for row in rows if row["state_age_s"]]
    measured_tilts = [max(abs(float(row["roll_rad"])), abs(float(row["pitch_rad"]))) for row in valid]
    checks = [
        _check("state_monitor_complete", metadata.get("status") == "complete" and len(valid) == len(rows), {"valid": len(valid), "total": len(rows)}),
        _check("altitude_and_vertical_velocity_sign", correlation is not None and correlation >= 0.8 and sign_fraction >= 0.9 and positive_motion and negative_motion,
               {"correlation_dz_dt_to_vz": correlation, "sign_match_count": sign_matches,
                "significant_count": len(significant), "sign_match_fraction": sign_fraction,
                "positive_motion_observed": positive_motion, "negative_motion_observed": negative_motion}),
        _check("offline_attitude_commands_are_corrective", attitude_corrective,
               {"initial_yaw_rad": initial_yaw, "maximum_absolute_commands_rad_s": dict(zip(("roll", "pitch", "yaw"), max_commands)),
                "saturation_counts": dict(zip(("roll", "pitch", "yaw"), saturation_counts)), "sample_count": len(valid)}),
        _check("attitude_rate_saturation", all(value <= max_rate+1e-12 for value in max_commands), {"limit_rad_s": max_rate, "observed_rad_s": max_commands}),
        _check("normalized_thrust_encoding", thrust_encoded == thrust_expected,
               {"adapter_percent": thrust_encoded, "expected_percent": thrust_expected,
                "cflib_packed_uint16": thrust_uint16,
                "mapping": "int(10001 + normalized * (60000 - 10001)); stop-setpoint is required for true motor stop"}),
        _check("configured_thrust_limits", 0 <= safety.min_thrust <= config.equilibrium_thrust <= safety.max_thrust <= 1 and config.equilibrium_thrust+config.pid.correction_max <= safety.max_thrust and config.equilibrium_thrust+config.pid.correction_min >= safety.min_thrust,
               {"minimum": safety.min_thrust, "equilibrium": config.equilibrium_thrust, "maximum": safety.max_thrust,
                "equilibrium_plus_pid_range": [config.equilibrium_thrust+config.pid.correction_min, config.equilibrium_thrust+config.pid.correction_max]}),
        _check("recorded_state_safety_limits", max(measured_tilts) <= safety.max_tilt_rad and max(altitude) <= safety.max_altitude_m and max(abs(value) for value in vertical_velocity) <= safety.max_vertical_speed_m_s,
               {"maximum_tilt_rad": max(measured_tilts), "limit_tilt_rad": safety.max_tilt_rad,
                "altitude_range_m": [min(altitude), max(altitude)], "maximum_altitude_m": safety.max_altitude_m,
                "maximum_absolute_vertical_speed_m_s": max(abs(value) for value in vertical_velocity), "limit_vertical_speed_m_s": safety.max_vertical_speed_m_s}),
        _check("recorded_timing_safety_limits", max(state_ages) <= safety.max_state_age_s and max(sample_intervals) <= safety.max_loop_dt_s,
               {"maximum_state_age_s": max(state_ages), "limit_state_age_s": safety.max_state_age_s,
                "maximum_sample_interval_s": max(sample_intervals), "limit_loop_dt_s": safety.max_loop_dt_s}),
    ]
    assumptions = [
        {"name": "installed_firmware_matches_reviewed_generic-manual semantics", "result": "UNVERIFIED_HARDWARE", "detail": "Firmware identity and configured controller were not recorded by state-monitor; diagnostic models installed cflib pitch inversion."},
        {"name": "physical_attitude_response_polarity", "result": "UNVERIFIED_HARDWARE", "detail": "Commands were calculated offline and never transmitted."},
        {"name": "equilibrium_thrust", "result": "UNVERIFIED_HARDWARE", "detail": "0.30 is configured but cannot be validated without a separately authorized motor experiment."},
        {"name": "thrust_baseline_physical_safety", "result": "UNVERIFIED_HARDWARE", "detail": "Battery, mass, propellers, deck, test fixture, and actual lift threshold are not established by this recording."},
    ]
    report = {
        "schema_version": 1, "source_run": run.name,
        "diagnostic_config_source": str(config_path) if config_path is not None else "recorded run metadata",
        "overall_offline_result": "PASS" if all(item["result"] == "PASS" for item in checks) else "FAIL",
        "verified_checks": checks, "hardware_dependent_assumptions": assumptions,
        "artifacts": {"attitude_commands": diagnostic_path.name},
    }
    (run / "motor_free_control_report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report
