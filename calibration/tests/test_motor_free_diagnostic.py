import csv
import json

from calibration.config import CalibrationConfig
from calibration.motor_free_diagnostic import generate_motor_free_report
from calibration.state_monitor import MONITOR_FIELDS, StateMonitorLogger, StateMonitorSettings


def test_motor_free_report_and_offline_command_artifact(tmp_path):
    config = CalibrationConfig(output_directory=str(tmp_path))
    settings = StateMonitorSettings()
    logger = StateMonitorLogger(tmp_path, config, settings)
    altitudes = [0.0, 0.002, 0.004, 0.006, 0.004, 0.002, 0.0]
    velocities = [0.1, 0.1, 0.1, 0.1, -0.1, -0.1, -0.1]
    for index, (altitude, velocity) in enumerate(zip(altitudes, velocities)):
        row = dict.fromkeys(MONITOR_FIELDS, 0)
        row.update(
            elapsed_s=index*0.02, sample_interval_s=0.02,
            altitude_m=altitude, vertical_velocity_m_s=velocity,
            roll_rad=(-1)**index*0.02, pitch_rad=(-1)**(index+1)*0.03,
            yaw_rad=0.01*index, state_timestamp_s=index*0.02,
            state_age_s=0.001, valid=1, error="",
        )
        logger.row(row)
    logger.close("complete", "test fixture")

    report = generate_motor_free_report(logger.directory)
    assert report["overall_offline_result"] == "PASS"
    checks = {item["name"]: item for item in report["verified_checks"]}
    assert checks["normalized_thrust_encoding"]["evidence"]["adapter_percent"] == {
        "0.0": 0.0, "0.3": 30.0, "0.38": 38.0, "0.45": 45.0,
    }
    assert checks["normalized_thrust_encoding"]["evidence"]["cflib_packed_uint16"] == {
        "0.0": 10001, "0.3": 25000, "0.38": 29000, "0.45": 32500,
    }
    assert all(item["result"] == "UNVERIFIED_HARDWARE" for item in report["hardware_dependent_assumptions"])
    assert (logger.directory / "attitude_command_diagnostic.csv").exists()
    assert json.loads((logger.directory / "motor_free_control_report.json").read_text())["overall_offline_result"] == "PASS"

    with (logger.directory / "attitude_command_diagnostic.csv").open(newline="") as stream:
        commands = list(csv.DictReader(stream))
    assert len(commands) == len(altitudes)
    assert float(commands[1]["roll_rate_command_rad_s"]) > 0
    assert float(commands[1]["pitch_rate_command_rad_s"]) > 0
    assert float(commands[1]["firmware_pitch_rate_rad_s"]) < 0
