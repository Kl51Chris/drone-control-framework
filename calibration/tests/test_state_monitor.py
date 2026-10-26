import csv
import json

import pytest

from calibration.config import CalibrationConfig
from calibration.state import AltitudeState
from calibration.state_monitor import (
    StateMonitorLogger,
    StateMonitorRunner,
    StateMonitorSettings,
    analyze_state_monitor,
)


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class MonitorHardware:
    """Intentionally exposes no arm or command method."""

    def __init__(self, clock, stale_sample=None):
        self.clock = clock
        self.samples = 0
        self.stale_sample = stale_sample

    def state(self):
        self.samples += 1
        timestamp = self.clock()
        if self.samples == self.stale_sample:
            timestamp -= 1.0
        return AltitudeState(
            altitude_m=0.4 + self.samples * 0.001,
            vertical_velocity_m_s=0.01,
            roll_rad=0.01,
            pitch_rad=-0.01,
            yaw_rad=0.2,
            timestamp_s=timestamp,
        )

    def supervisor_status(self):
        return {
            "can_be_armed": True, "can_fly": True, "is_armed": False,
            "is_auto_armed": False, "is_flying": False, "is_tumbled": False,
            "is_locked": False, "is_crashed": False, "hl_control_active": False,
        }


def make_runner(tmp_path, *, stale_sample=None, sleeper=None):
    clock = FakeClock()
    config = CalibrationConfig(output_directory=str(tmp_path))
    settings = StateMonitorSettings(
        frequency_hz=10, duration_s=1,
        minimum_stationary_interval_s=0.5,
    )
    logger = StateMonitorLogger(tmp_path, config, settings)
    hardware = MonitorHardware(clock, stale_sample=stale_sample)
    runner = StateMonitorRunner(
        hardware, config, logger, settings,
        clock=clock, sleeper=sleeper or clock.sleep,
    )
    return runner, hardware, logger


def test_monitor_samples_without_arm_or_control_interface_and_generates_artifacts(tmp_path):
    runner, hardware, logger = make_runner(tmp_path)
    metrics = runner.run()
    assert 9 <= hardware.samples <= 11
    assert metrics["effective_sampling_frequency_hz"] == pytest.approx(10)
    assert metrics["invalid_or_stale_measurement_count"] == 0
    assert metrics["maximum_state_age_s"] == 0
    assert metrics["maximum_stationary_altitude_drift_m"] is not None
    assert (logger.directory / "state_telemetry.csv").exists()
    assert (logger.directory / "metrics.json").exists()
    assert (logger.directory / "state_response.svg").exists()


def test_monitor_counts_stale_measurements_and_logs_supervisor_status(tmp_path):
    runner, _, logger = make_runner(tmp_path, stale_sample=3)
    metrics = runner.run()
    assert metrics["invalid_or_stale_measurement_count"] == 1
    assert metrics["maximum_state_age_s"] == pytest.approx(1.0)
    with (logger.directory / "state_telemetry.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]["can_be_armed"] == "1"
    assert rows[0]["is_armed"] == "0"
    assert rows[2]["valid"] == "0"
    assert "stale" in rows[2]["error"]


def test_ctrl_c_saves_and_analyzes_partial_run(tmp_path):
    def interrupt(_duration):
        raise KeyboardInterrupt

    runner, _, logger = make_runner(tmp_path, sleeper=interrupt)
    metrics = runner.run()
    metadata = json.loads((logger.directory / "metadata.json").read_text())
    assert metadata["status"] == "interrupted"
    assert metrics["status"] == "interrupted"
    assert metrics["sample_count"] == 1
    assert (logger.directory / "state_response.svg").exists()


def test_monitor_settings_validation():
    with pytest.raises(ValueError):
        StateMonitorSettings(frequency_hz=0).validate()
