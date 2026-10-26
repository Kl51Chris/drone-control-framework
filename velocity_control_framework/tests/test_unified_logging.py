from __future__ import annotations

import csv
import json
from datetime import datetime, timezone

import numpy as np
import pytest

from velocity_control_framework.interfaces import (
    BodyRateThrustCommand,
    DroneState,
    Reference,
)
from velocity_control_framework.backends import CrazyflieStateProvider
from velocity_control_framework.logger import (
    EVENT_FIELDS,
    FLIGHT_FIELDS,
    FlightLogger,
    RunConfiguration,
)
from velocity_control_framework.runner import UniversalRunner, UniversalRunnerConfig


def configuration() -> RunConfiguration:
    return RunConfiguration(
        experiment_name="fixed-hover",
        reference_type="hover",
        reference_configuration={"height_m": 0.3},
        controller_type="outer-pid",
        parameter_set="tune-v3",
        controller_configuration={"kp": 0.4, "nested": {"kd": 0.8}},
        target_control_frequency_hz=50.0,
        hardware=None,
        state_estimation_source="test passthrough",
    )


def state(timestamp: float = 10.0) -> DroneState:
    return DroneState(
        position=np.array([1.0, 2.0, 3.0]),
        velocity=np.array([0.1, 0.2, 0.3]),
        roll=0.01,
        pitch=0.02,
        yaw=0.03,
        timestamp=timestamp,
        p=0.04,
        q=0.05,
        r=0.06,
    )


def reference() -> Reference:
    return Reference.hover(1.0, 2.0, 3.1, yaw=0.1, timestamp=10.0)


def command() -> BodyRateThrustCommand:
    return BodyRateThrustCommand(0.1, -0.1, 0.0, 0.42)


def rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def test_run_directory_name_collision_and_unique_ids(tmp_path) -> None:
    now = datetime(2026, 10, 26, 14, 45, 30, tzinfo=timezone.utc)
    first = FlightLogger(configuration(), tmp_path, now=now)
    second = FlightLogger(configuration(), tmp_path, now=now)
    first.close()
    second.close()

    assert first.directory.name == "20261026_144530_hover_outer-pid_tune-v3_fixed-hover"
    assert second.directory.name.startswith(first.directory.name + "_")
    assert first.run_id != second.run_id
    assert first.directory != second.directory


def test_metadata_serialization_and_terminal_status(tmp_path) -> None:
    logger = FlightLogger(configuration(), tmp_path)
    logger.mark_running(12.0)
    logger.close("failed", "test failure")

    metadata = json.loads((logger.directory / "metadata.json").read_text())
    assert metadata["run_id"] == logger.run_id
    assert datetime.fromisoformat(metadata["timestamp"]).tzinfo is not None
    assert metadata["parameter_set"] == "tune-v3"
    assert metadata["controller_configuration"]["nested"]["kd"] == 0.8
    assert metadata["target_control_frequency_hz"] == 50.0
    assert metadata["status"] == "failed"
    assert metadata["termination_reason"] == "test failure"
    assert metadata["ended_at"] is not None


def test_flight_schema_sources_missing_values_and_events(tmp_path) -> None:
    logger = FlightLogger(configuration(), tmp_path)
    logger.mark_running(10.0)
    onboard_sequence = logger.record_onboard_state(
        state(), source_timestamp_s=1.25, reception_monotonic_s=10.01
    )
    logger.log_flight(
        0.02, 10.02, "HOVER", state(), reference(), command(), 0.02,
        diagnostics={"altitude_error_m": 0.1},
        source_context={
            "onboard_sequence": onboard_sequence,
            "external_sequence": None,
        },
    )
    logger.event("safety_check", {"status": "nominal"}, monotonic_s=10.02)
    logger.close()

    flight_path = logger.directory / "flight.csv"
    with flight_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        assert tuple(reader.fieldnames or ()) == FLIGHT_FIELDS
        flight = next(reader)
    assert flight["estimated_position_x_m"] == "1.0"
    assert flight["onboard_available"] == "True"
    assert flight["onboard_sequence"] == "0"
    assert flight["external_available"] == "False"
    assert flight["external_sequence"] == ""
    assert json.loads(flight["controller_diagnostics_json"])["altitude_error_m"] == 0.1

    onboard = rows(logger.directory / "onboard_state.csv")
    assert onboard[0]["source_timestamp_s"] == "1.25"
    assert onboard[0]["reception_monotonic_s"] == "10.01"
    assert rows(logger.directory / "external_state.csv") == []
    with (logger.directory / "events.csv").open(newline="", encoding="utf-8") as stream:
        event_reader = csv.DictReader(stream)
        assert tuple(event_reader.fieldnames or ()) == EVENT_FIELDS
        events = list(event_reader)
    assert [event["event"] for event in events] == [
        "logger_initialized", "run_started", "safety_check", "run_finished"
    ]


def test_onboard_callback_preserves_firmware_and_reception_timestamps() -> None:
    captured = []

    def capture(value, **timestamps):
        captured.append((value, timestamps))
        return 17

    provider = CrazyflieStateProvider(object(), onboard_state_callback=capture)
    provider._on_translation_log_data(
        100,
        {
            "stateEstimate.x": 1.0, "stateEstimate.y": 2.0,
            "stateEstimate.z": 3.0, "stateEstimate.vx": 0.1,
            "stateEstimate.vy": 0.2, "stateEstimate.vz": 0.3,
        },
        None,
    )
    provider._on_attitude_log_data(
        200,
        {
            "stateEstimate.roll": 1.0, "stateEstimate.pitch": 2.0,
            "stateEstimate.yaw": 3.0,
        },
        None,
    )
    provider._on_body_rate_log_data(
        300,
        {
            "stateEstimateZ.rateRoll": 4.0,
            "stateEstimateZ.ratePitch": 5.0,
            "stateEstimateZ.rateYaw": 6.0,
        },
        None,
    )

    assert len(captured) == 1
    assert captured[0][1]["source_timestamp_s"] == pytest.approx(0.3)
    assert captured[0][1]["reception_monotonic_s"] > 0.0
    assert provider.get_logging_context() == {
        "onboard_sequence": 17,
        "external_sequence": None,
    }


class Provider:
    def start(self):
        pass

    def stop(self):
        pass

    def get_state(self):
        return state()

    def get_logging_context(self):
        return {"onboard_sequence": None, "external_sequence": None}


class Adapter:
    def send(self, value):
        del value

    def stop(self):
        pass


class OneStepProgram:
    def __init__(self, error: BaseException | None = None):
        self.updates = 0
        self.error = error

    def initialize(self, initial_state, start_time):
        del initial_state, start_time

    def is_complete(self, current_time):
        del current_time
        return self.updates >= 1

    def get_phase_name(self, current_time):
        del current_time
        return "TEST"

    def update(self, state, current_time, dt):
        del state, current_time, dt
        if self.error is not None:
            raise self.error
        self.updates += 1
        return reference(), command()


def make_runner(logger, program):
    return UniversalRunner(
        state_provider=Provider(),
        command_adapter=Adapter(),
        flight_program=program,
        arm=lambda: None,
        disarm=lambda: None,
        config=UniversalRunnerConfig(control_frequency_hz=1000.0, arming_wait_s=0.0),
        run_logger=logger,
    )


def test_runner_integration_normal_shutdown_without_hardware(tmp_path) -> None:
    logger = FlightLogger(configuration(), tmp_path)
    make_runner(logger, OneStepProgram()).run()

    assert json.loads((logger.directory / "metadata.json").read_text())["status"] == "completed"
    assert len(rows(logger.directory / "flight.csv")) == 1


def test_runner_integration_abnormal_shutdown_preserves_partial_log(tmp_path) -> None:
    logger = FlightLogger(configuration(), tmp_path)
    runner = make_runner(logger, OneStepProgram(RuntimeError("simulated failure")))

    with pytest.raises(RuntimeError, match="simulated failure"):
        runner.run()

    metadata = json.loads((logger.directory / "metadata.json").read_text())
    assert metadata["status"] == "failed"
    assert "simulated failure" in metadata["termination_reason"]
    assert (logger.directory / "flight.csv").exists()
    assert any(event["event"] == "run_finished" for event in rows(logger.directory / "events.csv"))
