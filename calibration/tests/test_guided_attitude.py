import csv
import json

from calibration.config import CalibrationConfig
from calibration.guided_attitude import GUIDED_PHASES, GuidedAttitudeRunner
from calibration.state import AltitudeState
from calibration.state_monitor import StateMonitorLogger, StateMonitorSettings


class Clock:
    def __init__(self):
        self.now = 50.0
    def __call__(self):
        return self.now
    def sleep(self, duration):
        self.now += duration


class GuidedHardware:
    """Read-only fake: no arm, disarm, or command API exists."""
    def __init__(self, clock):
        self.clock = clock
        self.start = clock()
    def state(self):
        elapsed = self.clock()-self.start
        phase = min(int(elapsed/5), 5)
        local = elapsed-phase*5
        active = 1 <= local < 4
        roll = (-0.2 if phase == 1 else 0.2 if phase == 2 else 0) if active else 0
        pitch = 0.2 if phase == 3 and active else 0
        yaw = (0.3 if phase == 4 else -0.3 if phase == 5 else 0) if active else 0
        return AltitudeState(0, 0, roll, pitch, yaw, self.clock())
    def supervisor_status(self):
        return {"can_be_armed": True, "can_fly": False, "is_armed": False,
                "is_auto_armed": False, "is_flying": False, "is_tumbled": False,
                "is_locked": False, "is_crashed": False, "hl_control_active": False}


def test_guided_sequence_marks_phases_prompts_and_matches_signs(tmp_path):
    clock = Clock()
    config = CalibrationConfig(output_directory=str(tmp_path))
    settings = StateMonitorSettings(frequency_hz=10, duration_s=30)
    logger = StateMonitorLogger(tmp_path, config, settings, mode="attitude-sign-monitor")
    prompts = []
    report = GuidedAttitudeRunner(GuidedHardware(clock), config, logger, settings,
                                  clock=clock, sleeper=clock.sleep, prompt=prompts.append).run()
    assert report["overall_result"] == "PASS"
    assert len(prompts) == len(GUIDED_PHASES)
    assert [item["measured_sign"] for item in report["phase_results"][1:]] == [
        "negative", "positive", "positive", "positive", "negative"
    ]
    assert all(item["returned_to_neutral"] for item in report["phase_results"][1:])
    with (logger.directory/"state_telemetry.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert {row["phase"] for row in rows} == {phase.name for phase in GUIDED_PHASES}
    assert all(row["physical_action"] for row in rows)
    assert json.loads((logger.directory/"attitude_sign_summary.json").read_text())["overall_result"] == "PASS"
    assert (logger.directory/"state_response.svg").exists()


def test_guided_sequence_never_exposes_a_flight_command_path(tmp_path):
    clock = Clock(); hardware = GuidedHardware(clock)
    config = CalibrationConfig(output_directory=str(tmp_path))
    settings = StateMonitorSettings(frequency_hz=1, duration_s=30)
    logger = StateMonitorLogger(tmp_path, config, settings, mode="attitude-sign-monitor")
    GuidedAttitudeRunner(hardware, config, logger, settings, clock=clock, sleeper=clock.sleep,
                         prompt=lambda _: None).run()
    assert not hasattr(hardware, "arm")
    assert not hasattr(hardware, "send")
