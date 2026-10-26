import pytest

from calibration.config import CalibrationConfig
from calibration.experiment import CalibrationRunner, Mode, reference_for
from calibration.state import AltitudeState


class FakeLogger:
    def __init__(self):
        self.closed = None

    def close(self, status, reason=""):
        self.closed = (status, reason)


class FakeHardware:
    def __init__(self, **status_overrides):
        self.arm_calls = 0
        self.disarm_calls = 0
        self.send_calls = []
        self.stop_calls = 0
        self.estop_calls = 0
        self.status = {
            "can_be_armed": True, "can_fly": True, "is_armed": False,
            "is_auto_armed": False, "is_flying": False, "is_tumbled": False,
            "is_locked": False, "is_crashed": False, "hl_control_active": False,
        }
        self.status.update(status_overrides)

    def state(self):
        return AltitudeState(0, 0, 0, 0, 0, 10)

    def supervisor_status(self):
        return self.status

    def arm(self):
        self.arm_calls += 1

    def disarm(self):
        self.disarm_calls += 1

    def send(self, *command):
        self.send_calls.append(command)

    def emergency_stop(self):
        self.stop_calls += 1

    def emergency_stop_available(self): return True
    def flow_deck_available(self): return True
    def protocol_version(self): return 12
    def supervisor_emergency_stop(self): self.estop_calls += 1


def test_takeoff_hover_landing_transitions():
    config = CalibrationConfig()
    p = config.profile
    up = p.takeoff_height_m / p.takeoff_rate_m_s
    down = p.takeoff_height_m / p.landing_rate_m_s
    assert reference_for(Mode.TAKEOFF_HOVER_LAND, 0, .1, config).phase == "TAKEOFF"
    assert reference_for(Mode.TAKEOFF_HOVER_LAND, up, .1, config).phase == "HOVER"
    assert reference_for(Mode.TAKEOFF_HOVER_LAND, up + p.hover_duration_s, .1, config).phase == "LANDING"
    assert reference_for(Mode.TAKEOFF_HOVER_LAND, up + p.hover_duration_s + down + p.settle_duration_s, .1, config).complete


def test_reference_profile_includes_consistent_vertical_velocity():
    config=CalibrationConfig();p=config.profile;up=p.takeoff_height_m/p.takeoff_rate_m_s
    assert reference_for(Mode.CASCADED_5B,0,0,config).vertical_velocity_m_s==p.takeoff_rate_m_s
    assert reference_for(Mode.CASCADED_5B,up,0,config).vertical_velocity_m_s==0
    assert reference_for(Mode.CASCADED_5B,up+p.hover_duration_s,0,config).vertical_velocity_m_s==-p.landing_rate_m_s


def test_p_modes_use_controlled_profile_and_complete_after_one_trial():
    config = CalibrationConfig()
    p = config.profile
    duration = p.takeoff_height_m / p.takeoff_rate_m_s + p.hover_duration_s + p.takeoff_height_m / p.landing_rate_m_s + p.settle_duration_s
    assert reference_for(Mode.P_ONLY, 0, 0, config).phase == "TAKEOFF"
    assert reference_for(Mode.P_ONLY, duration, 0, config).complete


def test_preflight_never_arms_or_sends_a_setpoint():
    hardware, logger = FakeHardware(), FakeLogger()
    CalibrationRunner(hardware, CalibrationConfig(), Mode.PREFLIGHT, logger, clock=lambda: 10).run()
    assert hardware.arm_calls == 0
    assert hardware.send_calls == []
    assert logger.closed[0] == "valid"


@pytest.mark.parametrize("condition", ["is_armed", "is_auto_armed", "is_flying", "is_tumbled", "is_locked", "is_crashed", "hl_control_active"])
def test_preflight_rejects_unsafe_supervisor_states(condition):
    hardware = FakeHardware(**{condition: True})
    with pytest.raises(RuntimeError, match="supervisor rejects"):
        CalibrationRunner(hardware, CalibrationConfig(), Mode.PREFLIGHT, FakeLogger(), clock=lambda: 10).run()
    assert hardware.arm_calls == 0
    assert hardware.send_calls == []


def test_arm_failure_still_executes_flight_shutdown():
    hardware, logger = FakeHardware(), FakeLogger()
    def fail_arm():
        hardware.arm_calls += 1
        raise RuntimeError("arm failed")
    hardware.arm = fail_arm
    with pytest.raises(RuntimeError, match="arm failed"):
        CalibrationRunner(hardware, CalibrationConfig(), Mode.PID, logger, clock=lambda: 10).run()
    assert hardware.stop_calls == 1
    assert hardware.disarm_calls == 1
    assert logger.closed[0] == "invalid"


def test_state_monitor_cannot_fall_through_to_flight_runner():
    hardware = FakeHardware()
    with pytest.raises(ValueError, match="non-flight monitor runner"):
        CalibrationRunner(hardware, CalibrationConfig(), Mode.STATE_MONITOR, FakeLogger()).run()
    assert hardware.arm_calls == 0
    assert hardware.send_calls == []


def test_guided_monitor_cannot_fall_through_to_flight_runner():
    hardware = FakeHardware()
    with pytest.raises(ValueError, match="non-flight monitor runner"):
        CalibrationRunner(hardware, CalibrationConfig(), Mode.ATTITUDE_SIGN_MONITOR, FakeLogger()).run()
    assert hardware.arm_calls == 0
    assert hardware.send_calls == []
