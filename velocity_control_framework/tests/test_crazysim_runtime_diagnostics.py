from __future__ import annotations

import pytest

from tools.validate_crazysim_actuators import (
    MAX_DURATION_S,
    MAX_NORMALIZED_THRUST,
    MAX_RATE_RAD_S,
    build_command,
    safe_cleanup,
    summarize_axis_response,
    validate_safe_startup,
    validate_duration,
    validate_authorized_command_phase,
)
from tools.validate_crazysim_reconnect import run_two_cycles
from tools.validate_crazysim_emergency_stop import execute_connected


@pytest.mark.parametrize(
    ("axis", "rate", "expected"),
    [
        ("roll", 0.2, (0.2, 0.0, 0.0)),
        ("roll", -0.2, (-0.2, 0.0, 0.0)),
        ("pitch", 0.2, (0.0, 0.2, 0.0)),
        ("pitch", -0.2, (0.0, -0.2, 0.0)),
        ("yaw", 0.2, (0.0, 0.0, 0.2)),
        ("yaw", -0.2, (0.0, 0.0, -0.2)),
    ],
)
def test_axis_command_generation(axis, rate, expected) -> None:
    command = build_command(axis, rate, 0.1)
    assert (command.roll_rate, command.pitch_rate, command.yaw_rate) == expected


@pytest.mark.parametrize(
    ("axis", "rate", "thrust"),
    [
        ("roll", MAX_RATE_RAD_S + 0.001, 0.0),
        ("pitch", -MAX_RATE_RAD_S - 0.001, 0.0),
        ("none", 0.01, 0.0),
        ("yaw", 0.0, MAX_NORMALIZED_THRUST + 0.001),
        ("yaw", 0.0, -0.001),
    ],
)
def test_axis_and_thrust_limits_fail_closed(axis, rate, thrust) -> None:
    with pytest.raises(ValueError):
        build_command(axis, rate, thrust)


def test_duration_is_positive_and_bounded() -> None:
    validate_duration(MAX_DURATION_S)
    for invalid in (0.0, -0.1, MAX_DURATION_S + 0.001):
        with pytest.raises(ValueError):
            validate_duration(invalid)


def startup_status(**changes):
    status = {
        "can_be_armed": True,
        "is_armed": False,
        "is_auto_armed": False,
        "can_fly": False,
        "is_flying": False,
        "is_tumbled": False,
        "is_locked": False,
        "is_crashed": False,
        "hl_control_active": False,
    }
    status.update(changes)
    return status


def inactive_outputs(**changes):
    outputs = {
        "motor.m1": 0.0,
        "motor.m2": 0.0,
        "motor.m3": 0.0,
        "motor.m4": 0.0,
        "stabilizer.thrust": 0.0,
        "ctrltarget.roll": 0.0,
        "ctrltarget.pitch": 0.0,
        "ctrltarget.yaw": 0.0,
    }
    outputs.update(changes)
    return outputs


def test_clean_disarmed_startup_is_supported() -> None:
    assert validate_safe_startup(startup_status(), inactive_outputs()) == "disarmed"


def test_verified_safe_auto_armed_startup_is_supported() -> None:
    status = startup_status(
        can_be_armed=False, is_armed=True, is_auto_armed=True, can_fly=True
    )
    assert validate_safe_startup(status, inactive_outputs()) == "safe_auto_armed"


@pytest.mark.parametrize(
    "changes",
    [
        {"is_armed": True},
        {"is_armed": True, "can_fly": True},
        {"is_armed": True, "is_auto_armed": True, "can_fly": False},
        {"is_flying": True},
        {"is_crashed": True},
        {"is_tumbled": True},
        {"is_locked": True},
        {"hl_control_active": True},
    ],
)
def test_unexpected_or_faulted_startup_fails_closed(changes) -> None:
    with pytest.raises(RuntimeError):
        validate_safe_startup(startup_status(**changes), inactive_outputs())


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("motor.m1", 1.0),
        ("stabilizer.thrust", 0.01),
        ("ctrltarget.roll", 0.01),
    ],
)
def test_unexpected_motor_thrust_or_controller_output_fails_closed(name, value) -> None:
    status = startup_status(
        can_be_armed=False, is_armed=True, is_auto_armed=True, can_fly=True
    )
    with pytest.raises(RuntimeError, match="not inactive"):
        validate_safe_startup(status, inactive_outputs(**{name: value}))


@pytest.mark.parametrize("is_flying", [False, True])
def test_authorized_command_phase_allows_firmware_motor_activity_flag(is_flying) -> None:
    status = startup_status(
        can_be_armed=False,
        is_armed=True,
        is_auto_armed=True,
        can_fly=True,
        is_flying=is_flying,
    )
    validate_authorized_command_phase(status)


@pytest.mark.parametrize(
    "changes",
    [
        {"is_armed": False},
        {"can_fly": False},
        {"is_tumbled": True},
        {"is_locked": True},
        {"is_crashed": True},
        {"hl_control_active": True},
    ],
)
def test_authorized_command_phase_retains_supervisor_fault_checks(changes) -> None:
    status = startup_status(
        can_be_armed=False,
        is_armed=True,
        is_auto_armed=True,
        can_fly=True,
    )
    status.update(changes)
    with pytest.raises(RuntimeError, match="authorized command"):
        validate_authorized_command_phase(status)


def test_axis_response_summary_compares_command_state_and_motion_signs() -> None:
    command = build_command("yaw", -0.1, 0.1)
    records = [
        {"state": {"yaw": 0.02, "r": -0.03}},
        {"state": {"yaw": 0.01, "r": -0.08}},
    ]
    summary = summarize_axis_response(records, command)
    assert summary["axis"] == "yaw"
    assert summary["signed_peak_rate_rad_s"] == -0.08
    assert summary["peak_rate_has_command_sign"] is True
    assert summary["attitude_change_rad"] == pytest.approx(-0.01)


class CleanupSupervisor:
    def __init__(self, events) -> None:
        self.events = events

    def send_arming_request(self, armed: bool) -> None:
        self.events.append(("arm", armed))


class CleanupCommander:
    def __init__(self, events) -> None:
        self.events = events

    def send_stop_setpoint(self) -> None:
        self.events.append(("stop", None))


class CleanupCrazyflie:
    def __init__(self, events) -> None:
        self.supervisor = CleanupSupervisor(events)
        self.commander = CleanupCommander(events)


def test_cleanup_always_stops_before_disarming_without_rearming() -> None:
    events = []
    errors = safe_cleanup(CleanupCrazyflie(events), adapter=None)
    assert errors == []
    assert events == [("stop", None), ("arm", False)]


def test_cleanup_uses_crazysim_supervisor_compatibility_disarm() -> None:
    events = []
    compatibility = CleanupSupervisor(events)
    errors = safe_cleanup(
        CleanupCrazyflie(events), adapter=None, supervisor=compatibility
    )
    assert errors == []
    assert events == [("stop", None), ("arm", False)]


def test_auto_armed_cleanup_stops_without_false_disarm_request() -> None:
    events = []
    errors = safe_cleanup(
        CleanupCrazyflie(events),
        adapter=None,
        supervisor=CleanupSupervisor(events),
        disarm=False,
    )
    assert errors == []
    assert events == [("stop", None)]


def test_two_successful_cycles_are_reported_as_supported() -> None:
    calls = []

    def cycle(uri, number):
        calls.append((uri, number))
        return {"cycle": number, "connected": True}

    result = run_two_cycles(
        "udp://127.0.0.1:19850", cycle_runner=cycle,
        endpoint_probe=lambda: pytest.fail("probe should not run"),
        sleeper=lambda _: None,
    )
    assert result["classification"] == "reconnect_supported"
    assert [number for _, number in calls] == [1, 2]


@pytest.mark.parametrize(
    ("alive", "classification"),
    [
        (True, "bridge_alive_but_cflib_session_failed"),
        (False, "simulator_or_udp_bridge_not_running"),
    ],
)
def test_second_cycle_failure_distinguishes_bridge_liveness(alive, classification) -> None:
    def cycle(_uri, number):
        if number == 2:
            raise TimeoutError("second session")
        return {"cycle": number, "connected": True}

    result = run_two_cycles(
        "udp://127.0.0.1:19850", cycle_runner=cycle,
        endpoint_probe=lambda: alive,
        sleeper=lambda _: None,
    )
    assert result["classification"] == classification
    assert result["cycles"][1]["udp_bridge_responded_after_failure"] is alive


class EmergencySupervisor:
    def __init__(self, events) -> None:
        self.events = events

    def send_emergency_stop(self) -> None:
        self.events.append(("emergency_stop", None))

    def send_arming_request(self, armed: bool) -> None:
        self.events.append(("arm", armed))


class EmergencyCrazyflie:
    def __init__(self, events) -> None:
        self.commander = CleanupCommander(events)
        self.supervisor = EmergencySupervisor(events)


def test_emergency_stop_diagnostic_has_separate_explicit_cleanup(monkeypatch) -> None:
    events = []
    statuses = iter(
        [
            {"is_armed": False, "is_flying": False},
            {"is_armed": False, "is_flying": False, "is_locked": True},
        ]
    )
    monkeypatch.setattr(
        "tools.validate_crazysim_emergency_stop.supervisor_status",
        lambda _cf: next(statuses),
    )
    result = execute_connected(EmergencyCrazyflie(events))
    assert result["supervisor_after"]["is_locked"] is True
    assert events == [
        ("stop", None),
        ("emergency_stop", None),
        ("stop", None),
        ("arm", False),
    ]


def test_emergency_stop_failure_still_stops_and_disarms(monkeypatch) -> None:
    events = []
    monkeypatch.setattr(
        "tools.validate_crazysim_emergency_stop.supervisor_status",
        lambda _cf: {"is_armed": False, "is_flying": False},
    )
    cf = EmergencyCrazyflie(events)

    def fail() -> None:
        events.append(("emergency_stop", None))
        raise RuntimeError("injected failure")

    cf.supervisor.send_emergency_stop = fail
    with pytest.raises(RuntimeError, match="injected failure"):
        execute_connected(cf)
    assert events[-2:] == [("stop", None), ("arm", False)]
