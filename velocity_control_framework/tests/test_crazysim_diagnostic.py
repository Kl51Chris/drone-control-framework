from __future__ import annotations

import pytest

from tools.diagnose_crazysim_backend import (
    CRAZYSIM_URI,
    inspect_connected,
    validate_crazysim_uri,
)
from velocity_control_framework.backends import CrazySimSupervisorAdapter


class FakeSupervisor:
    def __init__(self) -> None:
        self.can_be_armed = True
        self.can_fly = False
        self.is_armed = False
        self.is_flying = False
        self.is_tumbled = False
        self.is_locked = False
        self.is_crashed = False
        self.hl_control_active = False
        self.emergency_stops = 0

    def read_bitfield(self) -> int:
        return 1

    def send_emergency_stop(self) -> None:
        self.emergency_stops += 1


class FakeParam:
    def __init__(self) -> None:
        self.values = {
            "flightmode.stabModeRoll": "1",
            "flightmode.stabModePitch": "1",
            "flightmode.stabModeYaw": "1",
        }
        self.writes = []

    def get_value(self, name: str) -> str:
        return self.values[name]

    def set_value(self, name: str, value: str) -> None:
        self.writes.append((name, value))
        self.values[name] = value


class FakeCommander:
    def __init__(self) -> None:
        self.setpoints = []
        self.stop_calls = 0

    def send_setpoint(self, *values) -> None:
        self.setpoints.append(values)

    def send_stop_setpoint(self) -> None:
        self.stop_calls += 1


class FakeCrazyflie:
    def __init__(self) -> None:
        self.supervisor = FakeSupervisor()
        self.param = FakeParam()
        self.commander = FakeCommander()


def command_state(_cf) -> dict[str, float]:
    return {"stabilizer.thrust": 0.0}


def test_diagnostic_uri_is_restricted_to_exact_crazysim_endpoint() -> None:
    validate_crazysim_uri(CRAZYSIM_URI)
    for invalid in ("radio://0/80/2M", "udp://127.0.0.1:19851", "udp://localhost:19850"):
        with pytest.raises(ValueError, match="restricted"):
            validate_crazysim_uri(invalid)


def test_default_diagnostic_is_read_only() -> None:
    cf = FakeCrazyflie()

    result = inspect_connected(cf, command_state)

    assert result["rate_modes"] == {
        "flightmode.stabModeRoll": 1,
        "flightmode.stabModePitch": 1,
        "flightmode.stabModeYaw": 1,
    }
    assert result["stop_setpoint_sent"] is False
    assert cf.param.writes == []
    assert cf.commander.setpoints == []
    assert cf.commander.stop_calls == 0


def test_diagnostic_can_use_firmware_log_supervisor_status() -> None:
    cf = FakeCrazyflie()
    cf.supervisor.can_be_armed = False
    actual = {
        "bitfield": 1,
        "can_be_armed": True,
        "can_fly": False,
        "is_armed": False,
        "is_flying": False,
        "is_tumbled": False,
        "is_locked": False,
        "is_crashed": False,
        "hl_control_active": False,
    }
    result = inspect_connected(cf, command_state, status_reader=lambda: actual)
    assert result["supervisor_before"]["raw_bitfield_decimal"] == 1
    assert result["supervisor_before"]["raw_bitfield_hex"] == "0x0001"
    assert result["supervisor_before"]["decoded_flags"]["can_be_armed"] is True
    assert result["supervisor_before"]["decoded_flags"]["is_armed"] is False


def test_rate_mode_configuration_is_explicit_and_does_not_unlock() -> None:
    cf = FakeCrazyflie()

    result = inspect_connected(cf, command_state, configure_rate_modes=True)

    assert result["rate_modes"] == {
        "flightmode.stabModeRoll": 0,
        "flightmode.stabModePitch": 0,
        "flightmode.stabModeYaw": 0,
    }
    assert cf.commander.setpoints == []
    assert cf.commander.stop_calls == 0


def test_stop_verification_requires_explicit_rate_mode_opt_in() -> None:
    cf = FakeCrazyflie()
    with pytest.raises(ValueError, match="requires"):
        inspect_connected(cf, command_state, verify_stop=True)

    result = inspect_connected(
        cf, command_state, configure_rate_modes=True, verify_stop=True
    )
    assert result["stop_setpoint_sent"] is True
    assert cf.commander.stop_calls == 1
    assert cf.commander.setpoints == []


@pytest.mark.parametrize(
    ("bitfield", "is_armed", "is_flying"),
    [
        (0x0001, False, False),
        (0x000B, True, False),
        (0x001B, True, True),
    ],
)
def test_read_only_diagnostic_reports_disarmed_and_armed_bitfields(
    bitfield, is_armed, is_flying
) -> None:
    cf = FakeCrazyflie()
    status = CrazySimSupervisorAdapter.decode(bitfield)
    result = inspect_connected(cf, command_state, status_reader=lambda: status)
    report = result["supervisor_before"]
    assert report["raw_bitfield_decimal"] == bitfield
    assert report["raw_bitfield_hex"] == f"0x{bitfield:04X}"
    assert report["decoded_flags"]["can_be_armed"] is True
    assert report["decoded_flags"]["is_armed"] is is_armed
    assert report["decoded_flags"]["is_flying"] is is_flying
    assert result["operation_refused"] is None
    assert cf.param.writes == []
    assert cf.commander.stop_calls == 0


@pytest.mark.parametrize(
    ("bitfield", "trigger"),
    [(0x000B, "is_armed"), (0x0019, "is_flying")],
)
def test_armed_or_flying_firmware_refuses_mutation_after_reporting(bitfield, trigger) -> None:
    cf = FakeCrazyflie()
    status = CrazySimSupervisorAdapter.decode(bitfield)
    result = inspect_connected(
        cf,
        command_state,
        configure_rate_modes=True,
        status_reader=lambda: status,
    )

    assert result["operation_refused"]["triggered_by"] == [trigger]
    assert result["supervisor_before"]["raw_bitfield_decimal"] == bitfield
    assert cf.param.writes == []
    assert cf.commander.stop_calls == 0
