from __future__ import annotations

import pytest

from velocity_control_framework.backends import CrazySimSupervisorAdapter


class FakePlatform:
    def __init__(self, version: int = 7) -> None:
        self.version = version

    def get_protocol_version(self) -> int:
        return self.version


class FakeParam:
    def __init__(self) -> None:
        self.value = "0"
        self.writes: list[tuple[str, str]] = []

    def set_value(self, name: str, value: str) -> None:
        self.writes.append((name, value))
        self.value = value

    def get_value(self, name: str) -> str:
        assert name == "system.arm"
        return self.value


class FakeCrazyflie:
    def __init__(self, version: int = 7) -> None:
        self.platform = FakePlatform(version)
        self.param = FakeParam()


def sampled_adapter(bitfield: int, *, clock=lambda: 1.0):
    cf = FakeCrazyflie()
    adapter = CrazySimSupervisorAdapter(cf, clock=clock)
    adapter._on_data(0, {"supervisor.info": bitfield}, None)
    return cf, adapter


def test_decodes_firmware_supervisor_info_layout() -> None:
    status = CrazySimSupervisorAdapter.decode(0x00AB)
    assert status == {
        "bitfield": 0x00AB,
        "can_be_armed": True,
        "is_armed": True,
        "is_auto_armed": False,
        "can_fly": True,
        "is_flying": False,
        "is_tumbled": True,
        "is_locked": False,
        "is_crashed": True,
        "hl_control_active": False,
    }


def test_zero_is_real_unsafe_only_when_received_from_firmware_log() -> None:
    _, adapter = sampled_adapter(0)
    assert adapter.status()["can_be_armed"] is False


def test_status_reports_fresh_sample_evidence() -> None:
    _, adapter = sampled_adapter(0x0001)
    status, sample = adapter.status_with_sample()
    assert status["bitfield"] == 1
    assert sample == {"age_s": 0.0, "fresh": True, "maximum_age_s": 0.2}


def test_adapter_is_restricted_to_known_crazysim_protocol() -> None:
    with pytest.raises(RuntimeError, match="requires CRTP protocol version 7"):
        CrazySimSupervisorAdapter(FakeCrazyflie(version=12))


def test_legacy_arm_parameter_requires_firmware_can_be_armed() -> None:
    cf, adapter = sampled_adapter(0)
    with pytest.raises(RuntimeError, match="refusing legacy CrazySim arm"):
        adapter.send_arming_request(True)
    assert cf.param.writes == []


def test_legacy_arm_parameter_is_written_and_read_back() -> None:
    cf, adapter = sampled_adapter(0x0001)
    adapter.send_arming_request(True)
    adapter.send_arming_request(False)
    assert cf.param.writes == [("system.arm", "1"), ("system.arm", "0")]


def test_legacy_parameter_does_not_claim_to_disarm_compile_time_auto_arm() -> None:
    cf, adapter = sampled_adapter(0x000E)
    with pytest.raises(RuntimeError, match="cannot be cleared"):
        adapter.send_arming_request(False)
    assert cf.param.writes == []


def test_arm_readback_mismatch_attempts_fail_closed_disarm() -> None:
    cf, adapter = sampled_adapter(0x0001)
    cf.param.get_value = lambda _name: "0"
    with pytest.raises(RuntimeError, match="readback mismatch"):
        adapter.send_arming_request(True)
    assert cf.param.writes == [("system.arm", "1"), ("system.arm", "0")]


def test_stale_supervisor_telemetry_fails_closed() -> None:
    times = iter((1.0, 1.25))
    _, adapter = sampled_adapter(0x0001, clock=lambda: next(times))
    with pytest.raises(RuntimeError, match="stale"):
        adapter.status()
