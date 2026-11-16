from __future__ import annotations

import math
import struct

import pytest
from cflib.crazyflie.commander import Commander

from velocity_control_framework.backends import CrazySimCommandAdapter
from velocity_control_framework.interfaces import BodyRateThrustCommand


class FakeParam:
    def __init__(self, *, confirm: bool = True, fail: bool = False) -> None:
        self.values = {
            "flightmode.stabModeRoll": "1",
            "flightmode.stabModePitch": "1",
            "flightmode.stabModeYaw": "1",
        }
        self.writes: list[tuple[str, str]] = []
        self.confirm = confirm
        self.fail = fail

    def set_value(self, name: str, value: str) -> None:
        if self.fail:
            raise KeyError(name)
        self.writes.append((name, value))
        if self.confirm:
            self.values[name] = value

    def get_value(self, name: str) -> str:
        return self.values[name]


class FakeCommander:
    def __init__(self, *, unlock_fails: bool = False) -> None:
        self.setpoints: list[tuple[float, float, float, int]] = []
        self.stop_calls = 0
        self.unlock_fails = unlock_fails

    def send_setpoint(self, roll: float, pitch: float, yaw: float, thrust: int) -> None:
        if self.unlock_fails and thrust == 0:
            raise OSError("send failed")
        self.setpoints.append((roll, pitch, yaw, thrust))

    def send_stop_setpoint(self) -> None:
        self.stop_calls += 1


class FakeSupervisor:
    def __init__(self) -> None:
        self.emergency_stops = 0

    def send_emergency_stop(self) -> None:
        self.emergency_stops += 1


class FakeCrazyflie:
    def __init__(self, *, param: FakeParam | None = None, unlock_fails: bool = False) -> None:
        self.param = param or FakeParam()
        self.commander = FakeCommander(unlock_fails=unlock_fails)
        self.supervisor = FakeSupervisor()


def test_initialization_sets_and_verifies_rate_modes() -> None:
    cf = FakeCrazyflie()

    adapter = CrazySimCommandAdapter(cf)

    assert cf.param.writes == [
        ("flightmode.stabModeRoll", "0"),
        ("flightmode.stabModePitch", "0"),
        ("flightmode.stabModeYaw", "0"),
    ]
    assert cf.commander.setpoints == []
    assert cf.commander.stop_calls == 0

    adapter.unlock()
    assert cf.commander.setpoints == [(0.0, 0.0, 0.0, 0)]


def test_normal_command_fails_closed_until_unlock() -> None:
    cf = FakeCrazyflie()
    adapter = CrazySimCommandAdapter(cf)

    with pytest.raises(RuntimeError, match="not initialized"):
        adapter.send(BodyRateThrustCommand(0.0, 0.0, 0.0, 0.0))

    assert cf.commander.setpoints == []


@pytest.mark.parametrize(
    ("rates", "expected"),
    [
        ((math.pi / 6, -math.pi / 4, math.pi / 3), (30.0, -45.0, 60.0)),
        ((-math.pi / 6, math.pi / 4, -math.pi / 3), (-30.0, 45.0, -60.0)),
    ],
)
def test_legacy_rpyt_rate_and_thrust_encoding(rates, expected) -> None:
    cf = FakeCrazyflie()
    adapter = CrazySimCommandAdapter(cf)
    adapter.unlock()

    adapter.send(BodyRateThrustCommand(*rates, thrust=0.5))

    roll, pitch, yaw, thrust = cf.commander.setpoints[-1]
    assert roll == pytest.approx(expected[0])
    assert pitch == pytest.approx(expected[1])
    assert yaw == pytest.approx(expected[2])
    assert thrust == 35000


@pytest.mark.parametrize(
    ("normalized", "raw"),
    [(0.0, 10001), (0.5, 35000), (1.0, 60000)],
)
def test_thrust_scaling_matches_cflib_manual_mapping(normalized, raw) -> None:
    assert CrazySimCommandAdapter._encode_thrust(normalized) == raw


def test_installed_cflib_legacy_packet_negates_pitch_only() -> None:
    packets = []
    transport = type("Transport", (), {"send_packet": lambda self, packet: packets.append(packet)})()
    commander = Commander(transport)

    commander.send_setpoint(10.0, -20.0, 30.0, 10001)

    roll, encoded_pitch, yaw, thrust = struct.unpack("<fffH", bytes(packets[0].data))
    assert roll == pytest.approx(10.0)
    assert encoded_pitch == pytest.approx(20.0)
    assert yaw == pytest.approx(30.0)
    assert thrust == 10001


def test_unlock_minimum_normal_command_and_stop_are_distinct_encodings() -> None:
    cf = FakeCrazyflie()
    adapter = CrazySimCommandAdapter(cf)
    adapter.unlock()
    adapter.send(BodyRateThrustCommand.zero())
    adapter.stop()

    assert cf.commander.setpoints == [
        (0.0, 0.0, 0.0, 0),
        (0.0, 0.0, 0.0, 10001),
    ]
    assert cf.commander.stop_calls == 1


def test_stop_and_emergency_stop_are_distinct() -> None:
    cf = FakeCrazyflie()
    adapter = CrazySimCommandAdapter(cf)
    adapter.unlock()

    adapter.stop()
    adapter.emergency_stop()

    assert cf.commander.setpoints == [(0.0, 0.0, 0.0, 0)]
    assert cf.commander.stop_calls == 1
    assert cf.supervisor.emergency_stops == 1


def test_rate_mode_failure_sends_stop_and_does_not_unlock() -> None:
    cf = FakeCrazyflie(param=FakeParam(fail=True))

    with pytest.raises(RuntimeError, match="configure and verify"):
        CrazySimCommandAdapter(cf)

    assert cf.commander.stop_calls == 1
    assert cf.commander.setpoints == []


def test_unconfirmed_rate_mode_times_out_and_sends_stop() -> None:
    cf = FakeCrazyflie(param=FakeParam(confirm=False))
    ticks = iter((0.0, 0.0, 2.0))

    with pytest.raises(RuntimeError, match="did not confirm"):
        CrazySimCommandAdapter(
            cf, verification_timeout_s=1.0, poll_interval_s=0.0,
            clock=lambda: next(ticks), sleeper=lambda _: None,
        )

    assert cf.commander.stop_calls == 1
    assert cf.commander.setpoints == []


def test_unlock_failure_sends_stop() -> None:
    cf = FakeCrazyflie(unlock_fails=True)
    adapter = CrazySimCommandAdapter(cf)

    with pytest.raises(RuntimeError, match="unlock"):
        adapter.unlock()

    assert cf.commander.stop_calls == 1
